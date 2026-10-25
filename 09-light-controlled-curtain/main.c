#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdbool.h>

// ==================================================
// Configuration & Constants
// ==================================================

// --- Motor Pin Mapping ---
// Because we control motor by timer0, PB4 supplies PWM signal.
#define AIN1    PB7
#define AIN2    PB6
#define STBY    PB5
#define PWMA    PB4

// --- Motor Direction Commands ---
// Rationale: abstracting “what pins to drive” to avoid using a magic number.
// BRAKE vs STOP is deliberately separated; we use BRAKE when changing the dynamic state into others, and we use STOP for persisting stop state.
#define CW      0x10    // Clockwise rotation
#define CCW     0x01    // CounterCW
#define STOP    0x00    // Coast
#define BRAKE   0x11    // Short brake 

// --- ADC Threshold (10-bit) ---
// "These values were perfect for implementing the system!" - the writer
#define CDS_10   341
#define CDS_100  731

// --- Curtain Increment Command ---
// Tuned for fsm update frequency to implement moving curtain. 
// If you increase this macro, the time for moving curtain is increased.
#define CURTAIN_STEP 100


// ==================================================
// System Flags, Events, and States
// ==================================================
// Why a 32-bit flag? Well… in real projects, system flags are often defined as 32-bit for scalability.
// Same reason for event, too.

typedef enum {
	FLAG_FSM_UPDATE         = (1u << 0), // set every 100 ms tick (10Hz)
	FLAG_CURTAIN_OPENED     = (1u << 1), // limit flag
	FLAG_CURTAIN_CLOSED     = (1u << 2)  // 
} SysFlags;

typedef uint32_t SysFlagMask;

typedef enum {
	EV_NONE     = 0,
	EV_LUX_HIGH = (1u << 0),
	EV_LUX_LOW  = (1u << 1)
} SysEvents;

typedef uint32_t SysEventMask;

typedef enum {
	IDLE_STATE = 0,
	OPEN_STATE,
	CLOSE_STATE
} State_t;


// ==================================================
// Global Variables
// ==================================================
// Is this modified in ISR? Yes -> volatile, No -> non-v~.

volatile SysFlagMask flags  = 0;
volatile SysEventMask events = 0;

static State_t curr_state = IDLE_STATE;
static State_t next_state = IDLE_STATE;
static State_t prev_state = IDLE_STATE;

static volatile uint16_t adc_result;
static int curtain_position = 0;  // [-2500 .. +2499]
/*
	At first, I decided this range assuming a 1 ms FSM tick.
	Later, but, I switched the tick to 0.1 s after the code was done,
	so... yeah, the range is unnecessarily big now :)
*/

// ==================================================
// Interrupt Service Routines
// ==================================================
// Design intent: 'Minimalism'

ISR(TIMER3_COMPA_vect)
{
	// 1 ms base tick → request FSM update every 100 ms. (10Hz)
	// Why 100 ms? Human-scale motion feels smooth enough — no need to run at 1 kHz.
	// Plus, it saves power, energy, and CPU resources anyway.
	static uint8_t cnt_100ms = 0;
	if (cnt_100ms == 99) {
		flags |= FLAG_FSM_UPDATE;  // main() consume this.
		cnt_100ms = 0;
		} else {
		cnt_100ms++;
	}
}

ISR(TIMER3_COMPB_vect)
{
	// We thought a lot about how to set OCR3B — details are explained later in the Timer3 init part.
	ADCSRA |= (1<<ADSC);  // Start ADC conversion
}

ISR(ADC_vect)
{
	ADCSRA |= (1<<ADIF);  // Clear ADC interrupt flag

	uint8_t lo = ADCL;    // Read ADCL first (hardware requirement)(<- but why? I don't well about it -the writer)
	uint8_t hi = ADCH;    // Then ADCH
	adc_result = ((uint16_t)hi << 8) | lo;

	// These events are consumed by decide_next()
	if (adc_result > CDS_100)	
	events |= EV_LUX_HIGH;
	else if (adc_result < CDS_10)
	events |= EV_LUX_LOW;
	else
	events &= ~(EV_LUX_HIGH | EV_LUX_LOW);
}


// ==================================================
// Initialization Functions
// ==================================================
// cli(), sei() is written in main().

static void init_motor(void)
{
	DDRB  = (1<<PB4)|(1<<PB5)|(1<<PB6)|(1<<PB7);    // Set motor pins as output

	// Fast PWM, non-inverting, prescale 1024.
	// I don’t think the PWM prescaler really matters for motor speed control — only the duty ratio does.
	TCCR0 = (1<<WGM01)|(1<<WGM00)|(1<<COM01)|(1<<CS02)|(1<<CS01)|(1<<CS00);
	OCR0  = 0x00;                                
}

static void init_timer(void)
{
	// CTC, COMPA = 1kHz, COMPB is same freq but 240 µs earlier.
	/*
	That value mattered a lot — we really thought about it.
	We didn’t want the ADC to update too often, but still wanted the FSM to use a fresh result.
	So we calculated the ADC conversion period, added some guard time,
	and made it run at the same frequency as the FSM, but about 240 µs earlier.
	*/

	TCCR3A = 0;
	TCCR3B = (1<<WGM32)|(1<<CS31)|(1<<CS30);        // Prescale 64
	ETIMSK = (1<<OCIE3A)|(1<<OCIE3B);               // Enable interrupts
	TCNT3  = 0;
	ETIFR |= (1<<OCF3A)|(1<<OCF3B);                 // Clear pending

	OCR3A = 250 - 1;                                // 16 MHz / 64 / 250 = 1 kHz (1 ms)
	OCR3B = OCR3A - 60;                             // ~240 µs pre-trigger for ADC
}

static void init_adc(void)
{
	DDRF  &= ~(1<<PF0);              // ADC0 as input
	ADMUX  = (1<<REFS0);             // AVCC, right adjusted, ADC0
	ADCSRA = (1<<ADEN)|(1<<ADIE);    // Enable ADC and interrupt
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // Prescaler 128(125 kHz)
	// ADC conversion is triggered by the ISR.
}


// ==================================================
// Motor Control
// ==================================================

static inline void ctrl_motor(uint8_t direction, int speed)
{
	switch (direction)
	{
		case CW:
		PORTB = (1<<AIN1)|(1<<STBY);
		OCR0  = speed;           // duty = speed (0..255)
		break;

		case CCW:
		PORTB = (1<<AIN2)|(1<<STBY);
		OCR0  = speed;
		break;

		case STOP:
		PORTB = (1<<STBY);
		OCR0  = 255;             
		break;

		case BRAKE: // "끼-익!"
		PORTB = (1<<AIN1)|(1<<AIN2)|(1<<STBY);
		break;
	}
}


// ==================================================
// Curtain Logic
// ==================================================
// We use ATOMIC BLOCK to avoid race condition by ISR and fsm.
// So, every flags and events which are able to be modified in ISR are read/written in ATOMIC BLOCK.
/*
	Why atomicity? Otherwise, if someone reads the lower byte while the upper byte changes, the read value can be unreliable. 
	We use 8-bit MCU, so atomicity is effective in only 8-bit data read/write.
*/

static inline void update_limit_flags_from_position(void)
{
	if (curtain_position >= 2499) {
		curtain_position = 2499;  // clamp to max
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
			flags |=  FLAG_CURTAIN_OPENED;
			flags &= ~FLAG_CURTAIN_CLOSED;
		}
		} else if (curtain_position <= -2500) {
		curtain_position = -2500; // clamp to min
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
			flags |=  FLAG_CURTAIN_CLOSED;
			flags &= ~FLAG_CURTAIN_OPENED;
		}
		} else {
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
			flags &= ~(FLAG_CURTAIN_OPENED | FLAG_CURTAIN_CLOSED);
		}
	}
}

static inline void move_curtain(State_t s)
{
	SysFlagMask f;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ f = flags; }

	switch (s) {
		case OPEN_STATE:
		if (!(f & FLAG_CURTAIN_OPENED)) {
			ctrl_motor(CW, 120);                 // "There is no significant reason for that value"
			curtain_position += CURTAIN_STEP;     
			update_limit_flags_from_position();
			} else {
			ctrl_motor(STOP, 0);				 // Already opened
		}
		break;

        case CLOSE_STATE:
        if (!(f & FLAG_CURTAIN_CLOSED)) {
	        ctrl_motor(CCW, 120);
	        curtain_position -= CURTAIN_STEP;
	        update_limit_flags_from_position();
	        } else {
	        ctrl_motor(STOP, 0);				// Already closed
        }
        break;

        case IDLE_STATE:
        ctrl_motor(STOP, 0);
        break;
        }
        }


// ==================================================
// FSM Actions & Logic
// ==================================================
// Philosophy: split entry/during/exit for action clarity.

static void on_entry(State_t s)
{
	SysFlagMask f;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){ f = flags; }

	switch (s) {
		case IDLE_STATE:
		ctrl_motor(STOP, 0);
		break;
		case OPEN_STATE:
		if (!(f & FLAG_CURTAIN_OPENED)) ctrl_motor(CW, 50); // soft start: smooth motor start
		break;
		case CLOSE_STATE:
		if (!(f & FLAG_CURTAIN_CLOSED)) ctrl_motor(CCW, 50);
		break;
	}
}	

static void on_during(State_t s)
{
	// It was really hard to figure out the current state during debugging, so we added LED actions.
	switch (s) {
		case IDLE_STATE:
		move_curtain(IDLE_STATE);
		PORTA = 0x02;  
		break;
		case OPEN_STATE:
		move_curtain(OPEN_STATE);
		PORTA = 0x01; 
		break;
		case CLOSE_STATE:
		move_curtain(CLOSE_STATE);
		PORTA = 0x04;
		break;
	}
}

static void on_exit(State_t s)
{
	// Brake to clear the motor move.
	switch (s) {
		case OPEN_STATE:
		case CLOSE_STATE:
		ctrl_motor(BRAKE, 0);
		break;
		case IDLE_STATE:
		break;
	}
}

static State_t decide_next(void)
{
	// The next state is chosen only based on events.
	// Actually, not intended but the result was good.
	bool open, close;
	SysFlagMask f;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		open  = (events & EV_LUX_HIGH);
		close = (events & EV_LUX_LOW);
		f     = flags;
	}

	if (open) {
		if (f & FLAG_CURTAIN_OPENED) return IDLE_STATE;  // If opened/closed already, switch to IDLE
		return OPEN_STATE;
	}
	if (close) {
		if (f & FLAG_CURTAIN_CLOSED) return IDLE_STATE;  
		return CLOSE_STATE;
	}
	return IDLE_STATE;  
}


void fsm_step(void)
{
	if (curr_state != prev_state) {		// Check whether the state is transitioned.
		on_exit(prev_state);
		on_entry(curr_state);
		prev_state = curr_state;
	}

	on_during(curr_state);
	next_state = decide_next();
	curr_state = next_state;
}


// ==================================================
// Main Loop
// ==================================================

int main(void)
{
	DDRA = 0x07;  // LED output for state feedback

	cli();
	init_motor();
	init_timer();
	init_adc();
	sei();

	for (;;) {
		bool do_step = false;

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
			if (flags & FLAG_FSM_UPDATE) {
				flags &= ~FLAG_FSM_UPDATE;
				do_step = true;
			}
		}

		if (do_step) {
			// The ADC conversion is already operated by our precise trigger timing calculation.
			fsm_step();
		}
	}
}