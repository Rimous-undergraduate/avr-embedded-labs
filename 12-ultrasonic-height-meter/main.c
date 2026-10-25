#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>
#include <util/atomic.h>

// ======================================
// Events and States Definition
// ======================================
typedef enum {
	EV_NONE             = 0,
	EV_BUTTON_PRESSED   = (1u << 0)
} SysEvents;

typedef uint8_t SysEventMask;
static volatile SysEventMask events = EV_NONE;

typedef enum {
	STATE_0 = 0,   
	STATE_1,       
	STATE_2        
} State_t;

static State_t curr_state      = STATE_0;
static State_t next_state      = STATE_0;
static bool    state_first_ent = true;

// ======================================
// Ultrasonic Module
// ======================================
#define ULSONIC_TRIG (1 << PE6)   // TRIG: PE6 
#define ULSONIC_ECHO (1 << PE7)   // ECHO: PE7 

// Timer1: F_CPU=16MHz, prescale=64 → 4us/tick
static void timer1_init(void) {
	TCCR1A = 0x00;
	TCCR1B = (1 << CS11) | (1 << CS10);   // prescale 64, normal 
	TCNT1  = 0;
}

// 10 us pulse for triggering TRIG of ultrasonic module
static void ulsonic_trig(void) {
	PORTE &= ~ULSONIC_TRIG;
	_delay_us(2);
	PORTE |= ULSONIC_TRIG;
	_delay_us(10);
	PORTE &= ~ULSONIC_TRIG;
}

// Initialize PIN INT7 
#define INT7_ISC_MASK   ((1 << ISC70) | (1 << ISC71))

static inline void int7_set_rising(void) {	// Rising Edge Active
	EICRB = (EICRB & ~INT7_ISC_MASK) | (1 << ISC71) | (1 << ISC70);
}

static inline void int7_set_falling(void) {	// Falling Edge Active
	EICRB = (EICRB & ~INT7_ISC_MASK) | (1 << ISC71);    
}

typedef enum {	
	DETECT_RISING_EDGE = 0,
	DETECT_FALLING_EDGE
} Detect_State_t;

static volatile Detect_State_t echo_state   = DETECT_RISING_EDGE;
static volatile uint16_t       start_time   = 0;
static volatile uint16_t       ulsonic_ticks = 0;   // ECHO high duration (ticks)
static volatile uint8_t        ulsonic_done  = 0;   // Measure complete

static void ulsonic_init(void) {
	// TRIG: PE6 → output
	DDRE |= ULSONIC_TRIG;
	// ECHO: PE7 → input
	DDRE &= ~ULSONIC_ECHO;

	int7_set_rising();         // INT7: rising edge
	EIMSK |= (1 << INT7);      // enable INT7
	EIFR  |= (1 << INTF7);     // pending flag clear
}

// INT7: ECHO rising/falling edge
ISR(INT7_vect) {
	if (echo_state == DETECT_RISING_EDGE) {	// Echo-HIGH detected
		start_time = TCNT1;
		int7_set_falling();	// Set INT7 Active-falling edge
		echo_state = DETECT_FALLING_EDGE;
		} 
	else {	// Echo-LOW detected
		uint16_t end_time = TCNT1;
		ulsonic_ticks = end_time - start_time; // ECHO pulse duration
		ulsonic_done  = 1;
		int7_set_rising();	// Re-set INT7 as Active-rising edge for next measurement
		echo_state = DETECT_RISING_EDGE;
	}
}

static uint16_t ulsonic_measure_ticks(void) {
	ulsonic_done = 0;
	EIFR |= (1 << INTF7);   // Pending flag clear 

	ulsonic_trig();

	uint16_t ticks = 0;
	if (ulsonic_done) {
		ticks = ulsonic_ticks;
	} 
	else {
		ticks = 0;  // timeout
	}
	ulsonic_done = 0;
	return ticks;
}

static uint16_t ticks_to_mm(uint16_t ticks) {
	uint32_t tmp = ticks;
	tmp *= 68;      
	tmp /= 100;
	if (tmp > 65535UL) tmp = 65535UL;	// rounding at max of 2^16
	return (uint16_t)tmp;	// <- ticks / 4u * 340 * 1000 / 2 = ticks * 0.68
}	// 

// ======================================
// BUTTON (INT4)
// ======================================
static void button_init(void) {
	DDRE  &= ~(1 << PE4);    // Set PE4 input
	PORTE |=  (1 << PE4);    // Pull up
	EIMSK |=  (1 << INT4);   // INT4 enable
	EICRB |=  (1 << ISC41);  // INT4: falling edge
	EIFR  |=  (1 << INTF4);  // pending flag clear
}

ISR(INT4_vect) {
	_delay_ms(5);                 // Debounce
	if (PINE & (1 << PE4)) return;
	events |= EV_BUTTON_PRESSED;
}

// ======================================
// CLCD (4-bit mode on PORTA)
// ======================================
typedef enum {
	CLCD_RS  = (1 << 0),
	CLCD_RW  = (1 << 1),
	CLCD_E   = (1 << 2),
	CLCD_DB4 = (1 << 4),
	CLCD_DB5 = (1 << 5),
	CLCD_DB6 = (1 << 6),
	CLCD_DB7 = (1 << 7),
} Clcd_t;

#define CLCD_CTRL_MASK (CLCD_RS | CLCD_RW | CLCD_E)
#define CLCD_DATA_MASK (CLCD_DB4 | CLCD_DB5 | CLCD_DB6 | CLCD_DB7)

static inline void clcd_pulse_enable(void) {
	PORTA |= CLCD_E;
	_delay_us(1);
	PORTA &= ~CLCD_E;
	_delay_us(1);
}

static void clcd_write4(uint8_t val) {
	PORTA &= ~CLCD_DATA_MASK;

	if (val & (1 << 0)) PORTA |= CLCD_DB4;
	if (val & (1 << 1)) PORTA |= CLCD_DB5;
	if (val & (1 << 2)) PORTA |= CLCD_DB6;
	if (val & (1 << 3)) PORTA |= CLCD_DB7;

	clcd_pulse_enable();
}

typedef enum {
	BIT4_LINE2_DOT58       = 0x2C,
	DISPON_CURON_BLKON     = 0x0F,
	DISPOFF_CUROFF_BLKOFF  = 0x08,
	INC_NOSHIFT            = 0x06,
	DISP_CLEAR             = 0x01
} Clcd_Cmd_t;

static void clcd_write_cmd(uint8_t cmd) {
	PORTA &= ~CLCD_RS;
	PORTA &= ~CLCD_RW;

	clcd_write4(cmd >> 4);
	clcd_write4(cmd & 0x0F);

	if (cmd == DISP_CLEAR || cmd == 0x02) {
		_delay_ms(2);     // Long execution time
		} else {
		_delay_us(40);
	}
}

static void clcd_write_data(uint8_t data) {
	PORTA |=  CLCD_RS;
	PORTA &= ~CLCD_RW;

	clcd_write4(data >> 4);
	clcd_write4(data & 0x0F);

	_delay_us(40);
}

static void clcd_goto_xy(uint8_t row, uint8_t col) {
	uint8_t addr = 0x00;
	if (row == 1)      addr = col;
	else if (row == 2) addr = 0x40 + col;
	clcd_write_cmd(0x80 | addr);
}

static void clcd_write_str(uint8_t row, char *ptr) {
	if (row == 1) clcd_goto_xy(1, 0);
	else          clcd_goto_xy(2, 0);

	while (*ptr != 0x00) {
		clcd_write_data(*ptr++);
	}
}

static void clcd_init(void) {
	DDRA |= CLCD_CTRL_MASK | CLCD_DATA_MASK;
	_delay_ms(20);

	PORTA &= ~(CLCD_RS | CLCD_RW);

	clcd_write4(0x03);
	_delay_ms(5);
	clcd_write4(0x03);
	_delay_us(150);
	clcd_write4(0x03);
	_delay_us(150);
	clcd_write4(0x02);
	_delay_us(150);

	clcd_write_cmd(BIT4_LINE2_DOT58);
	clcd_write_cmd(DISPOFF_CUROFF_BLKOFF);
	clcd_write_cmd(DISP_CLEAR);
	clcd_write_cmd(INC_NOSHIFT);
	clcd_write_cmd(DISPON_CURON_BLKON);
}

// height(mm) → "xxx.xcm" string
static void convert_height_to_ascii(uint16_t height_mm, char ascii_result[9]) {
	memset(ascii_result, ' ', 9);
	ascii_result[8] = '\0';

	uint16_t h = height_mm;

	if (h > 999) {
		ascii_result[0] = '1';
		ascii_result[1] = ((h / 100) % 10) + '0';
		ascii_result[2] = ((h / 10)  % 10) + '0';
		ascii_result[3] = '.';
		ascii_result[4] = ( h        % 10) + '0';
		} else if (h > 99) {
		ascii_result[1] = ((h / 100) % 10) + '0';
		ascii_result[2] = ((h / 10)  % 10) + '0';
		ascii_result[3] = '.';
		ascii_result[4] = ( h        % 10) + '0';
		} else if (h > 9) {
		ascii_result[2] = ((h / 10)  % 10) + '0';
		ascii_result[3] = '.';
		ascii_result[4] = ( h        % 10) + '0';
		} else {
		ascii_result[2] = '0';
		ascii_result[3] = '.';
		ascii_result[4] = ( h        % 10) + '0';
	}

	ascii_result[6] = 'c';
	ascii_result[7] = 'm';
}

// ======================================
// Global Variable(about height)
// ======================================
static uint16_t height_top_mm   = 0;   // 바닥→천장 거리
static uint16_t height_human_mm = 0;   // 사람 키
static char     ascii_height[9];

// ======================================
// FSM
// ======================================
static void fsm_step(void) {
	// Compare prev state with curr state
	if (curr_state != next_state) {
		curr_state      = next_state;
		state_first_ent = true;
	}

	switch (curr_state) {
		case STATE_0:
		if (state_first_ent) {
			clcd_write_cmd(DISP_CLEAR);
			clcd_write_str(1, "If on bottom");
			clcd_write_str(2, "Press switch!");
			state_first_ent = false;
		}

		if (events & EV_BUTTON_PRESSED) {
			events &= ~EV_BUTTON_PRESSED;
			next_state = STATE_1;
		}
		break;

		case STATE_1:
		if (state_first_ent) {
			// Measure bottom to floor(top) height
			uint16_t ticks   = ulsonic_measure_ticks();
			uint16_t dist_mm = ticks_to_mm(ticks);
			height_top_mm    = dist_mm;   

			clcd_write_cmd(DISP_CLEAR);
			clcd_write_str(1, "If on head");
			clcd_write_str(2, "Press switch!");
			state_first_ent = false;
		}

		if (events & EV_BUTTON_PRESSED) {
			events &= ~EV_BUTTON_PRESSED;
			next_state = STATE_2;
		}
		break;

		case STATE_2: {
			bool do_measure = false;

			if (state_first_ent) {		// First measurement
				do_measure = true;      
				state_first_ent = false;
				} 
			else if (events & EV_BUTTON_PRESSED) { // After n times measurement by button
				events &= ~EV_BUTTON_PRESSED;
				do_measure = true;      
			}

			if (do_measure) {
				uint16_t ticks_head   = ulsonic_measure_ticks();
				uint16_t dist_head_mm = ticks_to_mm(ticks_head);

				clcd_write_cmd(DISP_CLEAR);
				clcd_write_str(1, "Your height");

				// If measured distance is zero or longer than top height, print "ERROR".
				if (dist_head_mm == 0 || dist_head_mm >= height_top_mm) {
					clcd_write_str(2, "ERROR");
					} 
				else {
					height_human_mm = height_top_mm - dist_head_mm;
					convert_height_to_ascii(height_human_mm, ascii_height);
					clcd_write_str(2, ascii_height);
				}
			}
			break;
		}

		default:
		next_state = STATE_0;
		break;
	}
}

// ======================================
// main
// ======================================
int main(void) {
	clcd_init();
	timer1_init();
	button_init();
	ulsonic_init();
	sei();

	while (true) {
		fsm_step();
	}
}
