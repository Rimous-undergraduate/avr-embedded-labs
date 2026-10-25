#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#define F_CPU	16000000UL
#include <util/delay.h>
#include <stdbool.h>
#include <string.h>
#include <util/atomic.h>   // For atomic 16-bit ADC read

// ======================================
// CLCD pin mask
// ======================================
typedef enum {
	CLCD_RS		= (1<<0),
	CLCD_RW		= (1<<1),
	CLCD_E		= (1<<2),
	CLCD_DB4	= (1<<4),
	CLCD_DB5	= (1<<5),
	CLCD_DB6	= (1<<6),
	CLCD_DB7	= (1<<7),
} Clcd_t;

#define	CLCD_CTRL_MASK	(CLCD_RS | CLCD_RW | CLCD_E)
#define CLCD_DATA_MASK	(CLCD_DB4 | CLCD_DB5 | CLCD_DB6 | CLCD_DB7)

// Generate enable pulse
static inline void clcd_pulse_enable(void){
	PORTA |= CLCD_E;
	_delay_us(1);
	PORTA &= ~CLCD_E;
	_delay_us(1);
}

// Write 4-bit data to CLCD
static void clcd_write4(uint8_t val){
	PORTA &= ~CLCD_DATA_MASK;
	
	if(val & (1<<0)) PORTA |= CLCD_DB4;
	if(val & (1<<1)) PORTA |= CLCD_DB5;
	if(val & (1<<2)) PORTA |= CLCD_DB6;
	if(val & (1<<3)) PORTA |= CLCD_DB7;
	
	clcd_pulse_enable();
}

// Write command byte
static void clcd_write_cmd(uint8_t cmd) {
	PORTA &= ~CLCD_RS;
	PORTA &= ~CLCD_RW;
	
	clcd_write4(cmd >> 4);
	clcd_write4(cmd & 0x0f);
	
	_delay_us(40);
}

// Write data byte
static void clcd_write_data(uint8_t data) {
	PORTA |= CLCD_RS;
	PORTA &= ~CLCD_RW;
	
	clcd_write4(data >> 4);
	clcd_write4(data & 0x0f);
	
	_delay_us(40);
}

// Set cursor position
static void clcd_goto_xy(uint8_t row, uint8_t col) {
	uint8_t addr = 0x00;
	
	if(row == 1) addr = col;
	else if(row == 2) addr = 0x40 + col;
	
	clcd_write_cmd(0x80 | addr);
}

// Print string on selected row
// Note: CLCD column addresses start from 0, but rows are numbered from 1 here
// for convenience (similar to typical matrix notation).
static void clcd_write_str(uint8_t row, char *ptr) {
	if(row == 1) clcd_goto_xy(1, 0);
	else         clcd_goto_xy(2, 0);
	
	while(*ptr != 0x00) {
		clcd_write_data(*ptr++);
	}
}

// LCD command set
typedef enum {
	BIT4_LINE2_DOT58		= 0x2c,
	DISPON_CUROFF_BLKOFF	= 0x0c,
	DISPON_CURON_BLKOFF		= 0x0e,
	DISPON_CUROFF_BLKON		= 0x0d,
	DISPON_CURON_BLKON		= 0x0f,
	// When DISPOFF is set, cursor/blink bits have no visible effect.
	DISPOFF_CUROFF_BLKOFF	= 0x08,
	DISPOFF_CURON_BLKOFF	= 0x0a,
	DISPOFF_CUROFF_BLKON	= 0x09,
	DISPOFF_CURON_BLKON		= 0x0b,
	INC_NOSHIFT				= 0x06,
	DISP_CLEAR				= 0x01
} Clcd_Cmd_t;

// Initialize CLCD
// Extra timing margin is added because some cloned modules do not strictly
// follow the original datasheet timing.
void clcd_init(void) {
	DDRA |= CLCD_CTRL_MASK | CLCD_DATA_MASK;
	_delay_ms(20);	// Power-up delay
	
	PORTA &= ~(CLCD_RS | CLCD_RW);

	// 4-bit init sequence
	clcd_write4(0x03);
	_delay_ms(5);
	clcd_write4(0x03);
	_delay_us(150);
	clcd_write4(0x03);
	_delay_us(150);
	clcd_write4(0x02);
	_delay_us(150);
	
	// Set LCD options
	clcd_write_cmd(BIT4_LINE2_DOT58);
	clcd_write_cmd(DISPOFF_CUROFF_BLKOFF);
	clcd_write_cmd(DISP_CLEAR);
	_delay_ms(2);
	clcd_write_cmd(INC_NOSHIFT);
}

// ======================================
// ADC section
// ======================================
void adc_init(void) {
	ADMUX  = (1<<REFS0);	// AVCC ref, ADC0, right adjust
	ADCSRA = (1<<ADEN) | (1<<ADFR) | (1<<ADIE) |
	(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

volatile static uint16_t adc_result = 0;
volatile static bool adc_updated = false;

// ADC interrupt: store the result and set update flag
ISR(ADC_vect) {
	uint8_t low  = ADCL;
	uint8_t high = ADCH;
	adc_result = (uint16_t)low | ((uint16_t)high << 8);
	adc_updated = true;
}

// Convert ADC value to a 4-character string
static void adc_to_ascii(uint16_t value, char ascii_result[5]) {
	// Fill with spaces. CLCD shows them as blank cells.
	memset(ascii_result, ' ', 4);
	ascii_result[4] = '\0';

	if(value > 999) {
		ascii_result[0] = 1 + '0';
		ascii_result[1] = ((value / 100) % 10) + '0';
		ascii_result[2] = ((value / 10)  % 10) + '0';
		ascii_result[3] = ( value        % 10) + '0';
	}
	else if(value > 99) {
		ascii_result[1] = ((value / 100) % 10) + '0';
		ascii_result[2] = ((value / 10)  % 10) + '0';
		ascii_result[3] = ( value        % 10) + '0';
	}
	else if(value > 9) {
		ascii_result[2] = ((value / 10)  % 10) + '0';
		ascii_result[3] = ( value        % 10) + '0';
	}
	else {
		ascii_result[3] = ( value        % 10) + '0';
	}
}

// Main loop: update CLCD when ADC data is ready
int main(void) {
	clcd_init();
	adc_init();
	sei();
	
	clcd_write_cmd(DISPON_CURON_BLKON);
	clcd_write_str(1, "Illuminance");
	
	ADCSRA |= (1<<ADSC);	// Start ADC (free-running)

	char ascii_result[5];
	uint16_t adc_snapshot = 0;

	while(1) {
		if(adc_updated) {
			ATOMIC_BLOCK(ATOMIC_FORCEON) {
				adc_snapshot = adc_result;
				adc_updated  = false;
			}
			
			adc_to_ascii(adc_snapshot, ascii_result);
			clcd_write_str(2, ascii_result);

			// Without this delay, the display updates too fast and appears to flicker.
			_delay_ms(100);
		}
	}
}
