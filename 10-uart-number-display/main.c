#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdbool.h>

#define F_CPU 16000000UL

// ==================================================
// Configuration & Constants
// ==================================================

// Display modes for the 4-digit FND
typedef enum{
    DISP_NOTHING = 0,   // show "----"
    DISP_RB,            // mirror latest 4-digit input from UART (last[])
    DISP_MEMORY,        // browse stored history with LEFT/RIGHT, delete with DOWN
    DISP_LOVE           // Easter egg: show "LOVE"
} Disp_t;

// Timer / UART configuration
#define PRESCALE_TIMER0     64
#define BAUDRATE_9600       103

// Logical FSM tick frequency (Timer0 generates this rate)
#define FREQ_MainFSM        3000    // Hz
#define CNTVALUE_MainFSM    (uint8_t)((F_CPU / (PRESCALE_TIMER0)) / FREQ_MainFSM) // must be < 255

// Used for simple time-based decay (LEDs etc.)
#define CNTVALUE_1ms        (uint16_t)(0.001 * FREQ_MainFSM)

// --- RX ring buffer ---
#define RXBUF_SIZE          64
#define RXBUF_MASK          (RXBUF_SIZE - 1)

// --- TX ring buffer ---
#define TXBUF_SIZE          256
#define TXBUF_MASK          (TXBUF_SIZE - 1)

// --- History memory (circular, sparse, not strict FIFO) ---
#define MEM_SIZE            256
#define MEM_MASK            (MEM_SIZE - 1)
#define DUMMY               0xff   // slot never written
#define BLANK               0xfe   // logically deleted slot

// 7-seg font table: 0~9, L O V E, '-', square-outline, nothing
// index:
//   0~9   : digits 0..9
//   10~13 : L, O, V, E
//   14    : '-'
//   15~18 : square outline pattern
//   19    : all segments OFF
static uint8_t digit[] = {
    0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7c,0x07,0x7f,0x67, // 0-9
    0x38,0x3f,0x3e,0x79,                               // L O V E
    0x40,                                              // '-'
    0x39,0x09,0x09,0x0f,                               // square outline
    0x00                                               // nothing (all off)
};

// Special marker used in last[] to mean "no valid digit stored"
#define DIGIT_NOTHING ('0' + 19)

// ==================================================
// System Flags / Events / States
// ==================================================

// Flags: long-lived conditions (level semantics)
typedef enum {
    FLAG_PRINTING        = (1u << 0),  // reserved for TX state, not used yet
    FLAG_FSM_UPDATE      = (1u << 1),  // raised by Timer0 ISR, consumed in main
    FLAG_INVALID_INPUT   = (1u << 2)   // set when invalid input is detected
} SysFlags;
typedef uint8_t SysFlagMask;

// Events: one-shot occurrences (edge semantics)
typedef enum {
    EV_NONE                    = (1u << 0),
    EV_UP                      = (1u << 1),
    EV_DOWN                    = (1u << 2),
    EV_LEFT                    = (1u << 3),
    EV_RIGHT                   = (1u << 4),
    EV_RXC                     = (1u << 5),  // reserved
    EV_FAIL_TX                 = (1u << 6),  // TX buffer overflow
    EV_INVALID_INPUT_DETECTED  = (1u << 7)   // parser detected non-digit or overflow
} SysEvents;
typedef uint8_t SysEventMask;

// Top-level FSM state: mode of operation
typedef enum {
    ST_IDLE    = (1u << 0),   // idle mode, show nothing
    ST_UPDATE  = (1u << 1),   // input/commit mode
    ST_EDIT    = (1u << 2)    // history browsing mode
} State_t;

// ==================================================
// Globals
// ==================================================

// Shared between ISR and main
volatile SysFlagMask   flags  = 0;
volatile SysEventMask  events = 0;

// FSM current/previous/next state
static State_t curr_state = ST_IDLE;
static State_t prev_state = ST_IDLE;
static State_t next_state = ST_IDLE;

// RX ring buffer (producer: RX ISR, consumer: main)
static volatile uint8_t rxbuf[RXBUF_SIZE];
static volatile uint8_t rx_head = 0;
static volatile uint8_t rx_tail = 0;

// TX ring buffer (producer: main, consumer: UDRE ISR)
uint8_t          txbuf[TXBUF_SIZE];
static volatile uint8_t tx_head = 0;
static volatile uint8_t tx_tail = 0;

// History memory: compressed 4-digit values
// bits: [15:12][11:8][7:4][3:0] = thousands/hundreds/tens/ones
static uint16_t mem[MEM_SIZE];
static uint8_t  mem_head = 0;   // next write position
static uint8_t  mem_tail = 0;   // current browsing position

// ANSI escape parser state for arrow keys
typedef enum{
    ST_NORMAL,       // normal character input
    ST_ESC,          // ESC (0x1B) received
    ST_ESC_BRACKET   // ESC '[' received
} RxParserState_t;
static RxParserState_t rx_p_state = ST_NORMAL;

// Latest 4 characters from UART (digits or others)
// last[3] = most significant position, last[0] = least significant
static uint8_t last[4];

// Current content to display on the FND (index into digit[])
static uint8_t fnd[4] = {14,14,14,14}; // "----" initially

// Number of digit characters pushed into last[]
static uint16_t cnt_push_last = 0;

// ==================================================
// Prototypes
// ==================================================
static void     init(void);

// Ring buffer (RX)
static inline uint8_t rb_empty(void);
static inline uint8_t rb_pop(uint8_t *out);
static inline uint8_t rb_push_from_isr(uint8_t new_data);

// Transmit buffer (TX)
static inline uint8_t tb_pop_from_isr(void);
static uint8_t        tb_push(uint8_t data);
static uint8_t        uart_puts(const char *s);

// History
static void    mem_push_4digits(const uint8_t last4[4]);
static void    mem_del(void);

// Parser
void           rx_parser_process(uint8_t c);
static inline void push_last(uint8_t rx_value);
static void    clear_last(void);

// Display
void           decomp_data_from_mem(uint16_t comp_data, uint8_t *out);
static void    display_fnd(uint8_t fnd[4]);
void           display(Disp_t d);

// FSM
static void    on_entry(State_t s);
static void    on_during(State_t s);
static void    on_exit(State_t s);
static State_t decide_next(State_t s);
static void    fsm_step(void);

// ==================================================
// ISRs
// ==================================================

/*
 * Timer0 Compare Match ISR
 * - Only raises FLAG_FSM_UPDATE.
 * - Main loop checks this flag and executes one FSM tick when set.
 */
ISR(TIMER0_COMP_vect){
    flags |= FLAG_FSM_UPDATE;
}

/*
 * USART0 RX ISR
 * - Reads one received byte from UDR0.
 * - Echoes it back immediately.
 * - Pushes it into RX ring buffer.
 */
ISR(USART0_RX_vect){
    uint8_t data = UDR0;
    UDR0 = data;                 // echo-back
    rb_push_from_isr(data);
}

/*
 * USART0 UDRE ISR
 * - Called when transmit data register is empty.
 * - Pops one byte from TX buffer.
 * - If TX buffer is empty, disables UDRE interrupt.
 */
ISR(USART0_UDRE_vect){
    uint8_t data = tb_pop_from_isr();
    if (data == 0) {
        UCSR0B &= ~(1 << UDRIE0);
        return;
    }
    UDR0 = data;
}

// ==================================================
// Initialization
// ==================================================

/*
 * init()
 * - Configures I/O ports for FND and LEDs.
 * - Sets up Timer0 in CTC mode.
 * - Configures UART0 for 9600 baud, 8N1, RX interrupt.
 * - Initializes history memory.
 */
static void init(void) {
    // FND / LED ports:
    //  PORTA = segments (a..g, dp)
    //  PORTG[0..3] = digit select lines
    //  PORTF[0..3] = status LEDs
    DDRA = 0xff;
    DDRF = 0x0f;
    DDRG = 0x0f;

    // Timer0: CTC mode, prescaler 64
    TCCR0 = (1 << WGM01) | (1 << CS01) | (1 << CS00);
    OCR0  = (uint8_t)(CNTVALUE_MainFSM) - 1;
    TIMSK = (1 << OCIE0);

    // UART0: 9600 8N1, RX interrupt enabled
    UBRR0H = (BAUDRATE_9600 >> 8);
    UBRR0L = BAUDRATE_9600;
    UCSR0B = (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // System ON indicator (PF0)
    PORTF |= (1 << PF0);

    // Initialize history memory with DUMMY
    for (int i = 0; i < MEM_SIZE; i++) {
        mem[i] = DUMMY;
    }
    mem_tail = mem_head;
}

// ==================================================
// Ring Buffer (RX)
// ==================================================

static inline uint8_t rb_empty(void){
    return (rx_head == rx_tail);
}

// Pop one byte from RX buffer (non-blocking). Returns 1 if data was read.
static inline uint8_t rb_pop(uint8_t *out){
    if (rb_empty()) return 0;

    *out = rxbuf[rx_tail];
    rx_tail = (rx_tail + 1) & RXBUF_MASK;
    return 1;
}

// Push one byte into RX buffer from ISR context.
static inline uint8_t rb_push_from_isr(uint8_t new_data){
    uint8_t next = (rx_head + 1) & RXBUF_MASK;
    if (next == rx_tail) return 0; // buffer full → drop the byte

    rxbuf[rx_head] = new_data;
    rx_head        = next;
    return 1;
}

// ==================================================
// Transmit buffer (TX)
// ==================================================

/*
 * uart_puts()
 * - Enqueues a null-terminated string into TX buffer.
 * - Enables UDRE interrupt so ISR will send data out.
 * - Returns 1 on success, 0 if TX buffer overflows.
 */
static uint8_t uart_puts(const char *s) {
    // Ensure UDRE interrupt is enabled (starts TX engine)
    UCSR0B |= (1 << UDRIE0);

    while (*s) {
        if (tb_push((uint8_t)*s) == false) {
            // TX buffer full: raise event and report failure
            events |= EV_FAIL_TX;
            return 0;
        }
        s++;
    }
    return 1;
}

/*
 * tb_pop_from_isr()
 * - Used only from UDRE ISR.
 * - Returns next TX byte or 0 if buffer empty.
 */
static inline uint8_t tb_pop_from_isr(void){
    if (tx_tail == tx_head) {
        return 0;
    }

    uint8_t data = txbuf[tx_tail];
    tx_tail = (tx_tail + 1) & TXBUF_MASK;
    return data;
}

/*
 * tb_push()
 * - Enqueues one byte into TX buffer.
 * - Returns 1 on success, 0 if buffer full.
 */
static uint8_t tb_push(uint8_t data) {
    uint8_t next = (tx_head + 1) & TXBUF_MASK;

    if (next == tx_tail) return 0;  // full

    txbuf[tx_head] = data;
    tx_head        = next;
    return 1;
}

// ==================================================
// History (circular, sparse)
// ==================================================

/*
 * mem_push_4digits()
 * - Takes 4 ASCII digits in last4[]: last4[3]..last4[0].
 * - Packs them into a 16-bit value and stores into mem[].
 * - Advances mem_head and resets mem_tail to the most recent entry.
 */
static void mem_push_4digits(const uint8_t last4[4]){
    uint8_t  next = (mem_head + 1) & MEM_MASK;

    uint16_t d3 = (uint16_t)((last4[3] - '0') & 0x0F);
    uint16_t d2 = (uint16_t)((last4[2] - '0') & 0x0F);
    uint16_t d1 = (uint16_t)((last4[1] - '0') & 0x0F);
    uint16_t d0 = (uint16_t)((last4[0] - '0') & 0x0F);

    uint16_t compressed = (uint16_t)((d3 << 12) | (d2 << 8) | (d1 << 4) | d0);

    mem[mem_head] = compressed;
    mem_head      = next;
    mem_tail      = mem_head;
}

/*
 * mem_del()
 * - Logically deletes the current mem_tail entry by marking it BLANK.
 */
static void mem_del(void){
    mem[mem_tail] = BLANK;
}

// ==================================================
// ANSI Parser (ESC [ A/B/C/D → arrow events)
// ==================================================

/*
 * push_last()
 * - Shifts the 4-byte window and stores the newest byte in last[0].
 */
static inline void push_last(uint8_t rx_value){
    last[3] = last[2];
    last[2] = last[1];
    last[1] = last[0];
    last[0] = rx_value;
}

/*
 * clear_last()
 * - Resets last[] to "no digit" state and clears cnt_push_last.
 */
static void clear_last(void) {
    for (int i = 0; i < 4; i++){
        last[i] = DIGIT_NOTHING;
    }
    cnt_push_last = 0;
}

/*
 * rx_parser_process()
 * - Parses incoming bytes:
 *   * '0'~'9' in ST_NORMAL: pushed into last[], up to 4 chars.
 *   * ESC [ A/B/C/D: converts into EV_UP/DOWN/LEFT/RIGHT.
 *   * Any non-digit in ST_NORMAL: raises EV_INVALID_INPUT_DETECTED.
 *   * More than 4 digits: also raises EV_INVALID_INPUT_DETECTED.
 * - Also sends ANSI clear-screen sequence on arrow keys
 *   (for TeraTerm screen cleanup).
 */
void rx_parser_process(uint8_t c){
    switch (rx_p_state){
        case ST_NORMAL:
            if (c == 0x1B) {
                // ESC: switch to ESC state
                rx_p_state = ST_ESC;
            }
            else if ((c - '0' > 9) || (c - '0' < 0)) {
                // Non-digit in normal state
                events |= EV_INVALID_INPUT_DETECTED;
            }
            else {
                // Valid digit
                push_last(c);
                if (cnt_push_last++ > 3) {
                    // More than 4 digits accumulated
                    events |= EV_INVALID_INPUT_DETECTED;
                }
            }
            break;

        case ST_ESC:
            // Expect '[' after ESC for arrow sequences
            rx_p_state = (c == '[') ? ST_ESC_BRACKET : ST_NORMAL;
            break;

        case ST_ESC_BRACKET:
            // Arrow key sequence: ESC [ A/B/C/D
            // First, clear terminal screen and move cursor to (1,1)
            uart_puts("\x1B[2J");
            uart_puts("\x1B[H");

            switch (c){
                case 'A': events |= EV_UP;    rx_p_state = ST_NORMAL; break;
                case 'B': events |= EV_DOWN;  rx_p_state = ST_NORMAL; break;
                case 'C': events |= EV_RIGHT; rx_p_state = ST_NORMAL; break;
                case 'D': events |= EV_LEFT;  rx_p_state = ST_NORMAL; break;
                default:  rx_p_state = ST_NORMAL;                      break;
            }
            break;
    }
}

// ==================================================
// Display
// ==================================================

/*
 * decomp_data_from_mem()
 * - Decompresses a 16-bit stored value into 4 BCD digits.
 * - out[3] = MSD, out[0] = LSD.
 */
void decomp_data_from_mem(uint16_t comp, uint8_t *out){
    out[3] = (comp >> 12) & 0x0F;
    out[2] = (comp >>  8) & 0x0F;
    out[1] = (comp >>  4) & 0x0F;
    out[0] =  comp        & 0x0F;
}

/*
 * display_fnd()
 * - Performs multiplexed display of the 4 digits according to fnd[].
 * - One call updates exactly one digit; repeated calls cycle through digits.
 */
static void display_fnd(uint8_t fnd[4]){
    static uint8_t cnt = 0;
    uint8_t i = (uint8_t)(cnt++ & 0x03);

    // Turn all digits off
    PORTG = 0x00;
    // Set segments
    PORTA = digit[fnd[i]];
    // Enable exactly one digit
    PORTG = (1 << i);
}

/*
 * display()
 * - Updates fnd[] based on logical display mode.
 * - Does NOT directly touch the hardware ports;
 *   hardware update is done in main() via display_fnd().
 */
void display(Disp_t d) {

    switch (d){
        case DISP_NOTHING:
            // "----"
            for (int i = 0; i < 4; i++) fnd[i] = 14;
            break;

        case DISP_LOVE:
            // Display L O V E
            for (int i = 0; i < 4; i++) {
                fnd[i] = (uint8_t)(13 - i); // 10:L,11:O,12:V,13:E
            }
            break;

        case DISP_RB:
            // Directly map last[] ('0'..'9') to digit table indices
            for (int i = 0; i < 4; i++) {
                fnd[i] = (uint8_t)(last[i] - '0');
            }
            break;

        case DISP_MEMORY: {
            // Browse mem[] history by LEFT/RIGHT
            uint16_t data;
            uint8_t  found = 0;

            // Move to older entry (LEFT)
            if (events & EV_LEFT) {
                uint16_t guard = MEM_SIZE - 1;
                uint8_t idx = mem_tail;

                while (guard--) {
                    idx--; // wraps automatically for uint8_t
                    if (mem[idx] != DUMMY && mem[idx] != BLANK) {
                        data = mem[idx];
                        decomp_data_from_mem(data, &fnd[0]);
                        mem_tail = idx;
                        found = 1;
                        break;
                    }
                }
                if (!found) {
                    // No older entries: show "----"
                    for (int i = 0; i < 4; i++) fnd[i] = 14;
                }
            }
            // Move to newer entry (RIGHT)
            else if (events & EV_RIGHT) {
                uint16_t guard = MEM_SIZE - 1;
                uint8_t idx = mem_tail;

                while (guard--) {
                    idx++; // wraps automatically
                    if (mem[idx] != DUMMY && mem[idx] != BLANK) {
                        data = mem[idx];
                        decomp_data_from_mem(data, &fnd[0]);
                        mem_tail = idx;
                        found = 1;
                        break;
                    }
                }
                if (!found) {
                    // No newer entries: show a "square frame" pattern
                    for (int i = 0; i < 4; i++) fnd[i] = (uint8_t)(18 - i);
                }
            }
            // Delete current history entry (DOWN)
            else if (events & EV_DOWN) {
                mem_del();
                for (int i = 0; i < 4; i++) fnd[i] = 14; // show "----"
            }
        } break;
    }
}

// ==================================================
// FSM
// ==================================================

static void on_entry(State_t s){
    (void)s;
    // Placeholder for future per-state entry actions
}

/*
 * on_during()
 * - Executes the "during" behavior of the current state.
 * - Handles LEDs, display updates, and UART input processing.
 */
static void on_during(State_t s){
    // LED semantics:
    //   PF0: system ON
    //   PF1: RX activity indicator
    //   PF2: EDIT (DOWN) indicator
    //   PF3: UPDATE (UP) indicator
    static uint16_t cnt_led_up   = 0;
    static uint16_t cnt_led_down = 0;
    static uint16_t cnt_led_rx   = 0;

    switch (s){
        case ST_IDLE:
            PORTF |= (1 << PF0);
            display(DISP_NOTHING);
            break;

        case ST_UPDATE:
            // UP LED (PF3) decay behaviour
            if (events & EV_UP){
                PORTF |= (1 << PF3);
                cnt_led_up = 100 * CNTVALUE_1ms;
            } 
            else if (cnt_led_up == 0) {
                PORTF &= ~(1 << PF3);
            } 
            else {
                cnt_led_up--;
            }
            
            // When EV_UP is raised, we handle commit / error / easter egg
            if (events & EV_UP) {
                // Easter egg: if last[] == "LOVE"
                if ((last[3] == 'L') && (last[2] == 'O') &&
                    (last[1] == 'V') && (last[0] == 'E')) {
                    display(DISP_LOVE);
                    clear_last();
                }
                // Some invalid input was detected earlier
                else if (flags & FLAG_INVALID_INPUT) { 
                    flags &= ~FLAG_INVALID_INPUT;
                    display(DISP_NOTHING);
                    clear_last();
                }
                // No data in last[]
                else if (last[0] == DIGIT_NOTHING) {
                    display(DISP_NOTHING);
                    clear_last();
                }
                // Normal case: commit 4 digits and update display
                else {
                    mem_push_4digits(last);
                    display(DISP_RB);
                    clear_last();
                }
            }

            // EV_UP is fully consumed here
            events &= ~EV_UP;
            break;

        case ST_EDIT:
            // Clear invalid input flag when entering edit mode
            flags &= ~FLAG_INVALID_INPUT;
            
            // DOWN LED (PF2) decay behaviour
            if (events & EV_DOWN){        
                PORTF |= (1 << PF2);
                cnt_led_down = 100 * CNTVALUE_1ms;
            } 
            else if (cnt_led_down == 0) {
                PORTF &= ~(1 << PF2);
            } 
            else {
                cnt_led_down--;
            }

            // History browsing and delete
            display(DISP_MEMORY);

            // Consume directional events here (single-shot)
            events &= (uint8_t)~(EV_DOWN | EV_LEFT | EV_RIGHT);
            break;
    }

    // RX LED (PF1) and input parsing
    uint8_t rx_value;
    if (rb_pop(&rx_value)){
        // Process one received byte through parser
        rx_parser_process(rx_value);

        // If parser raised invalid-input event, reflect into FLAG_INVALID_INPUT
        if (events & EV_INVALID_INPUT_DETECTED) {
            cnt_push_last = 0;
            events &= ~EV_INVALID_INPUT_DETECTED;
            flags  |= FLAG_INVALID_INPUT;
        }

        // RX activity LED
        PORTF |= (1 << PF1);
        cnt_led_rx = 25 * CNTVALUE_1ms;
    } 
    else if (cnt_led_rx == 0) {
        PORTF &= ~(1 << PF1);
    } 
    else {
        cnt_led_rx--;
    }
}

/*
 * on_exit()
 * - Executes actions when leaving a state.
 * - Used here to keep LED outputs consistent.
 */
static void on_exit(State_t s){
    switch (s){
        case ST_IDLE:   break;
        case ST_UPDATE: PORTF &= ~(1 << PF3); break;
        case ST_EDIT:   PORTF &= ~(1 << PF2); break;
    }
}

/*
 * decide_next()
 * - Computes next state from current state and events.
 * - Uses only events (flags are treated inside states).
 */
static State_t decide_next(State_t s){
    switch (s){
        case ST_IDLE:
            if (events & EV_UP)                           return ST_UPDATE;
            else if (events & (EV_DOWN | EV_LEFT | EV_RIGHT)) return ST_EDIT;
            break;

        case ST_UPDATE:
            if (events & (EV_DOWN | EV_LEFT | EV_RIGHT))  return ST_EDIT;
            break;

        case ST_EDIT:
            if (events & EV_UP)                           return ST_UPDATE;
            break;
    }
    return s;
}

/*
 * fsm_step()
 * - Performs one full FSM step:
 *   exit(previous) → entry(current) → during(current) → next-state decision.
 */
static void fsm_step(void){
    if (prev_state != curr_state){
        on_exit(prev_state);
        on_entry(curr_state);
    }
    on_during(curr_state);
    next_state = decide_next(curr_state);
    prev_state = curr_state;
    curr_state = next_state;
}

// ==================================================
// Main
// ==================================================

int main(void)
{
    cli();
    init();
    sei();

    while (1){
        // FSM tick: executed whenever Timer0 raised FLAG_FSM_UPDATE
        if (flags & FLAG_FSM_UPDATE) {
            flags &= ~FLAG_FSM_UPDATE;
            fsm_step();
        }

        // Continuously refresh FND hardware according to current fnd[]
        display_fnd(fnd);
    }
}
