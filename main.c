/* 4-digit display with ATtiny261
 * Pinout:
 * PA0-PA1 - Messeingang 1+2, PA0 doppelbelegung leds4
 * PA0-PA7 - 7-segment-display pin a-h (ACTIVE LOW)
 * PB3-PB6 - 7-segment anode drivers 0-3 */

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <stdbool.h>

#define DOT_DMX_SEND 3
#define DOT_DMX_RECV 0
#define DOT(x) (dots & (1<<x))
#define SET_DOT(x) do {dots |= (1<<x);} while (0)
#define UNSET_DOT(x) do {dots &= ~(1<<x);} while (0)
#define TOGGLE_DOT(x) do {if (DOT(x)) UNSET_DOT(x); else SET_DOT(x);} while (0)

#include "types.h"
#include "usiTwiSlave.h"

config_t config = {
    .channel = 0,
    .dmx_sending = 0,
    .dmx_receiving = 0,
    .override_count = 1,
    .override_value = {255, 255, 255}
};

uint8_t* const rxbuffer = (uint8_t*)&config;
uint8_t* const txbuffer = (uint8_t*)&config;

const unsigned char plexdelay = 32; //~244Hz = 8MHz/1024/plexdelay
const unsigned short debouncedelay = 40; // ~12Hz -> 8MHz/16384/debouncedelay
const unsigned char fast_flash = 1; // toggle with 6Hz -> ~3Hz
const unsigned char menu_toggle = 12; // ~1Hz

volatile unsigned char currentCode[] = {0xff, 0xff, 0xff, 0xff};
volatile unsigned char dots = 0; // lower 4 bits for dots
const unsigned char segmentCode[] PROGMEM = {
//    edpcgbfa
    0b00101000, // 0
    0b11101011, // 1
    0b00110010, // 2
    0b10100010, // 3     a
    0b11100001, // 4   f   b
    0b10100100, // 5     g
    0b00100100, // 6   e   c
    0b11101010, // 7     d  p
    0b00100000, // 8
    0b10100000, // 9
    0b11111111, // 10: off
    0b01100000, // 11: A
    0b00111100, // 12: C
    0b00100011, // 13: D
    0b00110100, // 14: E
    0b01110100, // 15: F
    0b01100001, // 16: H
    0b10101011, // 17: J
    0b00111101, // 18: L
    0b01100111, // 19: N
    0b01110000, // 20: P
    0b01110111, // 21: R
    0b00110101, // 22: T
    0b00101001  // 23: U
};
#define LETTER_A 11
#define LETTER_C 12
#define LETTER_D 13
#define LETTER_E 14
#define LETTER_F 15
#define LETTER_H 16
#define LETTER_J 17
#define LETTER_L 18
#define LETTER_N 19
#define LETTER_P 20
#define LETTER_R 21
#define LETTER_T 22
#define LETTER_U 23

ISR(TIMER0_COMPA_vect) {
    static unsigned char activeSegment = 0;
    PORTA = 0b11111111; // disable all segments
    PORTB |= 0b00111010; // disable all drivers
    PORTA = currentCode[activeSegment];
    if (dots & (1 << (3-activeSegment)))
        PORTA &= ~(1 << 5);
    if (activeSegment == 3) {
        PORTB &= ~(1 << PB1);
        activeSegment = 0;
    } else {
        PORTB &= ~(1 << (5-activeSegment));
        activeSegment++;
    }
}

unsigned char menu_toggle_ctr = 0; // set by other functions
ISR(TIMER1_OVF_vect) { // ~12Hz
    static unsigned char fast_ctr = 0;
    fast_ctr++;
    if (fast_ctr > fast_flash) {
        if (config.dmx_receiving && !DOT(DOT_DMX_SEND))
            SET_DOT(DOT_DMX_SEND);
        else
            UNSET_DOT(DOT_DMX_SEND);
        if (config.dmx_sending && !DOT(DOT_DMX_RECV))
            SET_DOT(DOT_DMX_RECV);
        else
            UNSET_DOT(DOT_DMX_RECV);
        fast_ctr = 0;
    }
    if (menu_toggle_ctr > 0) {
        menu_toggle_ctr--;
    }
}

/* Button values:
    BTN  = 33k/(10k+33k) * 256 = 196 test: 176
    DOWN = 10k/(10k+10k) * 256 = 128 test: 102
    UP   = 3k3/(10k+3k3) * 256 = 64  test: 46
  Add margins:
    BTN/IDLE:  (196+256)/2 = 226 t: (176+255)/2=215
    DOWN/BTN:  (128+196)/2 = 162 t: (102+176)/2=144
    UP/DOWN:    (64+128)/2 = 96  t:  (46+102)/2=74
*/
enum btn_state_t {BTN_IDLE, BTN_PUSH, BTN_DOWN, BTN_UP} btn_state = BTN_IDLE;
enum btn_state_t last_btn_state = BTN_IDLE;
void handle_menu();
ISR(ADC_vect) {
    unsigned char adc = ADCH;
    if (adc < 74)
        btn_state = BTN_UP;
    else if (adc < 144)
        btn_state = BTN_DOWN;
    else if (adc < 215)
        btn_state = BTN_PUSH;
    else
        btn_state = BTN_IDLE;
    handle_menu();
}

static inline void set_digit(unsigned char idx, unsigned char digit) {
    currentCode[3-idx] = pgm_read_byte(&segmentCode[digit]);
}

// sets numbers from 0..999 to segments 1..3
const unsigned short decadeValues[] PROGMEM = {100, 10, 1};
void set_number(unsigned short number) {
    unsigned char digit;
    if (number > 999L) {
        set_digit(1, LETTER_E);
        set_digit(2, LETTER_E);
        set_digit(3, LETTER_E);
        return;
    }
    for (unsigned char decade = 0; decade < 3; decade++) {
        digit = 0;
        const unsigned short decadeValue = pgm_read_dword(&decadeValues[decade]);
        while (number >= decadeValue) {
            digit++;
            number -= decadeValue;
        }
        // if (digit == 0 && decade == 0)
        //     digit = 10;
        set_digit(decade+1, digit);
    }
}

unsigned char lastbtn = 0;
bool handle_number(unsigned short* number, unsigned short max) {
    unsigned char increment = 0;
    if ((btn_state == BTN_UP && last_btn_state != BTN_DOWN) ||
        (btn_state == BTN_DOWN && last_btn_state != BTN_UP)) {
        if (lastbtn > 100) {
            increment = 13;
        } else if (lastbtn > 60) {
            increment = 8;
        } else if (lastbtn > 30) {
            increment = 3;
        } else if (lastbtn > 10) {
            increment = 1;
        } else if (lastbtn == 0) {
            increment = 1;
        }
        if (lastbtn < 250)
            lastbtn++;

        if (increment > 0) {
            if (btn_state == BTN_UP) {
                *number += increment;
                if (*number > max)
                    *number -= max+1    ;
            } else {
                if (*number < increment)
                    *number += max+1;
                *number -= increment;
            }
            return true;
        }
    } else
        lastbtn = 0;
    return false;
}

enum {menu_channel, menu_override_count, menu_override_value1, menu_override_value2, menu_override_value3, menu_rst, menu_cycle_max} menu_state = menu_channel;
void handle_menu() {
    static unsigned char flash_menu_init = 0;
    unsigned short* override_value;

    if (btn_state == BTN_IDLE && last_btn_state == BTN_PUSH) {
        menu_state = menu_state + 1;
        if (menu_state == menu_cycle_max)
            menu_state = 0;
        flash_menu_init = 0;
        menu_toggle_ctr = 0;
        lastbtn = 0;
    }

    switch (menu_state) {
        case menu_channel:
            set_number(config.channel+1);
            set_digit(0, LETTER_C);
            handle_number(&config.channel, 511);
            break;
        case menu_override_count:
            if (btn_state == BTN_UP && last_btn_state == BTN_IDLE) {
                if (config.override_count < 3)
                    config.override_count++;
                else
                    config.override_count = 1;
            }
            if (btn_state == BTN_DOWN && last_btn_state == BTN_IDLE) {
                if (config.override_count > 1)
                    config.override_count--;
                else
                    config.override_count = 3;
            }

            set_digit(0, LETTER_N);
            set_digit(1, LETTER_C);
            set_digit(2, LETTER_H);
            set_digit(3, config.override_count);
            break;
        case menu_override_value1:
        case menu_override_value2:
        case menu_override_value3:
            override_value = &config.override_value[menu_state-menu_override_value1];
            switch (flash_menu_init) {
                case 0:
                    set_digit(0, LETTER_T);
                    set_digit(1, 0);
                    set_digit(2, LETTER_P);
                    set_digit(3, menu_state-menu_override_value1+1);
                    menu_toggle_ctr = 2*menu_toggle;
                    flash_menu_init = 1;
                    break;
                case 1:
                    if (menu_toggle_ctr == 0) {
                        flash_menu_init = 2;
                        menu_toggle_ctr = menu_toggle;
                        set_digit(0, LETTER_T);
                        set_number(*override_value);
                    }
                    break;
                case 2:
                    if (menu_toggle_ctr == 0) {
                        flash_menu_init = 1;
                        menu_toggle_ctr = menu_toggle;
                        set_digit(0, menu_state-menu_override_value1+1);
                        set_number(*override_value);
                    }
                    break;
            };
            if (handle_number(override_value, 255))
                set_number(*override_value);
            break;
        case menu_rst:
            if ((btn_state == BTN_UP || btn_state == BTN_DOWN) && last_btn_state == BTN_IDLE) {
                menu_toggle_ctr = 2*menu_toggle;
                config.channel = 1;
                config.override_count = 1;
                config.override_value[0] = 255;
                config.override_value[1] = 255;
                config.override_value[2] = 255;
            }
            if (menu_toggle_ctr == 0) {
                set_digit(0, LETTER_R);
                set_digit(1, LETTER_E);
                set_digit(2, 5);
                set_digit(3, LETTER_T);
            } else {
                set_digit(0, LETTER_D);
                set_digit(1, 0);
                set_digit(2, LETTER_N);
                set_digit(3, LETTER_E);
            }
            break;
        default:
            menu_state = menu_channel;
    }
    last_btn_state = btn_state;
}

int main() {
    DDRA = 0b11111111;
    DDRB = 0b00111010;
    PORTA = 0b11111111; // disable all segments
    PORTB = 0b00111010; // disable all anode drivers
    DIDR0 = 0b11111111; // digital input disable adc0..6+aref (PA0..7)
    DIDR1 = 0b11110000; // digital input disable adc7..10 (PB4..7)

    // ADC to read buttons
    ADCSRA = (1<<ADEN) | (1<<ADATE) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0); // enable adc + auto trigger + adc interrupt; prescaler 128
    ADCSRB = (1<<ADTS2) | (1<<ADTS1) | (0<<ADTS0); // trigger adc on timer1 overflow
    ADMUX = (1<<ADLAR) | 0b1001; // PB6

    // timer0 for segment multiplexing
    OCR0A = plexdelay; // set compare match value
    TCCR0A = 1; // CTC mode (TOP=OCR0A)
    TCCR0B = (1<<CS02) | (0<<CS01) | (1<<CS00); // prescaler 1024

    // timer1 to debounce buttons and handle the ui
    TC1H = debouncedelay >> 8;
    OCR1C = debouncedelay & 0xff;
    TCCR1B = 0b1111; // prescaler 16384

    // i2c/twi slave
    usiTwiSlaveInit(0x1A<<1);

    // initialization state
    set_digit(0, LETTER_H);
    set_digit(1, LETTER_E);
    set_digit(2, LETTER_L);
    set_digit(3, 0);

    TIMSK = (1<<TOIE1) | (1<<OCIE0A); // enable timer 0 compare match interrupt / timer1 overflow interrupt
    sei();

    while(1) {
        // sanity checks
        if (config.channel > 511)
            config.channel = 0;
        if (config.override_count > 3 || config.override_count == 0)
            config.override_count = 1;
    }

    return 0;
}
