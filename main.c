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

#define BTN (!(PINB & (1 << PB1)))

#define NUMBER_CHANGED (dots & (1 << 0)) // leftmost dot for changed number
#define SET_NUMBER_CHANGED do {dots |= (1 << 0);} while (0)
#define UNSET_NUMBER_CHANGED do {dots &= ~(1 << 0);} while (0)

const unsigned char plexdelay = 32; //~244Hz = 8MHz/1024/plexdelay
const unsigned char debouncedelay = 20; //~12Hz
const unsigned char savedelay = 60; // ~5s 240; //~20s

unsigned short currentNumber = 1;
unsigned char currentCode[] = {0xff, 0xff, 0xff, 0xff};
unsigned char dots = 0; // lower 4 bits for dots 0b1230
const unsigned char segmentCode[] PROGMEM = {
//    afbgcpde
    0b00010100, // 0
    0b11010111, // 1
    0b01001100, // 2
    0b01000101, // 3
    0b10000111, // 4
    0b00100101, // 5
    0b00100100, // 6
    0b01010111, // 7
    0b00000100, // 8
    0b00000101, // 9
    0b11111111, // 10: off
    0b00000110, // 11: A
    0b00111100, // 12: C
    0b00101100, // 13: E
    0b00101110, // 14: F
    0b10000110, // 15: H
    0b11010101, // 16: J
    0b10111100, // 17: L
    0b00001110, // 18: P
    0b10010100  // 19: U
};

ISR(TIMER0_COMPA_vect) {
    static unsigned char activeSegment = 0;
    PORTA = 0b11111111; // disable all segments
    PORTB |= 0b01111000; // disable all drivers
    if (activeSegment == 4)
        activeSegment = 0;
    PORTA = currentCode[activeSegment];
    if (dots & (1 << activeSegment))
        PORTA &= ~(1 << 2);
    PORTB &= ~(1 << (activeSegment+3));
    activeSegment++;
}

// void doadc() {
//     TCCR0B = 0;
//     ADCSRA |= 1<<ADSC; //start conversion, afterwards do button stuff to save time
//     unsigned char decade, number;
//         /*decade = PORTA; //use decade to save PORTA
//     DDRA &= ~(1<<PA2); //make PA2 input
//     PORTA |= 1<<PA2; //enable pullup on PA2 (where button resides)

//     if( (PINA >> 3 & 1) ^ lastbtnstate ) { //if button and saved state differ
//         lastbtnstate = !lastbtnstate; //save current state
//         if( !lastbtnstate ) { //if button was pressed right now, toggle adc
//             ADMUX ^= 1;
//             currentChannel = ADMUX & 1;
//         }
//     }
//     PORTA = decade; //restore portb
//     DDRA = 0b11111100; //make PB0 output again*/

//     while( ADCSRA & (1<<ADSC) ); //wait for conversion to end
//     unsigned short adc = ADC;

//         if( adc > 1016 ) { //Error threshold
//           currentCode[0] = segmentCode[11] << 2;
//           currentCode[1] = segmentCode[11]>> 6 & 1;
//           currentCode[2] = segmentCode[11] << 2;
//           currentCode[3] = segmentCode[11]>> 6 & 1;
//           currentCode[4] = segmentCode[11] << 2;
//           currentCode[5] = segmentCode[11]>> 6 & 1;
//         } else
//           for(decade = 0; decade < 3; decade++) {
//                   number = 0;
//                   while( adc >= compareValues[currentChannel][decade] ) {
//                           number++;
//                           adc -= compareValues[currentChannel][decade];
//                   }
//                   if( decade == 0 && number == 0 )
//                           number=10; //display nothing instead of zero if U<10V
//                   currentCode[2*decade] = segmentCode[number] << 2;
//                   currentCode[2*decade+1] = segmentCode[number] >> 6 & 1; //get bit 6, move it to PB0 (segment g)
//           }

//     TCCR0B = 5;
// }

unsigned char save_ctr = 0;
void save_number() {
    if (NUMBER_CHANGED) {
        if (save_ctr > savedelay) {
            eeprom_update_word(0, currentNumber);
            UNSET_NUMBER_CHANGED;
            save_ctr = 0;
        } else
            save_ctr++;
    }
}

void set_digit(unsigned char idx, unsigned char digit) {
    const unsigned short code = pgm_read_byte(&segmentCode[digit]);
    switch (idx) {
        case 0: currentCode[0] = code; break;
        case 1: currentCode[3] = code; break;
        case 2: currentCode[2] = code; break;
        case 3: currentCode[1] = code; break;
    }
}

// sets numbers from 0..511 to segments 1..3
const unsigned short decadeValues[] PROGMEM = {100, 10, 1};
void set_number(unsigned short number) {
    unsigned char digit;
    if (number > 999L) {
        set_digit(1, 13);
        set_digit(2, 13);
        set_digit(3, 13);
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

void poll_button() {
    static unsigned char lastbtn = 0;
    unsigned char increment = 0;
    if (BTN) {
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
        if (increment > 0) {
            currentNumber += increment;
            if (currentNumber > 512)
                currentNumber -= 512;
            set_number(currentNumber);
            SET_NUMBER_CHANGED;
            save_ctr = 0;
        }
        if (lastbtn < 250)
            lastbtn++;
    } else
        lastbtn = 0;
}

int main() {
    DDRA = 0b11111111;
    DDRB = 0b01111000;
    PORTA = 0b11111111; // disable all segments
    PORTB = 0b01111010; // disable all anode drivers, pullup on PB1
    // ADCSRA = 1<<ADEN;
    // ADMUX = 0;

    OCR0A = plexdelay; // set compare match value
    TIMSK = (1 << OCIE0A); // enable compare match interrupt
    TCCR0A = (1 << CTC0); // CTC mode (TOP=OCR0A)
    TCCR0B = (1 << CS02) | (0 << CS01) | (1 << CS00); // prescaler 1024

    set_digit(0, 12);
    currentNumber = eeprom_read_word(0);
    set_number(currentNumber);

    sei();

    unsigned char debounce = 0;
    while(1) {
        sleep_mode();
        //if( (currentCode[2*activeSegment+1] & (6-PB3)) && !(DDRA & 1) && wait > adcdelay ) {
        // if( !(DDRA & 4) ) {
        //     if( (PINA >> 2 & 1) ^ lastbtnstate ) { //if button and saved state differ
        //     lastbtnstate = !lastbtnstate; //save current state
        //     if( lastbtnstate ) { //if button was pressed right now, toggle adc
        //         ADMUX ^= 1;
        //         currentChannel = ADMUX & 1;
        //     }
        //     }
        // }
        if (debounce > debouncedelay) {
            // poll button each according to debouncedelay
            poll_button();

            // save to eeprom after some time (savedelay) if number is changed
            save_number();

            // #define NMAX 19
            // if (currentNumber == NMAX)
            //     currentNumber = 0;
            // else
            //     currentNumber++;
            // currentCode[0] = segmentCode[currentNumber];
            // currentCode[3] = currentNumber == NMAX ? segmentCode[0] : segmentCode[currentNumber+1];
            // currentCode[2] = currentNumber >= NMAX-1 ? segmentCode[currentNumber+1-NMAX] : segmentCode[currentNumber+2];
            // currentCode[1] = currentNumber >= NMAX-2 ? segmentCode[currentNumber+2-NMAX] : segmentCode[currentNumber+3];
            debounce = 0;
        } else
            debounce++;
    }

    return 0;
}
