/*
 * File:   newavr-main.c
 * Author: Jack
 *
 * Created on December 27, 2022, 4:07 PM
 */
//constant definitions
#define DEVICE_ID 0x09
#define LIGHTSHIELD 0x11
#define F_CPU 8000000UL
#define BAUD 115200
#define MAX 255
#define MIN 0
#define MID 127

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdarg.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define TRUE 1
#define FALSE 0
//pin definitions
//face buttons
#define START PB0
#define Y PB4
#define X PB3
#define B PB2
#define A PB1

//side buttons
#define LT PC0
#define RT PC1
#define Z PB5


//joystick buttons
#define UP PC2
#define DOWN PC3
#define RIGHT PC4
#define LEFT PC5

//angle modifiers
#define MODX PD0
#define MODY PD1

//lightshield button
#define LS PD2

//c-stick buttons
#define CL PD3
#define CR PD4
#define CU PD6
#define CD PD7



//face and side packets
volatile uint8_t face = 0;
volatile uint8_t side = 0b10000000;

//X and Y values for joystick, C-stick (128 is centered)
volatile uint8_t JX = MID;
volatile uint8_t JY = MID;
volatile uint8_t CX = MID;
volatile uint8_t CY = MID;

//trigger values
//LTrd and RTrd are digital triggers
//LTr is used for intermediate shield values; light shield, mid shield
volatile uint8_t LTr = 0;
volatile uint8_t LTrd = 0;
volatile uint8_t RTrd = 0;

//stick status variables
//H suffix variables tell whether or not the physical button is still held
//other variables tell whether or not the direction is actually being treated as active
volatile unsigned int upH;
volatile unsigned int downH;
volatile unsigned int leftH;
volatile unsigned int rightH;
volatile unsigned int up;
volatile unsigned int down;
volatile unsigned int left;
volatile unsigned int right;
volatile unsigned int lockUp;
volatile unsigned int lockDown;
volatile unsigned int lockLeft;
volatile unsigned int lockRight;

volatile unsigned int cUpH;
volatile unsigned int cDownH;
volatile unsigned int cLeftH;
volatile unsigned int cRightH;
volatile unsigned int cUp;
volatile unsigned int cDown;
volatile unsigned int cLeft;
volatile unsigned int cRight;
volatile unsigned int lockCUp;
volatile unsigned int lockCDown;
volatile unsigned int lockCRight;
volatile unsigned int lockCLeft;

volatile unsigned int modX;
volatile unsigned int modY;


//sends a character through UART

int uartSend(uint8_t c) {
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    return 0;
}
//reads a character through UART

int readChar(void) {
    while (!(UCSR0A & (1 << RXC0))) {
    }
    return UDR0;
}
//sets up uart communication
//256k baud, 8N1 format, enables recieve interrupt

void uart_init() {
    UBRR0L = 8;
    UCSR0C |= (1 << UCSZ00) | (1 << UCSZ01); //8 bit, no parity, 1 stop bit
    UCSR0B |= (1 << RXEN0) | (1 << RXCIE0) | (1 << TXEN0); //enable recieve and recieve interrupt
    UCSR0A |= (1 << U2X0);
}
//sets all pins as inputs (except for communication, debug, and oscillator pins)

void pinInit() {
    DDRB &= ~(1 << PB0);
    DDRB &= ~(1 << PB1);
    DDRB &= ~(1 << PB2);
    DDRB &= ~(1 << PB3);
    DDRB &= ~(1 << PB4);
    DDRB &= ~(1 << PB5);

    DDRC &= ~(1 << PC0);
    DDRC &= ~(1 << PC1);
    DDRC &= ~(1 << PC2);
    DDRC &= ~(1 << PC3);
    DDRC &= ~(1 << PC4);
    DDRC &= ~(1 << PC5);

    DDRD &= ~(1 << PD0);
    DDRD &= ~(1 << PD1);
    DDRD &= ~(1 << PD2);
    DDRD &= ~(1 << PD3);
    DDRD &= ~(1 << PD4);
    DDRD &= ~(1 << PD6);
    DDRD &= ~(1 << PD7);

}
//sends response to the origin probe

void sendOrigin() {
    uartSend(0x1F);
    uartSend(0xFF);
    uartSend(0x80);
    uartSend(0x80);
    uartSend(0x80);
    uartSend(0x80);
    uartSend(0x00);
    uartSend(0x00);
    uartSend(0x00);
    uartSend(0x00);
}
//sends the input status

void sendInputs() {
    uartSend(face);
    uartSend(side);
    uartSend(JX);
    uartSend(JY);
    uartSend(CX);
    uartSend(CY);
    uartSend(LTr);
    uartSend(0x00); //right trigger is never being used
}
//does calculations on angles of the joystick

void joyRead() {
    //any time a button is not held the lock and direction vals are set to false
    if (~upH) {
        up = FALSE;
        lockUp = FALSE;
    }
    if (~downH) {
        down = FALSE;
        lockDown = FALSE;
    }
    if (~rightH) {
        right = FALSE;
        lockRight = FALSE;
    }
    if (~leftH) {
        left = FALSE;
        lockLeft = FALSE;
    }

    
    if (upH & ~lockUp) {//if up is held and not yet locked
        if (~downH | lockDown) {//if down is not held or down is locked
            up = TRUE;
        } else if (downH & up) {//if down is held and we're already moving up
            up = FALSE;
            lockUp = TRUE;
        }
    }
    if (downH & ~lockDown) {//if down is held and not locked
        if (~upH | lockUp) {//if up is not held or is locked
            down = TRUE;
        } else if (upH & down) {//if up held and already moving down
            down = FALSE;
            lockDown = TRUE;
        }
    }

    if (leftH & ~lockLeft) {
        if (~right | lockRight) {
            left = TRUE;
        } else if (rightH & left) {
            left = FALSE;
            lockLeft = TRUE;
        }
    }
    if (rightH & ~lockRight) {
        if (~left | lockLeft) {
            right = TRUE;
        } else if (leftH & right) {
            right = FALSE;
            lockRight = TRUE;
        }
    }

    if (right) {
        if (down) {//right/down diagonal
            if (modX & ~modY) {
                JX = 222;
                JY = 88;
            } else if (modY & ~modX) {
                JY = 34;
                JX = 168;
            } else {
                JX = 217;
                JY = 39;

            }
        }
        if (up) {//up/right diagonal
            if (modX & ~modY) {
                JX = 222;
                JY = 168;
            } else if (modY & ~modX) {
                JX = 168;
                JY = 222;
            } else {
                JX = 217;
                JY = 217;
            }
        } else {//right
            JY = MID;
            if (modX & ~modY) {
                JX = 212;
            } else if (modY & ~modX) {
                JX = 171;
            } else {
                JX = MAX;
            }
        }
    } else if (left) {
        if (down) {//down/left diagonal
            if (modX & ~modY) {
                JX = 34;
                JY = 88;
            } else if (modY & ~modX) {
                JX = 88;
                JY = 34;
            } else {
                JX = 39;
                JY = 39;
            }
        }
        if (up) {//up/left diagonal
            if (modX & ~modY) {
                JY = 168;
                JX = 34;
            } else if (modY & ~modX) {
                JX = 88;
                JY = 222;
            } else {
                JX = 39;
                JY = 217;
            }
        } else {//left
            JY = MID;
            if (modX & ~modY) {
                JX = 44;
            } else if (modY & ~modX) {
                JX = 85;
            } else {
                JX = MIN;
            }
        }
    } else if (down) {//down
        JX = MID;
        if (modX & ~modY) {
            JY = 60;
        } else if (modY & ~modX) {
            JY = 34;
        } else {
            JY = MIN;
        }
    } else if (up) {//up
        JX = MID;
        if (modX & ~modY) {
            JY = 196;
        } else if (modY & ~modX) {
            JY = 222;
        } else {
            JY = MAX;
        }

    } else {//middle
        JX = MID;
        JY = MID;
    }

}
//does calculations on angles of the c-stick

void cRead() {
    side &= ~15; //clears d-pad
    if (~cUpH) {
        cUp = FALSE;
        lockCUp = FALSE;
    }
    if (~cDownH) {
        cDown = FALSE;
        lockCDown = FALSE;
    }
    if (~cRightH) {
        cRight = FALSE;
        lockCRight = FALSE;
    }
    if (~cLeftH) {
        cLeft = FALSE;
        lockCLeft = FALSE;
    }

    if (cUpH & ~lockCUp) {
        if (~cDown | lockCDown) {
            cUp = TRUE;
        } else if (cDownH & cUp) {
            cUp = FALSE;
            lockCUp = TRUE;
        }
    }
    if (cDownH & ~lockCDown) {
        if (~cUp | lockCUp) {
            cDown = TRUE;
        } else if (cUpH & cDown) {
            cDown = FALSE;
            lockCDown = TRUE;
        }
    }

    if (cLeftH & ~lockCLeft) {
        if (~cRight | lockCRight) {
            cLeft = TRUE;
        } else if (cRightH & cLeft) {
            cLeft = FALSE;
            lockCLeft = TRUE;
        }
    }
    if (cRightH & ~lockCRight) {
        if (~cLeft | lockCLeft) {
            cRight = TRUE;
        } else if (cLeftH & cRight) {
            cRight = FALSE;
            lockCRight = TRUE;
        }
    }
    if (modX & modY) {//d-pad inputs
        if(cRight)
        {
            side |= (1 << 2);
        }if(cLeft)
        {
            side |= (1);
        }if(cUp)
        {
            side |= (1 << 4);
        }if(cDown)
        {
            side |= (1 << 3);
        }
    } else {
        if (cRight) {
            if (cDown) {
                
            } else if (cUp) {

            } else {
                CX = MAX;
                CY = MID;
            }
        } else if (cLeft) {
            if (cDown) {

            } else if (cUp) {

            } else {
                CX = MIN;
                CY = MID;
            }
        } else if (cUp) {
            CY = MAX;
            CX = MID;
        } else if (cDown) {
            CY = MIN;
            CX = MID;

        } else {
            CX = MID;
            CY = MID;
        }
    }



}
//does calculations on the status of the triggers (basically just a lightshield check)

void trigRead() {
    if (PIND & (1 << LS)) {
        LTrd = FALSE;
        RTrd = FALSE;
        LTr = LIGHTSHIELD;
    } else {
        LTr = 0;

        if (PINB & (1 << LT)) {
            side |= (1 << 6);
        } else {
            side &= ~(1 << 6);
        }
        if (PINB & (1 << RT)) {
            side |= (1 << 5);
        } else {
            side &= ~(1 << 5);
        }
    }
}
//checks the status of all pins being used as inputs.  Face buttons and side buttons are stored in a ready to send format (lightshield check still needs to be done before sending)

void readInput() {
    if (PINB & (1 << START)) {
        face |= (1 << 4);
    } else {
        face &= ~(1 << 4);
    }
    if (PINB & (1 << Y)) {
        face |= (1 << 3);
    } else {
        face &= ~(1 << 3);
    }
    if (PINB & (1 << X)) {
        face |= (1 << 2);
    } else {
        face &= ~(1 << 2);
    }
    if (PINB & (1 << B)) {
        face |= (1 << 1);
    } else {
        face &= ~(1 << 1);
    }
    if (PINB & (1 << A)) {
        face |= (1 << 0);
    } else {
        face &= ~(1 << 0);
    }



    if (PINB & (1 << Z)) {
        side |= (1 << 4);
    } else {
        side &= ~(1 << 4);
    }

    if (PIND & (1 << CU)) {
        cUpH = TRUE;
    } else {
        cUpH = FALSE;
    }
    if (PIND & (1 << CD)) {
        cDownH = TRUE;
    } else {
        cDownH = FALSE;
    }
    if (PIND & (1 << CR)) {
        cRightH = TRUE;
    } else {
        cRightH = FALSE;
    }
    if (PIND & (1 << CL)) {
        cLeftH = TRUE;
    } else {
        cLeftH = FALSE;
    }

    if (PINB & (1 << UP)) {
        upH = TRUE;
    } else {
        upH = FALSE;
    }
    if (PINB & (1 << DOWN)) {
        downH = TRUE;
    } else {
        downH = FALSE;
    }
    if (PINB & (1 << RIGHT)) {
        rightH = TRUE;
    } else {
        rightH = FALSE;
    }
    if (PINB & (1 << LEFT)) {
        leftH = TRUE;
    } else {
        leftH = FALSE;
    }

    if (PIND & (1 << MODX)) {
        modX = TRUE;
    } else {
        modX = FALSE;
    }
    if (PIND & (1 << MODY)) {
        modY = TRUE;
    } else {
        modY = FALSE;
    }
}

//main loop.  Setup communications, enable interrupts, checks pin status on loop until console probes.

int main(void) {
    uart_init();
    sei();
    while (1) {
        readInput();
    }
}
//packet received interrupt

ISR(USART_RX_vect, ISR_BLOCK) {
    uint8_t probe = readChar();
    if ((probe == 0x00) | (probe == 0xFF))//id probe
    {
        uartSend(0x09);
    } else if ((probe == 0x41) | (probe == 0x42)) //origin probe
    {
        sendOrigin();
    } else if (probe == 0x40) //standard poll 
    {
        //just for flushing the other two chars out of the recieve
        readChar();
        readChar();

        //does stick angle calculations and trigger calculations
        //joyRead();
        //cRead();
        //trigRead();

        //send the input status packet to the console
        sendInputs();
    }else
    {
        uartSend(probe);
    }
}
