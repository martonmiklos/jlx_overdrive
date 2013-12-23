/* 
 * File:   main.c
 * Author: mm
 *
 * Created on 2013. november 23., 14:43
 *
 *  PPM decoder for the JLX overdrive RC car
 *  Pinout:
 *      RA0: CAL BTN
 *      RA1: Left motor B halfbridge
 *      RA2: Left motor A halfbridge
 *      RA3: PPM IN
 *      RA4: Right motor B halfbridge
 *      RA5: Right motor A halfbridge
 *
 */


#include <xc.h>
#include <pic12lf1822.h>
#include <stdint.h>

#ifndef __PICCPRO__
#define __PICCPRO__
#endif

#define _XTAL_FREQ 16000000
#include "config.h"

/* Running from 16 MHz internal clock, Timer0 driven by FOSC/4 (4MHz)
 * Timer0 prescaler is 64 so it increments on every 16 uS
 * Above the MAX_PPM_CHANNEL_DURATION (2.7 ms) the PPM channel values are set to invalid
 */
#define MAX_PPM_CHANNEL_DURATION 33
#define CHANNEL_CENTER_TOP 25
#define CHANNEL_CENTER 23
#define CHANNEL_CENTER_BOT 20
#define MIN_PPM_CHANNEL_DURATION 12

#define SYNC_PULSE_MIN 180
#define CHANNEL_COUNT 3

#define PPM_INDEX_LAST CHANNEL_COUNT+1

#define FAILSAFE_TMR0_OVF_COUNT 2


typedef enum CalState {
    CalChannelAFull = 0,
    CalChannelANorminal = 1,
    CalChannelAMin = 2,
    CalChannelBFull = 3,
    CalChannelBNorminal = 4,
    CalChannelBMin = 5,
    CalDone = 6,
};

enum CalState calibrationState = CalChannelAFull;
uint8_t calValues[6];

static uint8_t PPMTimerValues[CHANNEL_COUNT+3]; // to be a able to keep the channel data + 2 sync pulses
static uint8_t PWMValues[4] = {0,0,0,0};
static uint8_t TMR0OVFCounter = 0, TMR0OVF = 0;

#define PWM_PERIOD_MAX 62
#define PWM_MIN 3
#define PWM_MAX 8

void shiftPPMValues()
{
    for (uint8_t i = 0; i<PPM_INDEX_LAST+1; i++) {
        PPMTimerValues[i] = PPMTimerValues[i+1];
    }
}

void checkBuffer()
{
    for (uint8_t i = 1; i<PPM_INDEX_LAST; i++) {
        if (PPMTimerValues[i] < MIN_PPM_CHANNEL_DURATION || PPMTimerValues[i] > MAX_PPM_CHANNEL_DURATION) {
            return;
        }
    }

    if(PPMTimerValues[0] == UINT8_MAX && PPMTimerValues[PPM_INDEX_LAST] == UINT8_MAX) {
        uint16_t temp = 0;
        // first stop everything
        PWMValues[0] = 0;
        PWMValues[1] = 0;
        PWMValues[2] = 0;
        PWMValues[3] = 0;
        /* Calculation method:
         * Full foward:        2.0 ms   -> 32 TMR0 increments
         * Norminal top:       1.125 ms -> 25 TMR0 increments
         * Norminal value:     1.5 ms   -> 23 TMR0 increments
         * Norminal bottom:    0.875 ms -> 21 TMR0 increments
         * Full backward:      1.0 ms   -> 14 TMR0 increments
         * We have ~31 TMR0 increments on the full scale
         * Our PWM resolution is set to 62 so we will use a simple logical shift to get the PWM value
         */

        if (PPMTimerValues[1] > CHANNEL_CENTER_TOP) {
            temp = ((PPMTimerValues[1] - CHANNEL_CENTER_TOP));
            if (temp > PWM_MAX) temp = PWM_MAX;
            if (temp < PWM_MIN) temp = PWM_MIN;
            PWMValues[0] = temp;
        } else if (PPMTimerValues[1] < CHANNEL_CENTER_BOT) {
            temp = ((CHANNEL_CENTER_BOT - PPMTimerValues[1]));
            if (temp > PWM_MAX) temp = PWM_MAX;
            if (temp < PWM_MIN) temp = PWM_MIN;
            PWMValues[1] = temp;
        }

        if (PPMTimerValues[2] > CHANNEL_CENTER_TOP) {
            temp = ((PPMTimerValues[2] - CHANNEL_CENTER_TOP));
            if (temp > PWM_MAX) temp = PWM_MAX;
            if (temp < PWM_MIN) temp = PWM_MIN;
            PWMValues[2] = temp;
        } else if (PPMTimerValues[2] < CHANNEL_CENTER_BOT) {
            temp = ((CHANNEL_CENTER_BOT - PPMTimerValues[2]));
            if (temp > PWM_MAX) temp = PWM_MAX;
            if (temp < PWM_MIN) temp = PWM_MIN;
            PWMValues[3] = temp;
        }
    }
}

void interrupt isr(void)
{
    if (IOCAFbits.IOCAF3) {
        IOCAFbits.IOCAF3 = 0;
        if (PORTAbits.RA3) { // check if rising edge
            shiftPPMValues();
            PPMTimerValues[PPM_INDEX_LAST+1] = TMR0;
            if (TMR0OVF || TMR0 > SYNC_PULSE_MIN)
                PPMTimerValues[PPM_INDEX_LAST+1] = UINT8_MAX;
            TMR0 = 0;
            TMR0OVF = 0;
            if (MIN_PPM_CHANNEL_DURATION < PPMTimerValues[PPM_INDEX_LAST+1]  && PPMTimerValues[PPM_INDEX_LAST+1] < MAX_PPM_CHANNEL_DURATION)
                checkBuffer();
        }
    }

    // Interrupt fired in every 16.384 ms
    if (INTCONbits.T0IF) {
        INTCONbits.T0IF = 0;
        TMR0OVFCounter++;
        TMR0OVF = 1;
        if (TMR0OVFCounter > FAILSAFE_TMR0_OVF_COUNT) {
            PWMValues[0] = 0;
            PWMValues[1] = 0;
            PWMValues[2] = 0;
            PWMValues[3] = 0;
            LATA &= 0b00001001;
            TMR0OVFCounter = FAILSAFE_TMR0_OVF_COUNT;
        }
    }
}

/*
 * 
 */
void main() {
    static uint_fast8_t PWMPeriodTimer = 0;

    LATA  &= 0b11001000;
    TRISA  = 0b00001000; // setup pins directions
    //WPUAbits.WPUA0 = 1;  // enable weak pullup on RA0
    OSCCON = 0b01111011; // internal oscillator 16 MHz
    IOCAPbits.IOCAP3 = 1; // enable rising edge interrupt on the PPM input
    INTCONbits.IOCIE = 1; // enable Port change interrupts

    // setup Timer0 to run at 15625 Hz (64 us step time)
    OPTION_REG = 0b00000111; // setup timer0 with no prescaler
    INTCONbits.TMR0IE = 1;  // enable TMR0 OVF interrupt -> OVF freq will be 16.384 ms

    ei(); // enable interrupts
    while (1) {
        /*if (++PWMPeriodTimer == 8) {
            LATAbits.LATA1 = PWMValues[0];
            LATAbits.LATA2 = PWMValues[1];
            LATAbits.LATA4 = PWMValues[2];
            LATAbits.LATA5 = PWMValues[3];
            PWMPeriodTimer = 0;
        }

        if (PWMPeriodTimer == PWMValues[0])
            LATAbits.LATA1 = 0;
        if (PWMPeriodTimer == PWMValues[1])
            LATAbits.LATA2 = 0;
        if (PWMPeriodTimer == PWMValues[2])
            LATAbits.LATA4 = 0;
        if (PWMPeriodTimer == PWMValues[3])
            LATAbits.LATA5 = 0;
        _delay(1);*/
        if (PWMValues[0] && PWMValues[1] == 0) {
            LATAbits.LATA2 = 0;
            LATAbits.LATA1 = 1;
        } else if (PWMValues[1] && PWMValues[0] == 0) {
            LATAbits.LATA1 = 0;
            LATAbits.LATA2 = 1;
        } else {
            LATAbits.LATA1 = 0;
            LATAbits.LATA2 = 0;
        }

        if (PWMValues[2] && PWMValues[3] == 0) {
            LATAbits.LATA5 = 0;
            LATAbits.LATA4 = 1;
        } else if (PWMValues[3] && PWMValues[2] == 0) {
            LATAbits.LATA4 = 0;
            LATAbits.LATA5 = 1;
        } else {
            LATAbits.LATA4 = 0;
            LATAbits.LATA5 = 0;
        }
    }
}

