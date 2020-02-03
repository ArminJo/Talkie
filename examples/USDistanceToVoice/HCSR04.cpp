/*
 *  HCSR04.cpp
 *
 *  US Sensor (HC-SR04) functions
 *  The non blocking functions are using pin change interrupts and need the PinChangeInterrupt library to be installed.
 *
 *  Copyright (C) 2018-2020  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>
#include "HCSR04.h"

#define DEBUG

uint8_t sTriggerOutPin;
uint8_t sEchoInPin;
bool sHCSR04PinsAreInitialized = false;

void initUSDistancePins(uint8_t aTriggerOutPin, uint8_t aEchoInPin) {
    sTriggerOutPin = aTriggerOutPin;
    sEchoInPin = aEchoInPin;
    pinMode(aTriggerOutPin, OUTPUT);
    pinMode(sEchoInPin, INPUT);
    sHCSR04PinsAreInitialized = true;
}

/*
 * Start of standard blocking implementation using pulseInLong() since PulseIn gives wrong (too small) results :-(
 */
unsigned int getUSDistance(unsigned int aTimeoutMicros) {
    if (!sHCSR04PinsAreInitialized) {
        return 0;
    }

// need minimum 10 usec Trigger Pulse
    digitalWrite(sTriggerOutPin, HIGH);
#ifdef DEBUG
    delay(2); // to see it on scope
#else
    delayMicroseconds(10);
#endif
// falling edge starts measurement
    digitalWrite(sTriggerOutPin, LOW);

    /*
     * Get echo length.
     * Speed of sound at 20 degree is 343,46 m/s => 58,23 us per centimeter and 17,17 cm/ms (forth and back)
     * Speed of sound at 10 degree is 337,54 m/s => 59,25 us per centimeter and 16,877 cm/ms (forth and back)
     * At 20 degree => 50cm gives 2914 us, 2m gives 11655 us
     */
    unsigned long tUSPulseMicros = pulseInLong(sEchoInPin, HIGH, aTimeoutMicros);
    if (tUSPulseMicros == 0) {
// timeout happened
        tUSPulseMicros = aTimeoutMicros;
    }
    return tUSPulseMicros;
}

unsigned int getCentimeterFromUSMicroSeconds(unsigned int aDistanceMicros) {
    // The reciprocal of formula in getUSDistanceAsCentiMeterWithCentimeterTimeout()
    return (aDistanceMicros * 100L) / 5825;
}

/*
 * @return  Distance in centimeter @20 degree (time in us/58.25)
 *          aTimeoutMicros/58.25 if timeout happens
 *          0 if pins are not initialized
 *          timeout of 5825 micros is equivalent to 1 meter
 *          Default timeout of 20000 micro seconds is 3.43 meter
 */
unsigned int getUSDistanceAsCentiMeter(unsigned int aTimeoutMicros) {
    unsigned int tDistanceMicros = getUSDistance(aTimeoutMicros);
    if (tDistanceMicros == 0) {
// timeout happened
        tDistanceMicros = aTimeoutMicros;
    }
    return (getCentimeterFromUSMicroSeconds(tDistanceMicros));
}

// 58,23 us per centimeter (forth and back)
unsigned int getUSDistanceAsCentiMeterWithCentimeterTimeout(unsigned int aTimeoutCentimeter) {
// The reciprocal of formula in getCentimeterFromUSMicroSeconds()
    unsigned int tTimeoutMicros = ((aTimeoutCentimeter * 233L) + 2) / 4; // = * 58.25 (rounded by using +1)
    return getUSDistanceAsCentiMeter(tTimeoutMicros);
}

void testUSSensor(uint16_t aSecondsToTest) {
    for (long i = 0; i < aSecondsToTest * 50; ++i) {
        digitalWrite(sTriggerOutPin, HIGH);
        delayMicroseconds(582); // pulse is as long as echo for 10 cm
        // falling edge starts measurement
        digitalWrite(sTriggerOutPin, LOW);
        delay(20); // wait time for 3,43 meter to let the US pulse vanish
    }
}

/*
 * The NON BLOCKING version only blocks for ca. 12 microseconds for code + generation of trigger pulse
 * Be sure to have the right interrupt vector below.
 * check with: while (!isUSDistanceMeasureFinished()) {<do something> };
 * Result is in sUSDistanceCentimeter;
 */

// Comment out the line according to the sEchoInPin if using the non blocking version
// or define it as symbol for the compiler e.g. -DUSE_PIN_CHANGE_INTERRUPT_D0_TO_D7
//#define USE_PIN_CHANGE_INTERRUPT_D0_TO_D7  // using PCINT2_vect - PORT D
//#define USE_PIN_CHANGE_INTERRUPT_D8_TO_D13 // using PCINT0_vect - PORT B - Pin 13 is feedback output
//#define USE_PIN_CHANGE_INTERRUPT_A0_TO_A5  // using PCINT1_vect - PORT C
#if (defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) | defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) | defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5))

unsigned int sUSDistanceCentimeter;
volatile unsigned long sUSPulseMicros;

volatile bool sUSValueIsValid = false;
volatile unsigned long sMicrosAtStartOfPulse;
unsigned int sTimeoutMicros;

/*
 * common code for all interrupt handler.
 */
void handlePCInterrupt(uint8_t aPortState) {
    if (aPortState > 0) {
        // start of pulse
        sMicrosAtStartOfPulse = micros();
    } else {
        // end of pulse
        sUSPulseMicros = micros() - sMicrosAtStartOfPulse;
        sUSValueIsValid = true;
    }
#ifdef DEBUG
// for debugging purposes, echo to PIN 13 (do not forget to set it to OUTPUT!)
// digitalWrite(13, aPortState);
#endif
}
#endif // USE_PIN_CHANGE_INTERRUPT_D0_TO_D7 ...

#if defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7)
/*
 * pin change interrupt for D0 to D7 here.
 */
ISR (PCINT2_vect) {
// read pin
    uint8_t tPortState = (*portInputRegister(digitalPinToPort(sEchoInPin))) & bit((digitalPinToPCMSKbit(sEchoInPin)));
    handlePCInterrupt(tPortState);
}
#endif

#if defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13)
/*
 * pin change interrupt for D8 to D13 here.
 * state of pin is echoed to output 13 for debugging purpose
 */
ISR (PCINT0_vect) {
// check pin
    uint8_t tPortState = (*portInputRegister(digitalPinToPort(sEchoInPin))) & bit((digitalPinToPCMSKbit(sEchoInPin)));
    handlePCInterrupt(tPortState);
}
#endif

#if defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5)
/*
 * pin change interrupt for A0 to A5 here.
 * state of pin is echoed to output 13 for debugging purpose
 */
ISR (PCINT1_vect) {
// check pin
    uint8_t tPortState = (*portInputRegister(digitalPinToPort(sEchoInPin))) & bit((digitalPinToPCMSKbit(sEchoInPin)));
    handlePCInterrupt(tPortState);
}
#endif

#if (defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) | defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) | defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5))

void startUSDistanceAsCentiMeterWithCentimeterTimeoutNonBlocking(unsigned int aTimeoutCentimeter) {
// need minimum 10 usec Trigger Pulse
    digitalWrite(sTriggerOutPin, HIGH);
    sUSValueIsValid = false;
    sTimeoutMicros = ((aTimeoutCentimeter * 233) + 2) / 4; // = * 58.25 (rounded by using +1)
    *digitalPinToPCMSK(sEchoInPin) |= bit(digitalPinToPCMSKbit(sEchoInPin));// enable pin for pin change interrupt
// the 2 registers exists only once!
    PCICR |= bit(digitalPinToPCICRbit(sEchoInPin));// enable interrupt for the group
    PCIFR |= bit(digitalPinToPCICRbit(sEchoInPin));// clear any outstanding interrupt
    sUSPulseMicros = 0;
    sMicrosAtStartOfPulse = 0;

#ifdef DEBUG
    delay(2); // to see it on scope
#else
    delayMicroseconds(10);
#endif
// falling edge starts measurement and generates first interrupt
    digitalWrite(sTriggerOutPin, LOW);
}

/*
 * Used to check by polling.
 * If ISR interrupts these code, everything is fine, even if we get a timeout and a no null result
 * since we are interested in the result and not in very exact interpreting of the timeout.
 */
bool isUSDistanceMeasureFinished() {
    if (sUSValueIsValid) {
        sUSDistanceCentimeter = getCentimeterFromUSMicroSeconds(sUSPulseMicros);
        return true;
    }

    if (sMicrosAtStartOfPulse != 0) {
        if ((micros() - sMicrosAtStartOfPulse) >= sTimeoutMicros) {
            // Timeout happened, value will be 0
            *digitalPinToPCMSK(sEchoInPin) &= ~(bit(digitalPinToPCMSKbit(sEchoInPin)));// disable pin for pin change interrupt
            return true;
        }
    }
    return false;
}
#endif // USE_PIN_CHANGE_INTERRUPT_D0_TO_D7 ...
