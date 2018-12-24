/*
 * Talkie.h
 * Based on the Talkie library. https://github.com/going-digital/Talkie.
 * Copyright 2011 Peter Knight
 *
 *  SUMMARY
 *  Talkie is a speech library for Arduino.
 *
 *  Copyright (C) 2018  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Talkie_new https://github.com/ArminJo/Talkie_new.
 *
 *  Talkie_new is free software: you can redistribute it and/or modify
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

#ifndef _Talkie_h_
#define _Talkie_h_

#include <inttypes.h>

#if defined(__AVR__)
#define TIMING_PIN 12
#if !defined(__AVR_ATmega32U4__) && !defined(TCCR2A)
#error "Sorry, when using an AVR chip, Talkie requires Timer2.  This board doesn't have it."
#endif
#if F_CPU < 8000000L
#error "F_CPU must be at least 8 MHz"
#endif
#endif // (__AVR__)

#define FIFO_BUFFER_SIZE     24 // 24 sets of 4 bytes plus added queue indexes is about 100 added bytes.

#define TALKIE_USE_PIN 0xFF // use pin as output, but pin number is not yet specified

class Talkie {
public:
    Talkie();
    void say(const uint8_t * aWordDataAddress); // Blocking version
    int8_t sayQ(const uint8_t * aWordDataAddress); // Queuing version. Returns free space in FIFO
    void wait(); // wait for sayQ to end
    void stop(); // allow the actual word to end -> only clears the FIFO guarded with cli and sei
    void terminate(); // terminate synthesizer directly
    void resetFIFO();

    void initializeHardware();
    void terminateHardware();
    void doNotUseUseInvertedOutput(bool aDoNotUseInvertedOutput = true);
    void doNotUseNonInvertedOutput(bool aDoNotUseNonInvertedOutput = true);
    uint8_t NonInvertedOutputPin; // =0 Disable output. =0xFF Enable output. On Arduino enables pin 3 (Talkie default) as PWM output
    uint8_t InvertedOutputPin;    // =0 Disable output. =0xFF Enable output. On Arduino enables pin 11 as inverted PWM output to increase the volume
    const uint8_t * volatile WordDataPointer; // Pointer to word data array !!! Must be volatile, since it is accessed also by ISR
    uint8_t WordDataBit; // [0-7] bit number of next bit in array bitstream
    volatile bool isTalkingFlag = false;
    uint8_t getNumberOfWords(); // Returns 0 if nothing to play, otherwise the number of the queued items plus the one which is active.
    bool isTalking();
    uint8_t getBits(uint8_t bits);
    void setPtr(const uint8_t * aAddress);
    void FIFOPushBack(const uint8_t *aAddress);	// only sayQ() calls this
    const uint8_t * FIFOPopFront();	// only sayISR() calls this

    volatile uint8_t free; // init on setup = FIFO_BUFFER_SIZE

private:
    // FIFO queue for sayQ
    const uint8_t * FIFOBuffer[FIFO_BUFFER_SIZE];
    // not needed to specify the next 2 variables as volatile, since it code using them is synchronized by cli() and sei().
    uint8_t back; // index of last voice init on setup = 0
    uint8_t front; // index of next voice init on setup = 0

    // Bitstream parser
    uint8_t rev(uint8_t a);
};

#endif
