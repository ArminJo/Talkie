/*
 * Talkie.h
 * Based on the Talkie library. https://github.com/going-digital/Talkie.
 * Copyright 2011 Peter Knight
 *
 *  SUMMARY
 *  Talkie is a speech library for Arduino.
 *  Output is at pin 3 + 11
 *
 *  Copyright (C) 2018  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Talkie https://github.com/ArminJo/Talkie.
 *
 *  Talkie is free software: you can redistribute it and/or modify
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

#ifndef TALKIE_H_
#define TALKIE_H_

#include <inttypes.h>

#define VERSION_TALKIE "1.3.1"
#define VERSION_TALKIE_MAJOR 1
#define VERSION_TALKIE_MINOR 3
// The change log is at the bottom of the file

// If you do not use the Arduino Tone library, then activating can save up to 844 bytes program size :-)
//#define NO_COMPATIBILITY_FOR_TONE_LIB_REQUIRED

/*
 * Use 8bit coefficients K1 and K2.
 * Saves 10 microseconds (40 instead of 50 us) for a 16 MHz ATmega
 * has almost the same quality, except of a few "dropouts" e.g. in the word "thousand"
 */
//#define FAST_8BIT_MODE

#if defined(__AVR__)
#if !defined(__AVR_ATmega32U4__) && !defined(TCCR2A)
#error Sorry, when using an AVR chip, Talkie requires Timer2. This board does not have one.
#endif
#if F_CPU < 8000000L
#error F_CPU must be at least 8 MHz
#endif
#endif // (__AVR__)

#define FIFO_BUFFER_SIZE     24 // 24 sets of 4 bytes plus added queue indexes is about 100 added bytes.

#define TALKIE_USE_PIN_FLAG 0xFF // Flag to signal, that a pin (for inverted or not inverted output) should be used as output, but pin number is not yet filled in, since it depends of board type.
#define TALKIE_DO_NOT_USE_PIN_FLAG 0x00 // As pin number is initially != 0xFF, this is not really required at startup

class Talkie {
public:
    Talkie();
    Talkie(bool aUseNonInvertedOutputPin, bool aUseInvertedOutputPin);
    void beginPWM(uint8_t aPinPWM); // // To be compatible to Teensy library
    void say(const uint8_t *aWordDataAddress); // Blocking version
    int8_t sayQ(const uint8_t *aWordDataAddress); // Queuing version. Returns free space in FIFO
    void wait(); // wait for sayQ to end
    void stop(); // allow the actual word to end -> only clears the FIFO guarded with cli and sei
    void terminate(); // terminate synthesizer directly
    void resetFIFO();

    void initializeHardware();
    void terminateHardware();
    void doNotUseInvertedOutput(bool aDoNotUseInvertedOutput = true);
    void doNotUseNonInvertedOutput(bool aDoNotUseNonInvertedOutput = true);
    void digitalWriteInvertedOutput(uint8_t aValue);
    void digitalWriteNonInvertedOutput(uint8_t aValue);

    // Output pins to use. 0xFF -> Enable output (default). 0 -> Disable output.
    uint8_t NonInvertedOutputPin; // Pin number of output, maybe fixed for some boards. On Arduino enables pin 3 (Talkie default) as PWM output. 0xFF -> Enable output (default). 0 -> Disable output.
    uint8_t InvertedOutputPin;    // On Arduino enables pin 11 as inverted PWM output to increase the volume.

    const uint8_t * volatile WordDataPointer; // Pointer to word data array !!! Must be volatile, since it is accessed also by ISR
    uint8_t WordDataBit; // [0-7] bit number of next bit in array bitstream
    volatile bool isTalkingFlag;
    uint8_t getNumberOfWords(); // Returns 0 if nothing to play, otherwise the number of the queued items plus the one which is active.
    bool isTalking();
    uint8_t getBits(uint8_t bits);
    void setPtr(const uint8_t *aAddress);
    void FIFOPushBack(const uint8_t *aAddress); // only sayQ() calls this
    const uint8_t * FIFOPopFront(); // only sayISR() calls this

    volatile uint8_t free; // init on setup = FIFO_BUFFER_SIZE

private:
    // FIFO queue for sayQ
    const uint8_t * FIFOBuffer[FIFO_BUFFER_SIZE];
    // not required to specify the next 2 variables as volatile, since it code using them is guarded with noInterrupts() and interrupts()
    uint8_t back; // index of last voice init on setup = 0
    uint8_t front; // index of next voice init on setup = 0

    // Bitstream parser
    uint8_t rev(uint8_t a);
};

/*
 * Version 1.3.1
 * - Improved SAMD support.
 *
 * Version 1.3.0
 * - 10 bit Coefficients are working now, but they do not sound better :-(.
 * - Tested on an ESP32.
 * - Tested on a BluePill.
 *
 * Version 1.2.0 - 8/2020
 * - Corrected wrong function name doNotUseUseInvertedOutput().
 * - Added functions digitalWriteNonInvertedOutput() and digitalWriteInvertedOutput().
 *
 * Version 1.1.0 - 6/2020
 * - SAMD support.
 * - ESP32 support.
 * - Teensy support.
 * - Version number.
 * - Added function `sayQTimeout()` in *TalkieUtils.cpp*.
 * - Added example *USDistanceToVoice*.
 * - Added function `sayQVoltageVolts()`.
 * - Improved end handling to minimize clicks.
 *
 * Version 1.0.2 - 09-11/2019
 * - ATmega2560 supported and tested.
 * - Always set pins to input when finishing, to avoid a click.
 *
 * Version 1.0.1 - 09/2019
 * - Added SPI compatibility (after speaking do not reset pin 11 to input if SPI detected).
 *
 * Version 1.0.0 - 11/2018
 *  - Fix the ISR_RATIO Bug for plain Arduino
 *  - Added a lot of comments and do refactoring to better understand the functionality
 *  - Added stopping timer1 interrupts at every end of speech to free resources for usage of Arduino tone library
 *  - Extracted initializeHardware() function
 *  - Added some utility functions, extracted from the examples.
 *  - Improved shifting code so Talkie now runs on 8 MHz Arduino (with millis() interrupt disabled while talking)
 */

#endif // TALKIE_H_

#pragma once

