/*
 * Talkie.cpp
 * Based on the Talkie library. https://github.com/going-digital/Talkie.
 * Copyright 2011 Peter Knight
 * The code for the queued non blocking version is based on the fork of Paul Stoffregen. https://github.com/PaulStoffregen/Talkie
 *
 *  SUMMARY
 *  Talkie is a speech library for Arduino.
 *  Output is at pin 3 + 11 which are enabled by default
 *  It can also run on 8 MHz ATmega with either FAST_8BIT_MODE defined and slightly reduces speech quality or timer0 (for millis()) disabled  which is default (or both).
 *
 *  The speech PWM output signal is at pin 3 and/or 11.
 *  If both outputs are used and speaker is connected between pin 3 and pin 11 you get increased volume.
 *  On a plain Arduino Timer 1 (16 bit - for Servo library) is used to fill in new values for the PWM at the sample rate of 8000Hz / 125us.
 *                     Timer 2 (8 bit - for Tone library) is used to generate the 62500Hz / 16us PWM with 8 bit resolution on pin 3 + 11.
 *
 *  On ATmega32U4 Timer 4 for 200kHz / 5us PWM at pin 9 / PB5 & 10 /PB6 for Leonardo board
 *      at pin 5 / PC6 for Adafruit Circuit Playground Classic or Sparkfun Pro Micro board
 *      at pin 6 / PD7 for Esplora board
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
 * Version 1.0.1 09/2019
 * - Added SPI compatibility (after speaking do not reset pin 11 to input if SPI detected).
 * Version 1.0.0 11/2018
 *  - Fix the ISR_RATIO Bug for plain Arduino
 *  - Added a lot of comments and do refactoring to better understand the functionality
 *  - Added stopping timer1 interrupts at every end of speech to free resources for usage of Arduino tone library
 *  - Extracted initializeHardware() function
 *  - Added some utility functions, extracted from the examples.
 *  - Improved shifting code so Talkie now runs on 8MHz Arduino (with millis() interrupt disabled while talking)
 */

//#define __arm__
//#define CORE_TEENSY
#include <Arduino.h>

#include "Talkie.h"

// Enable this if you want to measure timing by toggling pin12 on an arduino
//#define MEASURE_TIMING
#ifdef MEASURE_TIMING
#include "digitalWriteFast.h"
#define TIMING_PIN 12
#endif

// If you do not use the Tone library, then activating can save up to 844 byte program size :-)
//#define NO_COMPATIBILITY_FOR_TONE_LIB_NEEDED

// If you do not use a SPI library, then activating can save 8 byte program size
//#define NO_COMPATIBILITY_FOR_SPI_NEEDED

/*
 * Use 8bit coefficients K1 and K2.
 * Saves 10 microseconds (40 instead of 50 us) for a 16 MHz ATmega
 * has almost the same quality, except of a few "dropouts" e.g. in the word "thousand"
 */
//#define FAST_8BIT_MODE
#include "TalkieLPC.h"

#define FS 8000 // Speech engine sample rate

static uint8_t synthPeriod;
static uint16_t synthEnergy;

#ifdef FAST_8BIT_MODE
static int8_t synthK1, synthK2;
#else
static int16_t synthK1, synthK2;
#endif
static int8_t synthK3, synthK4, synthK5, synthK6, synthK7, synthK8, synthK9, synthK10;

#define ISR_RATIO (25000/ (1000000 / FS) ) // gives 200 for FS 8000 -> 40Hz or 25ms Sample update frequency

#ifdef __cplusplus
extern "C" {
#endif
void timerInterrupt(void);
#ifdef __cplusplus
}
#endif

#if defined(__arm__)
#if defined(CORE_TEENSY)
IntervalTimer sIntervalTimer;
#else
static void tcConfigure(uint32_t sampleRate);
static void tcStartCounter();
static void tcEnd();
#endif
#endif

static void setNextSynthesizerData();

static Talkie *sPointerToTalkieForISR;
#if ISR_RATIO < 255
static uint8_t ISRCounterToNextData = 0;
#else
static uint16_t ISRCounterToNextData = 0;
#endif

Talkie::Talkie() { // @suppress("Class members should be properly initialized")
    /*
     * Enable non inverted and inverted output by default
     */
    NonInvertedOutputPin = TALKIE_USE_PIN_FLAG;
    InvertedOutputPin = TALKIE_USE_PIN_FLAG;
    isTalkingFlag = false;
    sPointerToTalkieForISR = this;
}

void Talkie::setPtr(const uint8_t * aAddress) {
    WordDataPointer = aAddress;
    WordDataBit = 0;
}

/*
 * Returns 0 if nothing to play, otherwise the number of the queued items plus the one which is active.
 */
uint8_t Talkie::getNumberOfWords() {
    yield();
    if (!isTalkingFlag) {
        return 0;   // Nothing playing!
    } else {
        return (1 + (FIFO_BUFFER_SIZE - free)); // 1 active plus X in queue
    }
}

bool Talkie::isTalking() {
    yield();
    return isTalkingFlag;
}

// The ROMs used with the TI speech were serial, not byte wide.
// Here's a handy routine to flip ROM data which is usually reversed.
uint8_t Talkie::rev(uint8_t a) {
// 76543210
    a = (a >> 4) | (a << 4); // Swap in groups of 4
// 32107654
    a = ((a & 0xcc) >> 2) | ((a & 0x33) << 2); // Swap in groups of 2
// 10325476
    a = ((a & 0xaa) >> 1) | ((a & 0x55) << 1); // Swap bit pairs
// 01234567
    return a;
}

/*
 * get next bits from array containing bit-stream data
 */
uint8_t Talkie::getBits(uint8_t bits) {
// 156 bytes compiled
    uint8_t value;
    uint16_t data;
    data = rev(pgm_read_byte(WordDataPointer)) << 8;
    if (WordDataBit + bits > 8) {
        data |= rev(pgm_read_byte(WordDataPointer + 1));
    }
    data <<= WordDataBit;
    value = data >> (16 - bits);
    WordDataBit += bits;
    if (WordDataBit >= 8) {
        WordDataBit -= 8;
        WordDataPointer++;
    }
    return value;
}

/*
 * Add to queue (back)
 */
void Talkie::FIFOPushBack(const uint8_t *aAddress) {
    free--;
    FIFOBuffer[back] = aAddress;
    if (++back >= FIFO_BUFFER_SIZE) {
        back = 0;
    }
}

/*
 * !! Only called by setNextSynthesizerData() in Interrupt context !!
 * Get next element from queue (front)
 * returns next element from queue or 0
 */
const uint8_t * Talkie::FIFOPopFront() {
// 56 byte compiled
    const uint8_t *addr = 0;    // returns 0 if empty.
    if (free < FIFO_BUFFER_SIZE) {
        free++;
        // get next addr
        addr = FIFOBuffer[front];
        if (++front >= FIFO_BUFFER_SIZE) {
            front = 0;
        }
    }
    return addr;
}

/*
 * The blocking version
 */
void Talkie::wait() {
    while (isTalkingFlag) {
        yield();
    }
}

/*
 * allow the actual word to end -> only clears the FIFO guarded with noInterrupts() and interrupts()
 */
void Talkie::stop() {
    noInterrupts();
    resetFIFO();
    interrupts();
}

/*
 * terminate synthesizer directly
 */
void Talkie::terminate() {
    resetFIFO();
    terminateHardware();
}

/*
 * The blocking version
 */
void Talkie::say(const uint8_t * aWordDataAddress) {
    sayQ(aWordDataAddress);
    while (isTalkingFlag) {
        yield();
    }
}

/*
 * The blocking version
 */
void Talkie::resetFIFO() {
    back = 0;
    front = 0;
    free = FIFO_BUFFER_SIZE;
}

/*
 * On Arduino enables pin 11 as inverted PWM output to increase the volume
 */
void Talkie::doNotUseUseInvertedOutput(bool aDoNotUseInvertedOutput) {
    if (aDoNotUseInvertedOutput) {
        InvertedOutputPin = 0;
    } else {
        InvertedOutputPin = TALKIE_USE_PIN_FLAG;
    }
}

/*
 * On Arduino disables pin 3 (Talkie default) as PWM output
 */
void Talkie::doNotUseNonInvertedOutput(bool aDoNotUseNonInvertedOutput) {
    if (aDoNotUseNonInvertedOutput) {
        NonInvertedOutputPin = 0;
    } else {
        NonInvertedOutputPin = TALKIE_USE_PIN_FLAG;
    }
}

void Talkie::initializeHardware() {
// Enable the speech system whenever say() is called.
#if defined(__AVR__)
#define _8_BIT_OUTPUT // 8 bit PWM
#if defined(__AVR_ATmega32U4__) // Use Timer 4 instead of Timer 2
#if defined(ARDUINO_AVR_CIRCUITPLAY) || defined(ARDUINO_AVR_PROMICRO)
// Adafruit Circuit Playground Classic or Sparkfun Pro Micro board. The first does not need inverted output, the latter does not connect it.
// Cannot be used on plain Leonardos since inverted output is connected to internal led.
    NonInvertedOutputPin = 5;// D5 / PC6 / !OC4A
    InvertedOutputPin = 0;// disable InvertedOutputPin
    pinMode(NonInvertedOutputPin, OUTPUT);
    TCCR4A = _BV(COM4A0) | _BV(PWM4A);// Clear on match, PWMA on, OC4A / PC7 & !OC4A / PC6 connected
#elif defined(ARDUINO_AVR_ESPLORA)
    NonInvertedOutputPin = 6; // Only D6 / PD7 / OC4D connected to Speaker
    InvertedOutputPin = 0;// disable InvertedOutputPin
    pinMode(NonInvertedOutputPin, OUTPUT);
    TCCR4C = _BV(COM4D1) |_BV(PWM4D);//
#else
// Leonardo, Lilypad USB, FLORA, TEENSY
    NonInvertedOutputPin = 10;// D10 / OC4B / PB6
    pinMode(NonInvertedOutputPin, OUTPUT);
    if (InvertedOutputPin) {
        // use both output
        InvertedOutputPin = 9;// D9 / !OC4B / PB5
        pinMode(InvertedOutputPin, OUTPUT);
        TCCR4A = _BV(COM4B0) | _BV(PWM4B);// Clear on match, PWMA on, OC4A & !OC4A connected
    } else {
        // use only non Inverted output
        TCCR4A = _BV(COM4B1) | _BV(PWM4B);// Clear on match, PWMA on, OC4A connected
    }
#endif // ATmega32U4 flavors

// common ATmega32U4
// Set up Timer4 for fast PWM 200kHz / 5us
    PLLFRQ = PLLFRQ | _BV(PLLTM1) | _BV(PLLTM0);// Route PLL to async clk, PLL Postcaler for High Speed Timer = 2
    TCCR4B = _BV(CS40);//  1:1 prescale
    TCCR4D = 0;// Fast PWM mode
    TCCR4E = 0;// Not enhanced mode
    TC4H = 0;// Not 10-bit mode
    DT4 = 0;// No dead time
    OCR4C = 255;// TOP
    OCR4A = 127;// 50% duty to start
    OCR4B = 127;// 50% duty to start
#else
    /*
     * Timer 2 set up as a 62500Hz 8Bit PWM for 16 MHz.
     * The PWM 'buzz' is well above human hearing range and is very easy to filter out.
     */
    TCCR2A = _BV(WGM21) | _BV(WGM20); // Fast PWM
    if (NonInvertedOutputPin) {
        NonInvertedOutputPin = 3; // OC2B
        pinMode(NonInvertedOutputPin, OUTPUT);
        TCCR2A |= _BV(COM2B1); // OC2B non-inverting mode
    }
    if (InvertedOutputPin) {
        InvertedOutputPin = 11; // OC2A
        pinMode(InvertedOutputPin, OUTPUT);
        TCCR2A |= _BV(COM2A1) | _BV(COM2A0); // OC2A inverting mode
    }
    TCCR2B = _BV(CS20);
    TIMSK2 = 0;
#endif // __AVR_ATmega32U4__

// common code for all AVR
#ifdef MEASURE_TIMING
    pinMode(TIMING_PIN, OUTPUT);
#endif
// Unfortunately we can't calculate the next sample every PWM cycle
// as the routine is too slow. So use Timer 1 to trigger that.
// Timer 1 set up as a 8000Hz / 125us sample interrupt
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10); // CTC mode, no prescale
    TCNT1 = 0;
#if F_CPU <= 8000000L
    TIMSK0 = 0; // // Tweak for 8 Mhz clock - must disable millis() interrupt
#endif
    OCR1A = ((F_CPU + (FS / 2)) / FS) - 1;  // 'FS' Hz (w/rounding)
    OCR1B = ((F_CPU + (FS / 2)) / FS) - 1;  // use the same value for register B
    TIMSK1 = _BV(OCIE1B); // enable compare register B match interrupt to use TIMER1_COMPB_vect and not interfere with the Servo library

#endif // (__AVR__)

#if defined(__arm__)

#define ISR(f) void f(void)

#if defined(CORE_TEENSY)
#if defined(__MKL26Z64__)
#define DAC_PIN A12
#elif defined(__MK20DX128__) || defined(__MK20DX256__)
#define DAC_PIN A14
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
#define DAC_PIN A21
#else
#error "Unknown Teensy"
#endif //(__MKL26Z64__) et.

#define _10_BIT_OUTPUT
    sIntervalTimer.begin(timerInterrupt, 1000000L / FS);
#else // (CORE_TEENSY)
#if defined(ARDUINO_ARCH_STM32)
#define DAC_PIN D7
    pinMode(DAC_PIN, PWM);
#else
#define DAC_PIN DAC0
#ifdef ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS
    static const int CPLAY_SPEAKER_SHUTDOWN= 11;
    pinMode(CPLAY_SPEAKER_SHUTDOWN, OUTPUT);
    digitalWrite(CPLAY_SPEAKER_SHUTDOWN, HIGH);
#endif
    analogWriteResolution(12);
    analogWrite(DAC_PIN, 0);
    tcConfigure(FS);
    tcStartCounter();
#endif
#endif // (CORE_TEENSY)
#endif

    isTalkingFlag = true;
}

void Talkie::terminateHardware() {
#if defined(__AVR__)
#if F_CPU <= 8000000
    TIMSK0 = _BV(TOIE0); // // Tweak for 8 Mhz clock - enable millis() interrupt again
#endif
    TIMSK1 = 0; // disable interrupts

#if defined(__AVR_ATmega32U4__)
    TCCR4A = 0; // disconnect outputs
#elif defined(ARDUINO_AVR_ESPLORA)
    TCCR4C = 0; // disconnect outputs
#else
    TCCR2A = 0; // disconnect outputs
#endif
#endif // (__AVR__)

#if defined(__arm__)
#if defined(CORE_TEENSY)
    sIntervalTimer.end();
#else
    tcEnd();
#endif
#endif

    /*
     * noTone disconnect the ports, so they are in normal port operation again.
     * pinMode(3|11, INPUT) avoids the click at the end, if speaker is coupled by a capacitance.
     */
    if (NonInvertedOutputPin) {

#ifndef NO_COMPATIBILITY_FOR_SPI_NEEDED
        // Reset pin 11 to input only if no active SPI detected - needs 8 byte Flash
        if (!(SPCR &  _BV(SPE))) {
#endif
            // force initializing of tone library, next time tone() is called.
#ifdef NO_COMPATIBILITY_FOR_TONE_LIB_NEEDED
            pinMode(NonInvertedOutputPin, INPUT); // tone needs this as output
#else
            noTone(NonInvertedOutputPin);
#endif
#ifndef NO_COMPATIBILITY_FOR_SPI_NEEDED
        }
#endif
    }
    if (InvertedOutputPin) {
#ifdef NO_COMPATIBILITY_FOR_TONE_LIB_NEEDED
        pinMode(InvertedOutputPin, INPUT); // tone needs this as output
#else
        noTone(InvertedOutputPin);
#endif
    }

    isTalkingFlag = false;
}

/*
 * The non blocking version as long as there is room in queue.
 * if queue is full then do a blocking wait for the queue to have room.
 * If talking, it just adds the aWordDataAddress to the queue by calling FIFOPushBack(aWordDataAddress)
 * If stopped it starts the speech output.
 * if aWordDataAddress is 0 it just clears the queue.
 */
int8_t Talkie::sayQ(const uint8_t * aWordDataAddress) {

    if (aWordDataAddress == 0) {
        // Caller asked to have queue made empty and sound stopped
        stop();
    } else {
        noInterrupts();
        // disable Interrupt since ptrAddr is also modified by ISR. This avoids race conditions.
        if (isTalkingFlag) {
            /*
             *  Word synthesizer still active -> queue this aWordDataAddress when there is room in FIFO
             */
            while (free == 0) {
                interrupts();
            }
            FIFOPushBack(aWordDataAddress);
            interrupts();
        } else {
            interrupts();
            /*
             * Word synthesizer inactive here -> START the word on this address
             */
            setPtr(aWordDataAddress);
            ISRCounterToNextData = ISR_RATIO;
            setNextSynthesizerData();   // Initialize first data for ISR
            resetFIFO();
            initializeHardware(); // sets isTalkingFlag to true;
        }
    }
    return (free);  // return free count after adding
}   // sayQ()

/*
 * Called every 125 microsecond / 8000 Hz
 */
#if defined(__AVR__)
ISR(TIMER1_COMPB_vect) {
    timerInterrupt();
}
#endif

#ifdef __cplusplus
extern "C" {
#endif
/*
 * 75 to 90 (when calling setNextSynthesizerData()) microseconds processing time @16MHz
 * 50 microseconds with 4 simple optimizations (change ">>15" to "<<1) >>16")
 */
void timerInterrupt() {
// 1286 byte compiled
#ifdef MEASURE_TIMING
    digitalWriteFast(TIMING_PIN, HIGH);
#endif

#if defined _8_BIT_OUTPUT
    static uint8_t nextPwm;
#else
    static uint16_t nextPwm;
#endif
    static uint8_t periodCounter;
    static int16_t x0, x1, x2, x3, x4, x5, x6, x7, x8, x9;

    int16_t u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, u10;

#if defined(__AVR__)
#if defined(__AVR_ATmega32U4__)
    OCR4A = nextPwm;
    OCR4B = nextPwm;
#elif defined(ARDUINO_AVR_ESPLORA)
    OCR4D = nextPwm;
#else
    OCR2B = nextPwm;
    OCR2A = nextPwm;
//    interrupts();
#endif // AVR flavors
#elif defined(__arm__)
    analogWrite(DAC_PIN, nextPwm);
#endif
    if (synthPeriod) {
        // Voiced source
        if (periodCounter < synthPeriod) {
            periodCounter++;
        } else {
            periodCounter = 0;
        }
        if (periodCounter < CHIRP_SIZE) {
            // inject chirp values to u10
            u10 = ((chirp[periodCounter]) * (uint32_t) synthEnergy) >> 8;
        } else {
            u10 = 0;
        }
    } else {
        // Unvoiced source. Use random bit generator / white noise
        static uint16_t synthRand = 1;
        synthRand = (synthRand >> 1) ^ ((synthRand & 1) ? 0xB800 : 0);
        u10 = (synthRand & 1) ? synthEnergy : -synthEnergy;
    }

// Lattice filter forward path -> fill temporary variables
// rescale by shifting >> 7 since we have signed values here (for unsigned it would need >> 8)
    u9 = u10 - (((int16_t) synthK10 * x9) >> 7);
    u8 = u9 - (((int16_t) synthK9 * x8) >> 7);
    u7 = u8 - (((int16_t) synthK8 * x7) >> 7);
    u6 = u7 - (((int16_t) synthK7 * x6) >> 7);
    u5 = u6 - (((int16_t) synthK6 * x5) >> 7);
    u4 = u5 - (((int16_t) synthK5 * x4) >> 7);
    u3 = u4 - (((int16_t) synthK4 * x3) >> 7);
    u2 = u3 - (((int16_t) synthK3 * x2) >> 7);
#ifdef FAST_8BIT_MODE
    u1 = u2 - (((int16_t) synthK2 * x1) >> 7);
    u0 = u1 - (((int16_t) synthK1 * x0) >> 7);
#else
// the 4 changes from ">> 15" to "<<1) >> 16" save 25 us processing time @16MHz :-)
    u1 = u2 - ((((int32_t) synthK2 * x1) << 1) >> 16);
    u0 = u1 - ((((int32_t) synthK1 * x0) << 1) >> 16);
#endif

// Output clamp
// Yes this happens generally seldom, but for spt_BUT it happens very often, which makes this sound unusable
    if (u0 > 511) {
        u0 = 511;
    }
    if (u0 < -512) {
        u0 = -512;
    }

// Lattice filter reverse path -> compute next x* values
    x9 = x8 + (((int16_t) synthK9 * u8) >> 7);
    x8 = x7 + (((int16_t) synthK8 * u7) >> 7);
    x7 = x6 + (((int16_t) synthK7 * u6) >> 7);
    x6 = x5 + (((int16_t) synthK6 * u5) >> 7);
    x5 = x4 + (((int16_t) synthK5 * u4) >> 7);
    x4 = x3 + (((int16_t) synthK4 * u3) >> 7);
    x3 = x2 + (((int16_t) synthK3 * u2) >> 7);
#ifdef FAST_8BIT_MODE
    x2 = x1 + (((int16_t) synthK2 * u1) >> 7);
    x1 = x0 + (((int16_t) synthK1 * u0) >> 7);
#else
    x2 = x1 + ((((int32_t) synthK2 * u1) << 1) >> 16);
    x1 = x0 + ((((int32_t) synthK1 * u0) << 1) >> 16);
#endif

    x0 = u0;

#if defined _8_BIT_OUTPUT
    nextPwm = (u0 >> 2) + 0x80;
#elif defined _10_BIT_OUTPUT
    nextPwm = (u0) + 0x200;
#elif defined _12_BIT_OUTPUT
    nextPwm = (u0 * 2) + 0x800;
#endif

    ISRCounterToNextData--;
    if (ISRCounterToNextData == 0) {
        if (sPointerToTalkieForISR->WordDataPointer == 0) {
            /*
             * Nothing to play -> go inactive and disable timer interrupt here and free pin resources
             */
            sPointerToTalkieForISR->terminateHardware();
        } else {
            ISRCounterToNextData = ISR_RATIO;
            setNextSynthesizerData();
        }
    }

#if defined(__arm__) && !defined(CORE_TEENSY)
    TC5->COUNT16.INTFLAG.bit.MC0 = 1;
#endif

#ifdef MEASURE_TIMING
    digitalWriteFast(TIMING_PIN, LOW);
#endif
}
#if defined(__arm__) && !defined(CORE_TEENSY)
void TC5_Handler(void) __attribute__ ((weak, alias("timerInterrupt")));
#endif
#ifdef __cplusplus
}
#endif

/*
 * is called every 25 ms / 40 Hz by timer1 ISR
 * or by sayQ to initialize new word
 */
static void setNextSynthesizerData() {
// 396 byte compiled
uint8_t energy = sPointerToTalkieForISR->getBits(4);
// Read speech data, processing the variable size frames.
if (energy == 0) {
// Energy = 0: rest frame
    synthEnergy = 0;
} else if (energy == 0xf) { // Energy = 15: stop frame. Silence the synthesizer and get new data.
    synthEnergy = 0;
    synthK1 = 0;
    synthK2 = 0;
    synthK3 = 0;
    synthK4 = 0;
    synthK5 = 0;
    synthK6 = 0;
    synthK7 = 0;
    synthK8 = 0;
    synthK9 = 0;
    synthK10 = 0;

// Get next word from FIFO
    sPointerToTalkieForISR->setPtr(sPointerToTalkieForISR->FIFOPopFront());

} else {
    uint8_t repeat;
    synthEnergy = pgm_read_byte(&tmsEnergy[energy]);
    repeat = sPointerToTalkieForISR->getBits(1);
    synthPeriod = pgm_read_byte(&tmsPeriod[sPointerToTalkieForISR->getBits(6)]); // 11 bits up to here
// A repeat frame uses the last coefficients
    if (!repeat) {
        // All frames use the first 4 coefficients
        synthK1 = pgm_read_word(&tmsK1[sPointerToTalkieForISR->getBits(5)]);
        synthK2 = pgm_read_word(&tmsK2[sPointerToTalkieForISR->getBits(5)]);
        synthK3 = pgm_read_byte(&tmsK3[sPointerToTalkieForISR->getBits(4)]);
        synthK4 = pgm_read_byte(&tmsK4[sPointerToTalkieForISR->getBits(4)]);        // 29 bits up to here
        if (synthPeriod) {
            // Voiced frames use 6 extra coefficients.
            synthK5 = pgm_read_byte(&tmsK5[sPointerToTalkieForISR->getBits(4)]);
            synthK6 = pgm_read_byte(&tmsK6[sPointerToTalkieForISR->getBits(4)]);
            synthK7 = pgm_read_byte(&tmsK7[sPointerToTalkieForISR->getBits(4)]);
            synthK8 = pgm_read_byte(&tmsK8[sPointerToTalkieForISR->getBits(3)]);
            synthK9 = pgm_read_byte(&tmsK9[sPointerToTalkieForISR->getBits(3)]);
            synthK10 = pgm_read_byte(&tmsK10[sPointerToTalkieForISR->getBits(3)]);        // 50 bits up to here
        }
    }
}
}

#if defined(__arm__) && !defined(CORE_TEENSY)
static bool tcIsSyncing() {
    return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

static void tcStartCounter() {
// Enable TC
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while (tcIsSyncing())
        ;
}

static void tcReset() {
// Reset TCx
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (tcIsSyncing())
        ;
    while (TC5->COUNT16.CTRLA.bit.SWRST)
        ;
}

static void tcDisable() {
// Disable TC5
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    while (tcIsSyncing())
        ;
}
static void tcEnd() {
    tcDisable();
    tcReset();
    analogWrite(DAC_PIN, 0);

}

static void tcConfigure(uint32_t sampleRate) {
// Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
    while (GCLK->STATUS.bit.SYNCBUSY)
        ;

    tcReset();

// Set Timer counter Mode to 16 bits
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

// Set TC5 mode as match frequency
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;

    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;

    TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
    while (tcIsSyncing())
        ;

// Configure interrupt request
    NVIC_DisableIRQ (TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);

// Enable the TC5 interrupt request
    TC5->COUNT16.INTENSET.bit.MC0 = 1;
    while (tcIsSyncing())
        ;
}

#endif
