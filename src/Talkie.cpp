/*
 * Talkie.cpp
 * Based on the Talkie library. https://github.com/going-digital/Talkie.
 * Copyright 2011 Peter Knight
 * The code for the queued non blocking version is based on the fork of Paul Stoffregen. https://github.com/PaulStoffregen/Talkie
 *
 *  SUMMARY
 *  Talkie is a speech library for Arduino.
 *  The analog output value is created by an 8 bit PWM, the sample rate is 8 kHz / 125 us.
 *
 *  On a plain ATmega, output is at pin 3 + 11 (of Timer 2) which are both enabled by default.
 *  It can also run on 8 MHz ATmega with either FAST_8BIT_MODE defined and slightly reduces speech quality or timer0 (for millis()) disabled  which is default.
 *
 *  You get increased volume if you use both outputs and connect speaker between non inverted (3) and inverted (11) output pin.
 *
 *  Timer 1 (16 bit - for Servo library) is used to fill in new values for the PWM at the sample rate of 8000 Hz / 125 us.
 *
 *  On a plain Arduino: Timer 2 (8 bit - for Tone library) is used to generate the 62500 Hz / 16 us PWM with 8 bit resolution on pin 3/PD3/OC2B + 11/PB3/OC2A.
 *
 *  On ATmega2560: Timer 4 is used to generate the 62500 Hz / 16 us PWM with 8 bit resolution on pin 6/PH3/OC4A + 7/PH4/OC4B.
 *
 *  On ATmega32U4: Timer 4 for 200 kHz / 5 us PWM at pin 9/PB5/!OC4B + 10/PB6/OC4B for Leonardo board
 *                                              at pin 5/PC6/!OC4A for Adafruit Circuit Playground Classic or Sparkfun Pro Micro board
 *                                              at pin 6/PD7/OC4D for Esplora board
 *
 * OUTPUT FILTER:
 *
 *     C to avoid clicks  Low pass 1600 Hz  DC decoupling (optional)
 *                      _____
 *  D3 >------||-------|_____|---+-----------||-------> to Power amplifier
 *           100nF       10k     |          10nF
 *                              ---
 *                              --- 10 nF
 *                               |
 *                               |
 *                               _ GND
 *
 * Pin mapping table for different platforms
 *
 * Platform     Normal      Inverted    8kHz timer  PWM timer
 * -------------------------------------------------------
 * AVR          3           11          1           2
 * ATmega2560   6/PH3       7/PH4       1           4
 * Leonardo     9/PB5       10/PB6      1           4
 * ProMicro     5/PC6       %           1           4 - or Adafruit Circuit Playground Classic
 * Esplora      6/PD7       %           1           4
 * Zero (SAMD)  A0          %           TC5         A0/DAC0
 * ESP32        25          %           hw_timer_t  DAC0
 * BluePill     3           %           timer3      analogWrite Roger Clarks core
 * BluePill     PA3         %           timer4      analogWrite STM core
 * Teensy       12/14721    %         IntervalTimer analogWrite
 *
 *  As default both inverted and not inverted outputs are enabled for AVR to increase volume if speaker is attached between them.
 *  Use Talkie Voice(true, false); if you only need not inverted pin or if you want to use SPI on ATmega328 which needs pin 11.
 *
 *  The outputs can drive headphones directly, or add a simple audio amplifier to drive a loudspeaker.
 *
 *  Copyright (C) 2018-2021  Armin Joachimsmeyer
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

#include <Arduino.h>

#include "Talkie.h"

// Enable this if you want to measure timing by toggling pin 8 on an Arduino
//#define MEASURE_TIMING
#ifdef MEASURE_TIMING
#include "digitalWriteFast.h"
#  if defined(ARDUINO_ARCH_STM32)
#define TIMING_PIN PA8
#  else
#define TIMING_PIN 8
#  endif
#endif

#include "TalkieLPC.h"

#define SAMPLE_RATE 8000 // Speech engine sample rate

static uint8_t synthPeriod;
static uint16_t synthEnergy;

#if defined(USE_10_BIT_KOEFFICIENT_VALUES)
static int32_t synthK1, synthK2;
static int32_t synthK3, synthK4, synthK5, synthK6, synthK7, synthK8, synthK9, synthK10;
#  else
#  ifdef FAST_8BIT_MODE
static int8_t synthK1, synthK2;
#  else
static int16_t synthK1, synthK2;
#  endif
static int8_t synthK3, synthK4, synthK5, synthK6, synthK7, synthK8, synthK9, synthK10;
#endif

#define ISR_RATIO (25000/ (1000000 / SAMPLE_RATE) ) // gives 200 for SAMPLE_RATE 8000 -> 40 Hz or 25 ms Sample update frequency

#ifdef __cplusplus
extern "C" {
#endif
void timerInterrupt(void);
#ifdef __cplusplus
}
#endif

#if defined(TEENSYDUINO)
IntervalTimer sIntervalTimer;

#elif  defined(ESP32)
#include <driver/dac.h>
static hw_timer_t *sTalkieSampleRateTimer = NULL;

#elif defined(ARDUINO_ARCH_SAMD) // Zero
static void tcStart(uint32_t sampleRate); // TC5
static void tcEnd();

#elif defined(__STM32F1__) || defined(ARDUINO_ARCH_STM32F1)
#include <HardwareTimer.h> // 4 timers and 4. timer (4.channel) is used for tone()
/*
 * Use timer 3 as sample rate timer.
 * Timer 3 blocks PA6, PA7, PB0, PB1, so if you require one of them as tone() or Servo output, you must choose another timer.
 */
HardwareTimer sTalkieSampleRateTimer(3);
timer_dev *sTalkiePWMTimer;
uint8_t sTalkiePWMTimerChannel;

#elif defined(STM32F1xx) || defined(ARDUINO_ARCH_STM32)
#include <HardwareTimer.h> // 4 timers and 3. timer is used for tone(), 2. for Servo
/*
 * Use timer 4 as IRMP timer.
 * Timer 4 blocks PB6, PB7, PB8, PB9, so if you require one of them as tone() or Servo output, you must choose another timer.
 */
#  if defined(TIM4)
HardwareTimer sTalkieSampleRateTimer(TIM4);
#  else
HardwareTimer sTalkieSampleRateTimer(TIM2);
#  endif

#endif

static void setNextSynthesizerData();

static Talkie *sPointerToTalkieForISR;
#if ISR_RATIO < 255
static uint8_t ISRCounterToNextData = 0;
#else
static uint16_t ISRCounterToNextData = 0;
#endif

/*
 * Pin numbers must only used here!
 */
void Talkie::initializeHardware() {
// Enable the speech system whenever say() is called.
#if defined(__AVR__)

#if defined(__AVR_ATmega32U4__) // Use Timer 4 instead of Timer 2
#  if defined(ARDUINO_AVR_CIRCUITPLAY) || defined(ARDUINO_AVR_PROMICRO)
// Adafruit Circuit Playground Classic or Sparkfun Pro Micro board. The first does not need inverted output, the latter does not connect it.
// Cannot be used on plain Leonardos because inverted output is connected to internal led.
    NonInvertedOutputPin = 5;// D5/PC6/!OC4A
#define PWM_VALUE_DESTINATION OCR4A
    InvertedOutputPin = TALKIE_DO_NOT_USE_PIN_FLAG; // disable InvertedOutputPin
    pinMode(NonInvertedOutputPin, OUTPUT);
    TCCR4A = _BV(COM4A0) | _BV(PWM4A);// Clear on match, PWMA on, OC4A/PC7 & !OC4A/PC6 connected

#  elif defined(ARDUINO_AVR_ESPLORA)
    NonInvertedOutputPin = 6; // Only D6/PD7/OC4D connected to Speaker
#define PWM_VALUE_DESTINATION OCR4D
    InvertedOutputPin = TALKIE_DO_NOT_USE_PIN_FLAG; // disable InvertedOutputPin
    pinMode(NonInvertedOutputPin, OUTPUT);
    TCCR4C = _BV(COM4D1) |_BV(PWM4D);

#  else
// Leonardo, Lilypad USB, FLORA, TEENSY
    NonInvertedOutputPin = 10;// D10/OC4B/PB6
#define PWM_VALUE_DESTINATION OCR4B
    pinMode(NonInvertedOutputPin, OUTPUT);
    if (InvertedOutputPin) {
        // use both output
        InvertedOutputPin = 9;// D9/!OC4B/PB5
        pinMode(InvertedOutputPin, OUTPUT);
        TCCR4A = _BV(COM4B0) | _BV(PWM4B);// Clear on match, PWMA on, OC4A & !OC4A connected
    } else {
        // use only non Inverted output
        TCCR4A = _BV(COM4B1) | _BV(PWM4B);// Clear on match, PWMA on, OC4A connected
    }
#  endif // ATmega32U4 flavors

// common ATmega32U4
// Set up Timer4 for fast PWM 200 kHz / 5 us
    PLLFRQ = PLLFRQ | _BV(PLLTM1) | _BV(PLLTM0);// Route PLL to async clk, PLL Postcaler for High Speed Timer = 2
    TCCR4B = _BV(CS40);//  1:1 prescale
    TCCR4D = 0;// Fast PWM mode
    TCCR4E = 0;// Not enhanced mode
    TC4H = 0;// Not 10-bit mode
    DT4 = 0;// No dead time
    OCR4C = 255;// TOP
    OCR4A = 127;// 50% duty to start
    OCR4B = 127;// 50% duty to start

#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TCCR4A = _BV(WGM40); // Fast PWM 8 Bit
    TCCR4B = _BV(WGM42) | _BV(CS40);// Fast PWM 8 Bit, direct clock

    if (NonInvertedOutputPin) {
        NonInvertedOutputPin = 6; // OC4A
        pinMode(NonInvertedOutputPin, OUTPUT);
        TCCR4A |= _BV(COM4A1);// OC4A non-inverting mode
    }
    if (InvertedOutputPin) {
        InvertedOutputPin = 7; // OC4B
        pinMode(InvertedOutputPin, OUTPUT);
        TCCR4A |= _BV(COM4B1) | _BV(COM4B0);// OC4B inverting mode
    }
#define PWM_VALUE_DESTINATION OCR4A
#define PWM_INVERTED_VALUE_DESTINATION OCR4B

#else // __AVR_ATmega32U4__
    /*
     * Plain ATmega e.g. 328P here
     *
     * Timer 2 set up as a 62500 Hz / 16 us 8Bit PWM for 16 MHz.
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
#define PWM_VALUE_DESTINATION OCR2B
#define PWM_INVERTED_VALUE_DESTINATION OCR2A

    TCCR2B = _BV(CS20); // direct clock
    TIMSK2 = 0;
#endif // __AVR_ATmega32U4__

    /*
     * Common code for all AVR
     */
// Setup sample rate Timer1
    /*
     * Unfortunately we can't calculate the next sample every PWM cycle
     * as the routine is too slow. So use Timer 1 to trigger that.
     * Timer 1 set up as a 8000 Hz / 125 us sample interrupt
     */
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10); // CTC mode, no prescale
    TCNT1 = 0;
#if F_CPU <= 8000000L
    TIMSK0 = 0; // // Tweak for 8 MHz clock - must disable millis() interrupt
#endif
    OCR1A = ((F_CPU + (SAMPLE_RATE / 2)) / SAMPLE_RATE) - 1;  // 'SAMPLE_RATE' Hz (w/rounding)
    OCR1B = ((F_CPU + (SAMPLE_RATE / 2)) / SAMPLE_RATE) - 1;  // use the same value for register B
    TIMSK1 = _BV(OCIE1B); // enable compare register B match interrupt to use TIMER1_COMPB_vect and not interfere with the Servo library

#elif defined(ARDUINO_ARCH_SAMD) // Zero
#define _10_BIT_OUTPUT // 10-bit, 350 ksps Digital-to-Analog Converter (DAC)
#if defined DAC0
#define DAC_PIN DAC0   // pin 14/A0 for Zero. PA02 + DAC1 on DUE
#else
#define DAC_PIN A0   // On some SAMD boards DAC0 definition is missing
#endif
#define PWM_OUTPUT_FUNCTION(nextPwm) analogWrite(sPointerToTalkieForISR->NonInvertedOutputPin, nextPwm)
#  ifdef ARDUINO_SAMD_CIRCUITPLAYGROUND_EXPRESS
    static const int CPLAY_SPEAKER_SHUTDOWN= 11;
    pinMode(CPLAY_SPEAKER_SHUTDOWN, OUTPUT);
    digitalWrite(CPLAY_SPEAKER_SHUTDOWN, HIGH);
#  endif
    analogWriteResolution(10); // 10-bit, 350 ksps Digital-to-Analog Converter (DAC)
    analogWrite(sPointerToTalkieForISR->NonInvertedOutputPin, 1 << 9); // DAC0
    tcStart(SAMPLE_RATE);

#elif defined(TEENSYDUINO)
    // common for all Teensy
#define _12_BIT_OUTPUT

#  if defined(__MKL26Z64__) // Teensy LC
#define DAC_PIN A12

#  elif defined(__MK20DX256__) // Teensy 3.1 / 3.2
#define DAC_PIN A14

#  elif defined(__MK64FX512__) || defined(__MK66FX1M0__) // Teensy 3.5 / 3.6
#define DAC_PIN A21 // Or A22
#  endif // defined(__MKL26Z64__)

    sIntervalTimer.begin(timerInterrupt, 1000000L / SAMPLE_RATE);

#  if defined(DAC_PIN)
#define PWM_OUTPUT_FUNCTION(nextPwm) analogWrite(sPointerToTalkieForISR->NonInvertedOutputPin, nextPwm)
    analogWriteResolution(12);
#  else // Teensy 3.0, Teensy 4.0 / 4.1
#define PWM_OUTPUT_FUNCTION(nextPwm) analogWrite(A6, nextPwm)
    analogWriteFrequency(NonInvertedOutputPin, 62500);
#  endif // (DAC_PIN)

#elif defined(ESP32)
#define DAC_PIN 25 // Or 26
#define PWM_OUTPUT_FUNCTION(nextPwm) dacWrite(sPointerToTalkieForISR->NonInvertedOutputPin, nextPwm)
    // Use Timer1 with 1 microsecond resolution, main APB clock is 80MHZ
#define APB_FREQUENCY_DIVIDER 80
    sTalkieSampleRateTimer = timerBegin(1, APB_FREQUENCY_DIVIDER, true);
    timerAttachInterrupt(sTalkieSampleRateTimer, timerInterrupt, true);
    timerAlarmWrite(sTalkieSampleRateTimer, (getApbFrequency() / APB_FREQUENCY_DIVIDER) / SAMPLE_RATE, true);
    timerAlarmEnable(sTalkieSampleRateTimer);
#if defined(DEBUG) && defined(ESP32)
    Serial.print("CPU frequency=");
    Serial.print(getCpuFrequencyMHz());
    Serial.println("MHz");
    Serial.print("Timer clock frequency=");
    Serial.print(getApbFrequency());
    Serial.println("Hz");
#endif
    // not required!
//    dac_output_enable(DAC_CHANNEL_1);
//    dac_output_voltage(DAC_CHANNEL_1, 255);

    // BluePill in 2 flavors see https://samuelpinches.com.au/3d-printer/cutting-through-some-confusion-on-stm32-and-arduino/
#elif defined(__STM32F1__) || defined(ARDUINO_ARCH_STM32F1) // Recommended original Arduino_STM32 by Roger Clark.
    // STM32F1 architecture for "Generic STM32F103C series" from "STM32F1 Boards (Arduino_STM32)" of Arduino Board manager
    // http://dan.drown.org/stm32duino/package_STM32duino_index.json
#define DAC_PIN PA3      // T2C4
#define _10_BIT_OUTPUT
#define PWM_OUTPUT_FUNCTION(nextPwm) timer_set_compare(sTalkiePWMTimer, sTalkiePWMTimerChannel, nextPwm)

    /*
     * Prepare 10 bit PWM @ 72 MHz
     */
    sTalkiePWMTimerChannel = PIN_MAP[sPointerToTalkieForISR->NonInvertedOutputPin].timer_channel; // set timer channel according to pin
    sTalkiePWMTimer = PIN_MAP[sPointerToTalkieForISR->NonInvertedOutputPin].timer_device; // set timer according to pin
    pinMode(sPointerToTalkieForISR->NonInvertedOutputPin, PWM); // this initializes the output pin and the timer mode
    timer_set_prescaler(sTalkiePWMTimer, 0); // set prescaler to 1
    timer_set_reload(sTalkiePWMTimer, (1 << 10) - 1); // setOverflow()
    timer_set_compare(sTalkiePWMTimer, sTalkiePWMTimerChannel, (1 << 9));
    timer_resume(sTalkiePWMTimer);  // Start timer
    timer_generate_update(sTalkiePWMTimer); // Reset to start values

    /*
     * Set timer for interrupts at SAMPLE_RATE
     */
    sTalkieSampleRateTimer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
    sTalkieSampleRateTimer.setPrescaleFactor(1);
    sTalkieSampleRateTimer.setOverflow(F_CPU / SAMPLE_RATE);
    sTalkieSampleRateTimer.attachInterrupt(TIMER_CH1, timerInterrupt);
    sTalkieSampleRateTimer.resume();  // Start timer
    sTalkieSampleRateTimer.refresh(); // Reset to start values

#elif defined(STM32F1xx) || defined(ARDUINO_ARCH_STM32)
    // STM32duino by ST Microsystems.
    // https://github.com/stm32duino/BoardManagerFiles/raw/master/STM32/package_stm_index.json
    // stm32 architecture for "Generic STM32F1 series" from "STM32 Boards (selected from submenu)" of Arduino Board manager
#define DAC_PIN PA3      // T2C4
#define PWM_OUTPUT_FUNCTION(nextPwm) analogWrite(sPointerToTalkieForISR->NonInvertedOutputPin, nextPwm)
#define _10_BIT_OUTPUT
    analogWriteResolution(10);
    analogWriteFrequency(F_CPU / (1 << 10)); // 70312 10 bit at 72 MHz (70312.5)
//    analogWriteFrequency(F_CPU / (1 << 8)); // 281250 8 bit at 72 MHz
    /*
     * Set timer for interrupts at SAMPLE_RATE
     */
    sTalkieSampleRateTimer.setOverflow(1000000L / SAMPLE_RATE, MICROSEC_FORMAT);
    sTalkieSampleRateTimer.attachInterrupt(timerInterrupt);
    sTalkieSampleRateTimer.resume();  // Start timer
    sTalkieSampleRateTimer.refresh(); // Reset to start values
#endif // AVR

#ifdef MEASURE_TIMING
    pinMode(TIMING_PIN, OUTPUT);
#endif

    isTalkingFlag = true;
}

#if ! defined(PWM_VALUE_DESTINATION) && ! defined(PWM_OUTPUT_FUNCTION)
#error One of PWM_VALUE_DESTINATION or PWM_OUTPUT_FUNCTION must be defined in initializeHardware()
#endif

void Talkie::terminateHardware() {
#if defined(__AVR__)
#  if F_CPU <= 8000000
    TIMSK0 = _BV(TOIE0); // // Tweak for 8 MHz clock - enable millis() interrupt again
#  endif
    TIMSK1 = 0; // disable interrupts

#  if defined(__AVR_ATmega32U4__)
    TCCR4A = 0; // disconnect outputs
#  elif defined(ARDUINO_AVR_ESPLORA)
    TCCR4C = 0; // disconnect outputs
#  else
    TCCR2A = 0; // disconnect outputs
#  endif

    /*
     * Call noTone() to force initializing of tone library for the next time tone() is called.
     * noTone() disconnect the pin from timer, and write a 0 to the pin.
     * pinMode(3|11, INPUT) avoids the click at the end, if speaker is coupled by a capacitance.
     * next tone() sets pin as output again.
     */
    if (NonInvertedOutputPin) {
        // Reset pin 11 to input only if no active SPI detected
        if (!(SPCR & _BV(SPE))) {
            pinMode(NonInvertedOutputPin, INPUT);
            // force initializing of tone library for the next time tone() is called.
#ifndef NO_COMPATIBILITY_FOR_TONE_LIB_REQUIRED // enable it to save Flash
            noTone(NonInvertedOutputPin);
#endif
        }
    }
    if (InvertedOutputPin) {
        pinMode(InvertedOutputPin, INPUT);
#ifndef NO_COMPATIBILITY_FOR_TONE_LIB_REQUIRED
        noTone(InvertedOutputPin);
#endif
    }

    #elif defined(ARDUINO_ARCH_SAMD) // Zero
    tcEnd();

#elif defined(ESP32)
    timerAlarmDisable(sTalkieSampleRateTimer);

#elif defined(TEENSYDUINO)
    sIntervalTimer.end();

#elif defined(__STM32F1__) || defined(ARDUINO_ARCH_STM32F1) || defined(STM32F1xx) || defined(ARDUINO_ARCH_STM32)
    sTalkieSampleRateTimer.pause();

#endif // defined(__AVR__)

    isTalkingFlag = false;
}

Talkie::Talkie() { // @suppress("Class members should be properly initialized")
    /*
     * Enable non inverted and inverted output by default
     */
#if defined(DAC_PIN)
    NonInvertedOutputPin = DAC_PIN; // initialize with DAC pin
#else
    NonInvertedOutputPin = TALKIE_USE_PIN_FLAG;
#endif
    InvertedOutputPin = TALKIE_USE_PIN_FLAG;

    isTalkingFlag = false;
    sPointerToTalkieForISR = this;
}

Talkie::Talkie(bool aUseNonInvertedOutputPin, bool aUseInvertedOutputPin) { // @suppress("Class members should be properly initialized")
    if (aUseNonInvertedOutputPin) {
#if defined(DAC_PIN)
        NonInvertedOutputPin = DAC_PIN; // initialize with DAC pin
#else
        NonInvertedOutputPin = TALKIE_USE_PIN_FLAG;
#endif
    }
    if (aUseInvertedOutputPin) {
        InvertedOutputPin = TALKIE_USE_PIN_FLAG;
    }

    isTalkingFlag = false;
    sPointerToTalkieForISR = this;
}

// To be compatible to Teensy library
void Talkie::beginPWM(uint8_t aPinPWM) {
    NonInvertedOutputPin = aPinPWM;
}

void Talkie::setPtr(const uint8_t *aAddress) {
    WordDataPointer = aAddress;
    WordDataBit = 0;
}

/**
 * Returns 0 if nothing to play, otherwise the number of the queued items plus the one which is active.
 */
uint8_t Talkie::getNumberOfWords() {
    if (!isTalkingFlag) {
        return 0;   // Nothing playing!
    } else {
        return (1 + (FIFO_BUFFER_SIZE - free)); // 1 active plus X in queue
    }
}

bool Talkie::isTalking() {
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

//// https://github.com/going-digital/Talkie/issues/32
//uint8_t Talkie::getBits(uint8_t bits) {
//    uint8_t value = 0;
//    for (; bits; WordDataBit--, WordData >>= 1, bits--) {
//        value <<= 1; // NULL on first pass
//        if (!WordDataBit) { // regular byte rollover junk...
//            WordData = pgm_read_byte(WordDataPointer);
//            WordDataPointer++;
//            WordDataBit = 8;
//        }
//        value |= (WordData & 0x01); // OR a 1 into value if the MSB of WordData is set
//    }
//    return value;
//}

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
const uint8_t* Talkie::FIFOPopFront() {
// 56 bytes compiled
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
void Talkie::say(const uint8_t *aWordDataAddress) {
    sayQ(aWordDataAddress);
    wait();
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
 * On Arduino disables/enables pin 11 as inverted PWM output ( in order to increase the volume, if speaker is attached between 3 and 11)
 */
void Talkie::doNotUseInvertedOutput(bool aDoNotUseInvertedOutput) {
    if (aDoNotUseInvertedOutput) {
        InvertedOutputPin = TALKIE_DO_NOT_USE_PIN_FLAG;
    } else {
        InvertedOutputPin = TALKIE_USE_PIN_FLAG;
    }
}

/*
 * On Arduino disables/enables pin 3 (Talkie default) as PWM output
 */
void Talkie::doNotUseNonInvertedOutput(bool aDoNotUseNonInvertedOutput) {
    if (aDoNotUseNonInvertedOutput) {
        NonInvertedOutputPin = TALKIE_DO_NOT_USE_PIN_FLAG;
    } else {
        NonInvertedOutputPin = TALKIE_USE_PIN_FLAG;
    }
}

void Talkie::digitalWriteInvertedOutput(uint8_t aValue) {
    if (InvertedOutputPin != 0) {
        pinMode(InvertedOutputPin, OUTPUT);
        digitalWrite(InvertedOutputPin, aValue);
    }
}

void Talkie::digitalWriteNonInvertedOutput(uint8_t aValue) {
    if (NonInvertedOutputPin != 0) {
        pinMode(NonInvertedOutputPin, OUTPUT);
        digitalWrite(NonInvertedOutputPin, aValue);
    }
}

/*
 * The non blocking version as long as there is room in queue.
 * if queue is full then do a blocking wait for the queue to have room.
 * If talking, it just adds the aWordDataAddress to the queue by calling FIFOPushBack(aWordDataAddress)
 * If stopped it starts the speech output.
 * if aWordDataAddress is 0 it just clears the queue.
 */
int8_t Talkie::sayQ(const uint8_t *aWordDataAddress) {

    if (aWordDataAddress == 0) {
        // Caller asked to have queue made empty and sound stopped
        stop();
    } else {
        noInterrupts();
        // disable Interrupt because ptrAddr is also modified by ISR. This avoids race conditions.
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
 * Computes next output value
 * Called every 125 microsecond / 8000 Hz
 * 75 to 90 (when calling setNextSynthesizerData()) microseconds processing time @16 MHz
 * 50 microseconds with 4 simple optimizations (change ">>15" to "<<1) >>16")
 * 8 us for a BluePill with Roger Clarks core
 * 12 us for a BluePill with official STM core
 * 15 us for Arduino Zero @ 48 MHz
 */
#if defined(ESP32)
IRAM_ATTR
#endif
void timerInterrupt() {
// 1286 byte compiled
#ifdef MEASURE_TIMING
    digitalWriteFast(TIMING_PIN, HIGH);
#endif

    static uint8_t periodCounter;
#if !defined(_10_BIT_OUTPUT) && !defined(_12_BIT_OUTPUT)
    static uint8_t nextPwm;
#else
    static uint16_t nextPwm;
#endif
#if defined(USE_10_BIT_KOEFFICIENT_VALUES)
    static int32_t x0, x1, x2, x3, x4, x5, x6, x7, x8, x9;
    int32_t u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, u10;
#else
    static int16_t x0, x1, x2, x3, x4, x5, x6, x7, x8, x9;
    int16_t u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, u10;
#endif

    /*
     * First output the value
     */
#if defined(PWM_VALUE_DESTINATION)
    PWM_VALUE_DESTINATION = nextPwm;
#endif
#if defined(PWM_INVERTED_VALUE_DESTINATION)
    PWM_INVERTED_VALUE_DESTINATION = nextPwm;
#endif
#if defined(PWM_OUTPUT_FUNCTION)
    PWM_OUTPUT_FUNCTION(nextPwm);
#endif

    if (synthPeriod) {
        // Voiced source
        if (periodCounter < synthPeriod) {
            periodCounter++;
        } else {
            periodCounter = 0;
        }
        if (periodCounter < sizeof(chirp)) {
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
// rescale by shifting >> 7 because we have signed values here (for unsigned it would require >> 8)
#if !defined(USE_10_BIT_KOEFFICIENT_VALUES)
    u9 = u10 - (((int16_t) synthK10 * x9) >> 7);
    u8 = u9 - (((int16_t) synthK9 * x8) >> 7);
    u7 = u8 - (((int16_t) synthK8 * x7) >> 7);
    u6 = u7 - (((int16_t) synthK7 * x6) >> 7);
    u5 = u6 - (((int16_t) synthK6 * x5) >> 7);
    u4 = u5 - (((int16_t) synthK5 * x4) >> 7);
    u3 = u4 - (((int16_t) synthK4 * x3) >> 7);
    u2 = u3 - (((int16_t) synthK3 * x2) >> 7);
#  ifdef FAST_8BIT_MODE
    u1 = u2 - (((int16_t) synthK2 * x1) >> 7);
    u0 = u1 - (((int16_t) synthK1 * x0) >> 7);
#  else
// the 4 changes from ">> 15" to "<<1) >> 16" save 25 us processing time at 16 MHz :-)
    u1 = u2 - ((((int32_t) synthK2 * x1) << 1) >> 16);
    u0 = u1 - ((((int32_t) synthK1 * x0) << 1) >> 16);
#  endif
#else

    u9 = u10 - ((synthK10 * x9) >> 9);
    u8 = u9 - ((synthK9 * x8) >> 9);
    u7 = u8 - ((synthK8 * x7) >> 9);
    u6 = u7 - ((synthK7 * x6) >> 9);
    u5 = u6 - ((synthK6 * x5) >> 9);
    u4 = u5 - ((synthK5 * x4) >> 9);
    u3 = u4 - ((synthK4 * x3) >> 9);
    u2 = u3 - ((synthK3 * x2) >> 9);
    u1 = u2 - ((synthK2 * x1) >> 9);
    u0 = u1 - ((synthK1 * x0) >> 9);
#endif

// Lattice filter reverse path -> compute next x* values
#if !defined(USE_10_BIT_KOEFFICIENT_VALUES)
    x9 = x8 + (((int16_t) synthK9 * u8) >> 7);
    x8 = x7 + (((int16_t) synthK8 * u7) >> 7);
    x7 = x6 + (((int16_t) synthK7 * u6) >> 7);
    x6 = x5 + (((int16_t) synthK6 * u5) >> 7);
    x5 = x4 + (((int16_t) synthK5 * u4) >> 7);
    x4 = x3 + (((int16_t) synthK4 * u3) >> 7);
    x3 = x2 + (((int16_t) synthK3 * u2) >> 7);
#  ifdef FAST_8BIT_MODE
    x2 = x1 + (((int16_t) synthK2 * u1) >> 7);
    x1 = x0 + (((int16_t) synthK1 * u0) >> 7);
#  else
    x2 = x1 + ((((int32_t) synthK2 * u1) << 1) >> 16);
    x1 = x0 + ((((int32_t) synthK1 * u0) << 1) >> 16);
#  endif
#else
    x9 = x8 + ((synthK9 * u8) >> 9);
    x8 = x7 + ((synthK8 * u7) >> 9);
    x7 = x6 + ((synthK7 * u6) >> 9);
    x6 = x5 + ((synthK6 * u5) >> 9);
    x5 = x4 + ((synthK5 * u4) >> 9);
    x4 = x3 + ((synthK4 * u3) >> 9);
    x3 = x2 + ((synthK3 * u2) >> 9);
    x2 = x1 + ((synthK2 * u1) >> 9);
    x1 = x0 + ((synthK1 * u0) >> 9);
#endif

    x0 = u0; // 10 bit value -512 to +511

#if defined(_10_BIT_OUTPUT)
    nextPwm = (u0) + 0x200;
#elif defined(_12_BIT_OUTPUT)
    nextPwm = (u0 * 4) + 0x800;
#else
    // 8 bit is default
    nextPwm = (u0 >> 2) + 0x80;
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

#if defined(ARDUINO_ARCH_SAMD)
    TC5->COUNT16.INTFLAG.bit.MC0 = 1;
#endif

#ifdef MEASURE_TIMING
    digitalWriteFast(TIMING_PIN, LOW);
#endif
}
#if defined(ARDUINO_ARCH_SAMD)
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
            // cast with (int16_t) to support int32_t type for synthK1 etc. since pgm_read_word returns an unsigned value.
            synthK1 = (int16_t) pgm_read_word(&tmsK1[sPointerToTalkieForISR->getBits(5)]);
            synthK2 = (int16_t) pgm_read_word(&tmsK2[sPointerToTalkieForISR->getBits(5)]);
#if !defined(USE_10_BIT_KOEFFICIENT_VALUES)
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
#else
            synthK3 = (int16_t) pgm_read_word(&tmsK3[sPointerToTalkieForISR->getBits(4)]);
            synthK4 = (int16_t) pgm_read_word(&tmsK4[sPointerToTalkieForISR->getBits(4)]);
            if (synthPeriod) {
                // Voiced frames use 6 extra coefficients.
                synthK5 = (int16_t) pgm_read_word(&tmsK5[sPointerToTalkieForISR->getBits(4)]);
                synthK6 = (int16_t) pgm_read_word(&tmsK6[sPointerToTalkieForISR->getBits(4)]);
                synthK7 = (int16_t) pgm_read_word(&tmsK7[sPointerToTalkieForISR->getBits(4)]);
                synthK8 = (int16_t) pgm_read_word(&tmsK8[sPointerToTalkieForISR->getBits(3)]);
                synthK9 = (int16_t) pgm_read_word(&tmsK9[sPointerToTalkieForISR->getBits(3)]);
                synthK10 = (int16_t) pgm_read_word(&tmsK10[sPointerToTalkieForISR->getBits(3)]);
            }
#endif
        }
    }
}

#if defined(ARDUINO_ARCH_SAMD)
static void tcReset() {
    // Reset TCx
    TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (TC5->COUNT16.STATUS.bit.SYNCBUSY)
        ;
    while (TC5->COUNT16.CTRLA.bit.SWRST)
        ;
}

static void tcEnd() {
    // Disable TC5
    TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
    tcReset();
#if defined _10_BIT_OUTPUT
    PWM_OUTPUT_FUNCTION(0x200);
#elif defined(_12_BIT_OUTPUT)
    PWM_OUTPUT_FUNCTION(0x800);
#endif
}

static void tcStart(uint32_t sampleRate) {
// Enable GCLK for TCC2 and TC5 (timer counter input clock)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)); // GCLK1=32kHz,  GCLK0=48MHz
    //    while (GCLK->STATUS.bit.SYNCBUSY) // not required to wait
    //        ;
    tcReset();

// Set Timer counter Mode to 16 bits, Set TC5 mode as match frequency
    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_MFRQ | TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
    TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
    //    while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY) // The next commands do an implicit wait :-)
    //        ;

// Configure interrupt request
    NVIC_DisableIRQ(TC5_IRQn);
    NVIC_ClearPendingIRQ(TC5_IRQn);
    NVIC_SetPriority(TC5_IRQn, 0);
    NVIC_EnableIRQ(TC5_IRQn);

// Enable the TC5 interrupt request
    TC5->COUNT16.INTENSET.bit.MC0 = 1;

    TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    //    while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY) // Not required to wait at end of function
    //        ; // wait until TC5 is done syncing
}

#endif
