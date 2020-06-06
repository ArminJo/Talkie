/*
 *  _2_VoltmeterSayQ.cpp
 *
 *  Tells the voltage measured by the ADC in millivolt.
 *
 *  Copyright (C) 2011-2020  Peter Knight, Armin Joachimsmeyer
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

#include <Arduino.h>

/* Pin mapping table for different platforms
 *
 * Platform     Normal      Inverted    8kHz timer  PWM timer
 * -------------------------------------------------------
 * AVR          3           11          1           2
 * ATmega2560   6/PH3       7/PH4       1           4
 * Leonardo     9/PB5       10/PB6      1           4
 * ProMicro     5/PC6       %           1           4 - or Adafruit Circuit Playground Classic
 * Esplora      6/PD7       %           1           4
 * SAMD         14          %           TC5         DAC0
 * ESP32        25          %           hw_timer_t  DAC0
 * Teensy       12/14721    %         IntervalTimer analogWrite
 */

#include <Talkie.h>

#include <TalkieUtils.h>
#include <Vocab_Special.h>

#if defined(__AVR__)
#include "ADCUtils.h" // for getVCCVoltage()
#elif defined(ARDUINO_ARCH_SAMD)
// On the Zero and others we switch explicitly to SerialUSB
//#define Serial SerialUSB // This does not work for Zero
#endif

/*
 * Voice PWM output pins for different ATmegas:
 *  ATmega328 (Uno and Nano): non inverted at pin 3, inverted at pin 11.
 *  ATmega2560: non inverted at pin 6, inverted at pin 7.
 *  ATmega32U4 (Leonardo): non inverted at pin 10, inverted at pin 9.
 *  ATmega32U4 (CircuitPlaygound): only non inverted at pin 5.
 *
 *  As default both inverted and not inverted outputs are enabled to increase volume if speaker is attached between them.
 *  Use Talkie Voice(true, false); if you only need not inverted pin or if you want to use SPI on ATmega328 which needs pin 11.
 *
 *  The outputs can drive headphones directly, or add a simple audio amplifier to drive a loudspeaker.
 */
Talkie Voice;
//Talkie Voice(true, false);
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
    while (!Serial)
        ; //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_TALKIE));

#if defined(TEENSYDUINO)
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH); //Enable Amplified PROP shield
#elif defined(ARDUINO_ARCH_SAMD)
    analogReadResolution(12);
#endif
    Serial.print("Voice queue size is: ");
    Serial.println(Voice.sayQ(spPAUSE1)); // this initializes the queue and the hardware

}

void loop() {

#if defined(__AVR__)
    float tVCCVoltage = getVCCVoltage();
    Serial.print(tVCCVoltage);
    Serial.println(" volt VCC");

    int tVoltage = analogRead(A0) * tVCCVoltage / 1.023;
#elif defined(ESP32)
    int tVoltage = analogRead(A0) * 3.3 / 4.096;
#elif defined(ARDUINO_ARCH_SAMD)
    int tVoltage = analogRead(A1) * 3.3 / 4.096; // A0 is DAC output
#else
    int tVoltage = analogRead(0) * 3.3 / 1.023;
#endif

    Serial.print(tVoltage);
    Serial.println(" mV input");

//    sayQVoltageMilliVolts(&Voice, tVoltage);
    float tVoltageFloat = tVoltage / 1000.0;
    sayQVoltageVolts(&Voice, tVoltageFloat);
    // Using .say() here is another way to block the sketch here and wait for end of speech as you can easily see in the source code of say().
    Voice.sayQ(spPAUSE1);
    while (Voice.isTalking()) {
        ;
    }
}
