/*
 *  USDistanceToVoice.cpp
 *
 *  Tells the distance measured by the ultrasonic distance sensor HCSR04 in meter, since we have no "centi" in the vocabulary.
 *
 *  Copyright (C) 2020  Armin Joachimsmeyer
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

#include "TalkieUtils.h"
#include "Vocab_US_Large.h" // for timeout
#include "Vocab_Special.h"

#include "HCSR04.h"

/* Pin mapping table for different platforms
 *
 * Platform     Normal      Inverted    8kHz timer  PWM timer
 * -------------------------------------------------------
 * AVR          3           11          1           2
 * ATmega2560   6/PH3       7/PH4       1           4
 * Leonardo     9/PB5       10/PB6      1           4
 * ProMicro     5/PC6       %           1           4 - or Adafruit Circuit Playground Classic
 * Esplora      6/PD7       %           1           4
 * Zero (SAMD)  A0          %           TC5         DAC0
 * ESP32        25          %           hw_timer_t  DAC0
 * BluePill     3           %           timer3      analogWrite Roger Clarks core
 * BluePill     PA3         %           timer4      analogWrite STM core
 * Teensy       12/14721    %         IntervalTimer analogWrite
 *
 *  As default both inverted and not inverted outputs are enabled for AVR to increase volume if speaker is attached between them.
 *  Use Talkie Voice(true, false); if you only need not inverted pin or if you want to use SPI on ATmega328 which needs pin 11.
 *
 *  The outputs can drive headphones directly, or add a simple audio amplifier to drive a loudspeaker.
 */
Talkie Voice;
//Talkie Voice(true, false);

const uint8_t ECHO_IN_PIN = 4;
const uint8_t TRIGGER_OUT_PIN = 5;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_TALKIE));

    initUSDistancePins(TRIGGER_OUT_PIN, ECHO_IN_PIN);

#if defined(TEENSYDUINO)
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH); //Enable Amplified PROP shield
#endif
    Serial.print("Voice queue size is: ");
    Serial.println(Voice.sayQ(spPAUSE1)); // this initializes the queue and the hardware

}

void loop() {

    int tCentimeter = getUSDistanceAsCentiMeterWithCentimeterTimeout(300);
    // print distance
    if (tCentimeter >= 300) {
        Serial.println("timeout");

        sayQTimeout(&Voice);
        Voice.sayQ(sp2_OUT);
    } else {
        Serial.print("cm=");
        Serial.println(tCentimeter);

        float tDistanceMeter = tCentimeter / 100.0;
        sayQFloat(&Voice, tDistanceMeter, 2, true, true);
        Voice.sayQ(sp2_METER);
    }

// Using .say() here is another way to block the sketch here and wait for end of speech as you can easily see in the source code of say().
    Voice.sayQ(spPAUSE1);
    while (Voice.isTalking()) {
        ;
    }

}
