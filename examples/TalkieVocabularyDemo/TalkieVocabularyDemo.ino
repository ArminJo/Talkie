/*
 * TalkieVocabularyDemo.cpp
 * Based on the Talkie library. https://github.com/going-digital/Talkie.
 * Copyright 2011 Peter Knight
 *
 *  SUMMARY
 *  Talkie is a speech library for Arduino.
 *  Output some sentences from the vocabularies
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

#include <Arduino.h>

#include "Talkie.h"
#include "Vocab_US_Large.h"
#include "Vocab_Special.h"
#include "Vocab_US_TI99.h"
#include "Vocab_US_Clock.h"

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
Talkie voice;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_TALKIE));

//        voice.doNotUseUseInvertedOutput();
#if defined(TEENSYDUINO)
        pinMode(5, OUTPUT);
        digitalWrite(5, HIGH); //Enable Amplified PROP shield
    #endif

    voice.say(sp3_WIND);
    voice.say(sp3_NORTHEAST);
    voice.say(sp3_GUSTING_TO);
    voice.say(sp3_FOURTY);
    voice.say(sp3_MILES);
    voice.say(sp3_PER);
    voice.say(sp3_HOUR);

}
void loop() {
    voice.say(spPAUSE2);
    voice.say(sp2_DANGER);
    voice.say(sp2_DANGER);
    voice.say(sp2_RED);
    voice.say(sp2_ALERT);
    voice.say(sp2_MOTOR);
    voice.say(sp2_IS);
    voice.say(sp2_ON);
    voice.say(sp2_FIRE);

    // US Clock words
    voice.say(spPAUSE2);
    voice.say(spc_GOOD);
    voice.say(spc_EVENING);
    voice.say(spPAUSE1);
    voice.say(spc_THE);
    voice.say(spc_TIME);
    voice.say(spc_IS);
    voice.say(spc_SEVEN);
    voice.say(spc_FOURTY);
    voice.say(spc_FIVE);
    voice.say(spc_P_M_);
    voice.say(spPAUSE1);

    // Ti99 words
    voice.say(spPAUSE2);
    voice.say(spt_CASSETTE);
    voice.say(spt_COMPLETED);
    voice.say(spPAUSE1);
    voice.say(spt_BUT);
    voice.say(spt_NOT);
    voice.say(spt_FINISHED);

}
