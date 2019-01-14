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

#include "Talkie.h"
#include "Vocab_US_Large.h"
#include "Vocab_Special.h"
#include "Vocab_Soundbites.h"

Talkie voice;

void setup() {
//        voice.doNotUseUseInvertedOutput();
    #if defined(CORE_TEENSY)
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

    // The strange soundbites demo
    voice.say(spPAUSE2);
    voice.say(spWHAT_IS_THY_BIDDING);
    voice.say(spHASTA_LA_VISTA);
    voice.say(spONE_SMALL_STEP);
    voice.say(spHMMM_BEER);
}
