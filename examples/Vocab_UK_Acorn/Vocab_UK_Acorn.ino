// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.

//
// The following phrases are derived from those built into the
// Acorn Computers Speech Synthesiser add-on from 1983.
//
// A male voice with an RP English accent, voiced by Kenneth Kendall.
//
// Due to the large vocabulary, this file takes up 16Kbytes of flash.
// To save space, just copy and paste the words you need.
//
// Sound is output on digital pin 3 and/or 11. It can drive headphones directly, or add a simple audio amplifier to drive a loudspeaker.

#include <Arduino.h>

#include "Talkie.h"
#include "Vocab_Special.h"
#include "Vocab_US_Acorn.h"

Talkie voice;

void setup() {
//    voice.doNotUseUseInvertedOutput();
#if defined(CORE_TEENSY)
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH); //Enable Amplified PROP shield
#endif
    voice.say(spa_THIS);
    voice.say(spa_IS);
    voice.say(spa_THE);
    voice.say(spa_ACORN);
    voice.say(spa_COMPUTER);
    voice.say(spa__Z);
    voice.say(spa_FILE);
    voice.say(spa_FROM);
    voice.say(spa_NINE_);
    voice.say(spa__TEEN);
    voice.say(spa_EIGH_);
    voice.say(spa_T);
    voice.say(spa_THREE);
    voice.say(spPAUSE1);
}

void loop() {
}
