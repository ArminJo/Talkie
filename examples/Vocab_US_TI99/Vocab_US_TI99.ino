// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.

//
// The following phrases are derived from those built into the
// Texas Instruments TI99/4A Speech System add-on from 1979.
//
// A deep male voice with a southern USA accent.
//
// Due to the large vocabulary, this file takes up 32Kbytes of flash.
// It will not fit in most Arduinos as is, so just copy and paste
// out the words you need.
//
// Note that some words/letters are repeated with different spellings.
// eg. 'TWO', 'TO', 'TOO' or 'YOU' and 'U'

#include "Talkie.h"
#include "Vocab_US_TI99.h"

Talkie voice;

void setup() {
    voice.say(spt_HELLO);
    voice.say(spt_THESE);
    voice.say(spt_ARE);
    voice.say(spt_THE);
    voice.say(spt_TEXAS_INSTRUMENTS);
    voice.say(spt_WORDS);
}
void loop() {
}