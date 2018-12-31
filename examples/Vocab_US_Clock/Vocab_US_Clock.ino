// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.

//
// A female voice with an American accent
//

#include "Talkie.h"

#include "Vocab_US_Clock.h"
#include "Vocab_Special.h"

Talkie voice;


void setup() {
	voice.say(spc_GOOD);
	voice.say(spc_MORNING);
	voice.say(spPAUSE1);
	voice.say(spc_THE);
	voice.say(spc_TIME);
	voice.say(spc_IS);
	voice.say(spc_ELEVEN);
	voice.say(spc_THIRTY);
	voice.say(spc_SIX);
	voice.say(spc_A_M_);
	voice.say(spPAUSE1);
}
void loop() {
}

