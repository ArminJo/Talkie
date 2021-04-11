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
//
// Sound is output on digital pin 3 and/or 11. It can drive headphones directly, or add a simple audio amplifier to drive a loudspeaker.

#include <Arduino.h>

#include "Talkie.h"
#include "Vocab_US_TI99.h"

Talkie voice;

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
#if defined(__AVR_ATmega32U4__) || defined(SERIAL_USB) || defined(SERIAL_PORT_USBVIRTUAL)  || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_TALKIE));

//    voice.doNotUseInvertedOutput();
#if defined(TEENSYDUINO)
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH); //Enable Amplified PROP shield
#endif
    voice.say(spt_HELLO);
    voice.say(spt_THESE);
    voice.say(spt_ARE);
    voice.say(spt_THE);
    voice.say(spt_TEXAS_INSTRUMENTS);
    voice.say(spt_WORDS);
}

void loop() {
}
