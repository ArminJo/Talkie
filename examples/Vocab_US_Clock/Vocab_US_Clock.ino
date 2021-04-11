// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.

//
// A female voice with an American accent
//
// Sound is output on digital pin 3 and/or 11. It can drive headphones directly, or add a simple audio amplifier to drive a loudspeaker.

#include <Arduino.h>

#include "Talkie.h"
#include "Vocab_US_Clock.h"
#include "Vocab_Special.h"

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
