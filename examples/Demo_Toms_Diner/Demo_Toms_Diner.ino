// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.
//
// Armin Joachimsmeyer 11/2018 converted to .c and .h files
//
// Here's a demo of the output of my work-in-progress custom speech compressor.
//
// Tom's Diner by Suzanne Vega, over two minutes of it, in 24 kilobytes of data.
//
// It does sound a little bit compressed though...
//
// Buy the original - it's stunning.
//     http://www.amazon.com/Solitude-Standing-Suzanne-Vega/dp/B000002GHB
//     http://www.amazon.co.uk/Solitude-Standing-Suzanne-Vega/dp/B000026GZQ
//
//     or look for 'Solitude Standing' on your preferred music store.
//            (Only this album contains the original a capella version)
//
// Sound is output on digital pin 3 and/or 11. It can drive headphones directly, or add a simple audio amplifier to drive a loudspeaker.

#include <Arduino.h>

#include "Talkie.h"
#if defined(__AVR_ATmega32U4__)
// not enough memory for the data of the song left on ATmega32U4 with USB software
#include "Vocab_US_TI99.h"
#else
#include "Vocab_Toms_Diner.h"
#endif

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

#if !defined(__AVR_ATmega32U4__)
    voice.say(spDINER);
#else
    voice.say(spt_NICE_TRY);
    voice.say(spt_WE);
    voice.say(spt_NEED);
    voice.say(spt_MORE);
    voice.say(spt_MEMORY);
    voice.say(spt_THAN);
    voice.say(spt_WE);
    voice.say(spt_HAVE);
#endif
}

void loop() {
}
