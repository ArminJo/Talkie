#include <Arduino.h>

// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.
//
// Now for something a bit more challenging.
//
// Building sentences by program.
//
// The sayQNumber() function can sayQ any number under a million by
// building the number from short phrases,
//
// Connect a sensor to Analog 0, and this program will read the sensor voltage.

#include <Talkie.h>

#include <TalkieUtils.h>
#include "ADCUtils.h" // for getVCCVoltage()

#define VERSION_EXAMPLE "1.2"

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
    Serial.begin(115200);
    while (!Serial)
        ; //delay for Leonardo
    // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // for LED
    pinMode(13, OUTPUT);

#if defined(CORE_TEENSY)
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH); //Enable Amplified PROP shield
#endif
    Serial.print("Voice queue size is: ");
    Serial.println(Voice.sayQ(spPAUSE1)); // this initializes the queue and the hardware

}

void loop() {

#if defined(__AVR__)
    float tVCCVoltage = getVCCVoltage();
    Serial.print(tVCCVoltage);
    Serial.println(" volt VCC");

    int voltage = analogRead(0) * tVCCVoltage / 1.023;
#else
    int voltage = analogRead(0) * 3.3 / 1.023;
#endif

    Serial.print(voltage);
    Serial.println(" mV input");

    sayQVoltageMilliVolts(&Voice, voltage);
    // Using .say() here is another way to block the sketch here and wait for end of speech as you can easily see in the source code of say().
    Voice.sayQ(spPAUSE1);
    while (Voice.isTalking()) {
        ;
    }
}
