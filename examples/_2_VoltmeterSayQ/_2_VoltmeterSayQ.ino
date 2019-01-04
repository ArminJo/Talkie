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

#if defined(__AVR_ATmega32U4__)
#define Serial Serial1
#endif

#define VERSION_EXAMPLE "1.0"

Talkie Voice;
uint16_t readADCChannel(uint8_t aChannelNumber);
float getVCCVoltage(void);

void setup() {
    Serial.begin(115200);
    Serial.println(F("START " __FILE__ "\r\nVersion " VERSION_EXAMPLE " from " __DATE__));

    // for LED
    pinMode(13, OUTPUT);

//    Voice.doNotUseUseInvertedOutput();
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH); //Enable Amplified PROP shield

    Serial.print("Voice queue size is: ");
    Serial.println(Voice.sayQ(spPAUSE1)); // this initializes the queue and the hardware

}

void loop() {

    float tVCCVoltage = getVCCVoltage();
    Serial.print(tVCCVoltage);
    Serial.println(" Volt VCC");

    int voltage = analogRead(0) * tVCCVoltage / 1.023;

    Serial.print(voltage);
    Serial.println(" mV input");

    sayQVoltageMilliVolts(&Voice, voltage);
    // Using .say() here is another way to block the sketch here and wait for end of speech as you can easily see in the source code of say().
    Voice.sayQ(spPAUSE1);
    while (Voice.isTalking()) {
        ;
    }
}

#define ADC_PRESCALE64   6 // 52 microseconds per ADC conversion at 16 MHz
uint16_t readADCChannel(uint8_t aChannelNumber) {
    ADMUX = aChannelNumber | (DEFAULT << REFS0); // DEFAULT = VCC

//  ADCSRB = 0; // free running mode  - is default
    // ADSC-StartConversion ADIF-Reset Interrupt Flag - NOT free running mode
    ADCSRA = ((1 << ADEN) | (1 << ADSC) | (1 << ADIF) | ADC_PRESCALE64);

// wait for single conversion to finish
    loop_until_bit_is_clear(ADCSRA, ADSC);

    // combine the two bytes
    return ADCL | (ADCH << 8);
}

#if defined(__AVR_ATmega32U4__)
#define ADC_1_1_VOLT_CHANNEL_MUX 0x1E
#else
#define ADC_1_1_VOLT_CHANNEL_MUX 0x0E
#endif
// computes VCC voltage using internal 1.1 reference
float getVCCVoltage(void) {
    // use VCC with external capacitor at AREF pin as reference
    readADCChannel(ADC_1_1_VOLT_CHANNEL_MUX); // to switch the channel
    delayMicroseconds(400); // wait for the value to settle. value must be > 100 but to get the last bits value must be greater than 400
    uint16_t tVCC = readADCChannel(ADC_1_1_VOLT_CHANNEL_MUX);
    return ((1024 * 1.1) / tVCC);
}
