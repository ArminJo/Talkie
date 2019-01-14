// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.

// GaryA 4/2018 Modified const variables to compile in Arduino 1.6 onwards
//
// Now for something a bit more challenging.
//
// Building sentences by program.
//
// The sayNumber() function can say any number under a million by
// building the number from short phrases,
//
// Connect a sensor to Analog 0, and this program will read the sensor voltage.
//
// Sound is output on digital pin 3 and/or 11. It can drive headphones directly, or add a simple audio amplifier to drive a loudspeaker.

#include <Arduino.h>

#include "Talkie.h"
#include "Vocab_US_Large.h"
#include "Vocab_Special.h"

Talkie voice;

/* Say any number between -999,999 and 999,999 */
void sayNumber(long n) {
  if (n<0) {
    voice.say(sp2_MINUS);
    sayNumber(-n);
  } else if (n==0) {
    voice.say(sp2_ZERO);
  } else {
    if (n>=1000) {
      int thousands = n / 1000;
      sayNumber(thousands);
      voice.say(sp2_THOUSAND);
      n %= 1000;
      if ((n > 0) && (n<100)) voice.say(sp2_AND);
    }
    if (n>=100) {
      int hundreds = n / 100;
      sayNumber(hundreds);
      voice.say(sp2_HUNDRED);
      n %= 100;
      if (n > 0) voice.say(sp2_AND);
    }
    if (n>19) {
      int tens = n / 10;
      switch (tens) {
        case 2: voice.say(sp2_TWENTY); break;
        case 3: voice.say(sp2_THIR_); voice.say(sp2_T); break;
        case 4: voice.say(sp2_FOUR); voice.say(sp2_T);  break;
        case 5: voice.say(sp2_FIF_);  voice.say(sp2_T); break;
        case 6: voice.say(sp2_SIX);  voice.say(sp2_T); break;
        case 7: voice.say(sp2_SEVEN);  voice.say(sp2_T); break;
        case 8: voice.say(sp2_EIGHT);  voice.say(sp2_T); break;
        case 9: voice.say(sp2_NINE);  voice.say(sp2_T); break;
      }
      n %= 10;
    }
    switch(n) {
      case 1: voice.say(sp2_ONE); break;
      case 2: voice.say(sp2_TWO); break;
      case 3: voice.say(sp2_THREE); break;
      case 4: voice.say(sp2_FOUR); break;
      case 5: voice.say(sp2_FIVE); break;
      case 6: voice.say(sp2_SIX); break;
      case 7: voice.say(sp2_SEVEN); break;
      case 8: voice.say(sp2_EIGHT); break;
      case 9: voice.say(sp2_NINE); break;
      case 10: voice.say(sp2_TEN); break;
      case 11: voice.say(sp2_ELEVEN); break;
      case 12: voice.say(sp2_TWELVE); break;
      case 13: voice.say(sp2_THIR_); voice.say(sp2__TEEN); break;
      case 14: voice.say(sp2_FOUR); voice.say(sp2__TEEN);break;
      case 15: voice.say(sp2_FIF_); voice.say(sp2__TEEN); break;
      case 16: voice.say(sp2_SIX); voice.say(sp2__TEEN); break;
      case 17: voice.say(sp2_SEVEN); voice.say(sp2__TEEN); break;
      case 18: voice.say(sp2_EIGHT); voice.say(sp2__TEEN); break;
      case 19: voice.say(sp2_NINE); voice.say(sp2__TEEN); break;
    }
  }
}
void setup() {
//    voice.doNotUseUseInvertedOutput();
#if defined(CORE_TEENSY)
    pinMode(5, OUTPUT);
    digitalWrite(5, HIGH); //Enable Amplified PROP shield
#endif
  Serial.begin(9600);
}

void loop() {
  int voltage = analogRead(0) * 5.000 / 1.023;
  Serial.println(voltage);
  sayNumber(voltage);
  voice.say(sp2_MILLI);
  voice.say(sp2_VOLTS);
  delay(2000);
}
