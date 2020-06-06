// Talkie library
// Copyright 2011 Peter Knight
// This code is released under GPLv2 license.
//
// Armin Joachimsmeyer 11/2018 converted to .c and .h files
//
// The following phrases are derived from those built into the
// Acorn Computers Speech Synthesiser add-on from 1983.
//
// A male voice with an RP English accent, voiced by Kenneth Kendall.
//

#ifndef TALKIE_VOCAB_US_ACORN_H_
#define TALKIE_VOCAB_US_ACORN_H_

#include <Arduino.h>

// moved to Vocab_Special
//extern const uint8_t spa_PAUSE1[]    PROGMEM;
//extern const uint8_t spa_PAUSE2[]    PROGMEM;

extern const uint8_t spa_TONE1[]     PROGMEM;
extern const uint8_t spa_TONE2[]     PROGMEM;
extern const uint8_t spa__D[]        PROGMEM;
extern const uint8_t spa__ED[]       PROGMEM;
extern const uint8_t spa__ING[]      PROGMEM;
extern const uint8_t spa__S[]        PROGMEM;
extern const uint8_t spa__TEEN[]     PROGMEM;
extern const uint8_t spa__TH[]       PROGMEM;
extern const uint8_t spa__T[]        PROGMEM;
extern const uint8_t spa__Z[]         PROGMEM;
extern const uint8_t spa_ZERO[]      PROGMEM;
extern const uint8_t spa_HUNDRED[]   PROGMEM;
extern const uint8_t spa_THOUSAND[]  PROGMEM;
extern const uint8_t spa_ONE[]       PROGMEM;
extern const uint8_t spa_TWO[]       PROGMEM;
extern const uint8_t spa_TWEN_[]     PROGMEM;
extern const uint8_t spa_THREE[]     PROGMEM;
extern const uint8_t spa_THIR_[]     PROGMEM;
extern const uint8_t spa_FOUR[]      PROGMEM;
extern const uint8_t spa_FOUR_[]     PROGMEM;
extern const uint8_t spa_FIVE[]      PROGMEM;
extern const uint8_t spa_FIF_[]      PROGMEM;
extern const uint8_t spa_SIX[]       PROGMEM;
extern const uint8_t spa_SIX_[]      PROGMEM;
extern const uint8_t spa_SEVEN[]     PROGMEM;
extern const uint8_t spa_SEVEN_[]    PROGMEM;
extern const uint8_t spa_EIGHT[]     PROGMEM;
extern const uint8_t spa_EIGH_[]     PROGMEM;
extern const uint8_t spa_NINE[]      PROGMEM;
extern const uint8_t spa_NINE_[]     PROGMEM;
extern const uint8_t spa_A[]         PROGMEM;
extern const uint8_t spa_ACORN[]     PROGMEM;
extern const uint8_t spa_AFTER[]     PROGMEM;
extern const uint8_t spa_AGAIN[]     PROGMEM;
extern const uint8_t spa_AMOUNT[]    PROGMEM;
extern const uint8_t spa_AN[]        PROGMEM;
extern const uint8_t spa_AND[]       PROGMEM;
extern const uint8_t spa_ANOTHER[]   PROGMEM;
extern const uint8_t spa_ANSWER[]    PROGMEM;
extern const uint8_t spa_ANY[]       PROGMEM;
extern const uint8_t spa_AVAILABLE[] PROGMEM;
extern const uint8_t spa_B[]         PROGMEM;
extern const uint8_t spa_BAD[]       PROGMEM;
extern const uint8_t spa_BETWEEN[]   PROGMEM;
extern const uint8_t spa_BOTH[]      PROGMEM;
extern const uint8_t spa_BUTTON[]    PROGMEM;
extern const uint8_t spa_C[]         PROGMEM;
extern const uint8_t spa_CASSETTE[]  PROGMEM;
extern const uint8_t spa_CHARACTER[] PROGMEM;
extern const uint8_t spa_COMPLETE[]  PROGMEM;
extern const uint8_t spa_COMPUTER[]  PROGMEM;
extern const uint8_t spa_CORRECT[]   PROGMEM;
extern const uint8_t spa_D[]         PROGMEM;
extern const uint8_t spa_DATA[]      PROGMEM;
extern const uint8_t spa_DATE[]      PROGMEM;
extern const uint8_t spa_DO[]        PROGMEM;
extern const uint8_t spa_DOLLAR[]    PROGMEM;
extern const uint8_t spa_DONT[]      PROGMEM;
extern const uint8_t spa_DOWN[]      PROGMEM;
extern const uint8_t spa_E[]         PROGMEM;
extern const uint8_t spa_EACH[]      PROGMEM;
extern const uint8_t spa_ELEVEN[]    PROGMEM;
extern const uint8_t spa_ENGAGED[]   PROGMEM;
extern const uint8_t spa_ENTER[]     PROGMEM;
extern const uint8_t spa_ERROR[]     PROGMEM;
extern const uint8_t spa_ESCAPE[]    PROGMEM;
extern const uint8_t spa_F[]         PROGMEM;
extern const uint8_t spa_FEW[]       PROGMEM;
extern const uint8_t spa_FILE[]      PROGMEM;
extern const uint8_t spa_FIRST[]     PROGMEM;
extern const uint8_t spa_FOUND[]     PROGMEM;
extern const uint8_t spa_FROM[]      PROGMEM;
extern const uint8_t spa_G[]         PROGMEM;
extern const uint8_t spa_GOOD[]      PROGMEM;
extern const uint8_t spa_H[]         PROGMEM;
extern const uint8_t spa_HAVE[]      PROGMEM;
extern const uint8_t spa_I[]         PROGMEM;
extern const uint8_t spa_ILLEGAL[]   PROGMEM;
extern const uint8_t spa_IN_[]       PROGMEM;
extern const uint8_t spa_INPUT[]     PROGMEM;
extern const uint8_t spa_IS[]        PROGMEM;
extern const uint8_t spa_J[]         PROGMEM;
extern const uint8_t spa_K[]         PROGMEM;
extern const uint8_t spa_KEY[]       PROGMEM;
extern const uint8_t spa_L[]         PROGMEM;
extern const uint8_t spa_LARGE[]     PROGMEM;
extern const uint8_t spa_LAST[]      PROGMEM;
extern const uint8_t spa_LINE[]      PROGMEM;
extern const uint8_t spa_M[]         PROGMEM;
extern const uint8_t spa_MANY[]      PROGMEM;
extern const uint8_t spa_MINUS[]     PROGMEM;
extern const uint8_t spa_MORE[]      PROGMEM;
extern const uint8_t spa_MUST[]      PROGMEM;
extern const uint8_t spa_N[]         PROGMEM;
extern const uint8_t spa_NAME[]      PROGMEM;
extern const uint8_t spa_NEGATIVE[]  PROGMEM;
extern const uint8_t spa_NEW[]       PROGMEM;
extern const uint8_t spa_NO[]        PROGMEM;
extern const uint8_t spa_NOT[]       PROGMEM;
extern const uint8_t spa_NOW[]       PROGMEM;
extern const uint8_t spa_NUMBER[]    PROGMEM;
extern const uint8_t spa_O[]         PROGMEM;
extern const uint8_t spa_OCLOCK[]    PROGMEM;
extern const uint8_t spa_OF[]        PROGMEM;
extern const uint8_t spa_OFF[]       PROGMEM;
extern const uint8_t spa_OLD[]       PROGMEM;
extern const uint8_t spa_ON[]        PROGMEM;
extern const uint8_t spa_ONLY[]      PROGMEM;
extern const uint8_t spa_OR[]        PROGMEM;
extern const uint8_t spa_P[]         PROGMEM;
extern const uint8_t spa_PARAMETER[] PROGMEM;
extern const uint8_t spa_PENCE[]     PROGMEM;
extern const uint8_t spa_PLEASE[]    PROGMEM;
extern const uint8_t spa_PLUS[]      PROGMEM;
extern const uint8_t spa_POINT[]     PROGMEM;
extern const uint8_t spa_POSITIVE[]  PROGMEM;
extern const uint8_t spa_POUN_[]     PROGMEM;
extern const uint8_t spa_PRESS[]     PROGMEM;
extern const uint8_t spa_PROGRAMME[] PROGMEM;
extern const uint8_t spa_Q[]         PROGMEM;
extern const uint8_t spa_R[]         PROGMEM;
extern const uint8_t spa_RED[]       PROGMEM;
extern const uint8_t spa_RESET[]     PROGMEM;
extern const uint8_t spa_RETURN[]    PROGMEM;
extern const uint8_t spa_RUN[]       PROGMEM;
extern const uint8_t spa_RUNNING[]   PROGMEM;
extern const uint8_t spa_S[]         PROGMEM;
extern const uint8_t spa_SAME[]      PROGMEM;
extern const uint8_t spa_SCORE[]     PROGMEM;
extern const uint8_t spa_SECOND[]    PROGMEM;
extern const uint8_t spa_SMALL[]     PROGMEM;
extern const uint8_t spa_START[]     PROGMEM;
extern const uint8_t spa_STOP[]      PROGMEM;
extern const uint8_t spa_SWITCH[]    PROGMEM;
extern const uint8_t spa_T[]         PROGMEM;
extern const uint8_t spa_TEN[]       PROGMEM;
extern const uint8_t spa_THANK[]     PROGMEM;
extern const uint8_t spa_THAT[]      PROGMEM;
extern const uint8_t spa_THE[]       PROGMEM;
extern const uint8_t spa_THEN[]      PROGMEM;
extern const uint8_t spa_THIRD[]     PROGMEM;
extern const uint8_t spa_THIS[]      PROGMEM;
extern const uint8_t spa_TIME[]      PROGMEM;
extern const uint8_t spa_TRY[]       PROGMEM;
extern const uint8_t spa_TWELVE[]    PROGMEM;
extern const uint8_t spa_TYPE[]      PROGMEM;
extern const uint8_t spa_U[]         PROGMEM;
extern const uint8_t spa_UH[]        PROGMEM;
extern const uint8_t spa_UP[]        PROGMEM;
extern const uint8_t spa_V[]         PROGMEM;
extern const uint8_t spa_VERY[]      PROGMEM;
extern const uint8_t spa_W[]         PROGMEM;
extern const uint8_t spa_WANT[]      PROGMEM;
extern const uint8_t spa_WAS[]       PROGMEM;
extern const uint8_t spa_WERE[]      PROGMEM;
extern const uint8_t spa_WHAT[]      PROGMEM;
extern const uint8_t spa_WHICH[]     PROGMEM;
extern const uint8_t spa_X[]         PROGMEM;
extern const uint8_t spa_Y[]         PROGMEM;
extern const uint8_t spa_YEAR[]      PROGMEM;
extern const uint8_t spa_YES[]       PROGMEM;
extern const uint8_t spa_YOUR[]      PROGMEM;
extern const uint8_t spa_Z[]         PROGMEM;
#endif /* TALKIE_VOCAB_US_ACORN_H_ */

