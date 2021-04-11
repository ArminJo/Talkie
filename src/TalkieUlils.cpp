/*
 * TalkieUtils.cpp
 * Based on the Talkie library. https://github.com/going-digital/Talkie.
 * Copyright 2011 Peter Knight
 *
 *  SUMMARY
 *  Talkie is a speech library for Arduino.
 *  GaryA 10/2018 added the original sayNumber() function
 *
 *  Copyright (C) 2018  Armin Joachimsmeyer
 *  armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Talkie https://github.com/ArminJo/Talkie.
 *
 *  Talkie is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/gpl.html>.
 *
 */

#include <Arduino.h>

#include "Vocab_US_Large.h"
#include "Vocab_Special.h"

#include "TalkieUtils.h"

#if defined(__arm__)
#  if ! defined(TEENSYDUINO)
#  include <avr/dtostrf.h>
#  endif
#endif

int8_t sayQDigit(Talkie * aVoice, char aDigit) {
    return sayQNumber(aVoice, aDigit - '0');
}

int8_t sayQVoltageMilliVolts(Talkie * aVoice, long aMilliVolt) {
    sayQNumber(aVoice, aMilliVolt);
    aVoice->sayQ(sp2_MILLI);
    return aVoice->sayQ(sp2_VOLTS);
}

int8_t sayQVoltageVolts(Talkie * aVoice, float aVolt) {
    sayQFloat(aVoice, aVolt, 2, true, true);
    return aVoice->sayQ(sp2_VOLTS);
}

int8_t sayQPause(Talkie * aVoice) {
    return aVoice->sayQ(spPAUSE1);
}

int8_t sayQTimeout(Talkie * aVoice) {
    aVoice->sayQ(sp2_TIME);
    return aVoice->sayQ(sp2_OUT);
}

/* sayQ any number between -999,999 and 999,999 */
int8_t sayQNumber(Talkie * aVoice, long aNumber) {
    if (aNumber < 0) {
        aVoice->sayQ(sp2_MINUS);
        sayQNumber(aVoice, -aNumber);
    } else if (aNumber == 0) {
        aVoice->sayQ(sp2_ZERO);
    } else {
        if (aNumber >= 1000) {
            int thousands = aNumber / 1000;
            sayQNumber(aVoice, thousands);
            aVoice->sayQ(sp2_THOUSAND);
            aNumber %= 1000;
            if ((aNumber > 0) && (aNumber < 100))
                aVoice->sayQ(sp2_AND);
        }
        if (aNumber >= 100) {
            int hundreds = aNumber / 100;
            sayQNumber(aVoice, hundreds);
            aVoice->sayQ(sp2_HUNDRED);
            aNumber %= 100;
            if (aNumber > 0)
                aVoice->sayQ(sp2_AND);
        }
        if (aNumber > 19) {
            int tens = aNumber / 10;
            switch (tens) {
            case 2:
                aVoice->sayQ(sp2_TWENTY);
                break;
            case 3:
                aVoice->sayQ(sp2_THIR_);
                aVoice->sayQ(sp2_T);
                break;
            case 4:
                aVoice->sayQ(sp2_FOUR);
                aVoice->sayQ(sp2_T);
                break;
            case 5:
                aVoice->sayQ(sp2_FIF_);
                aVoice->sayQ(sp2_T);
                break;
            case 6:
                aVoice->sayQ(sp2_SIX);
                aVoice->sayQ(sp2_T);
                break;
            case 7:
                aVoice->sayQ(sp2_SEVEN);
                aVoice->sayQ(sp2_T);
                break;
            case 8:
                aVoice->sayQ(sp2_EIGHT);
                aVoice->sayQ(sp2_T);
                break;
            case 9:
                aVoice->sayQ(sp2_NINE);
                aVoice->sayQ(sp2_T);
                break;
            }
            aNumber %= 10;
        }
        switch (aNumber) {
        case 1:
            aVoice->sayQ(sp2_ONE);
            break;
        case 2:
            aVoice->sayQ(sp2_TWO);
            break;
        case 3:
            aVoice->sayQ(sp2_THREE);
            break;
        case 4:
            aVoice->sayQ(sp2_FOUR);
            break;
        case 5:
            aVoice->sayQ(sp2_FIVE);
            break;
        case 6:
            aVoice->sayQ(sp2_SIX);
            break;
        case 7:
            aVoice->sayQ(sp2_SEVEN);
            break;
        case 8:
            aVoice->sayQ(sp2_EIGHT);
            break;
        case 9:
            aVoice->sayQ(sp2_NINE);
            break;
        case 10:
            aVoice->sayQ(sp2_TEN);
            break;
        case 11:
            aVoice->sayQ(sp2_ELEVEN);
            break;
        case 12:
            aVoice->sayQ(sp2_TWELVE);
            break;
        case 13:
            aVoice->sayQ(sp2_THIR_);
            aVoice->sayQ(sp2__TEEN);
            break;
        case 14:
            aVoice->sayQ(sp2_FOUR);
            aVoice->sayQ(sp2__TEEN);
            break;
        case 15:
            aVoice->sayQ(sp2_FIF_);
            aVoice->sayQ(sp2__TEEN);
            break;
        case 16:
            aVoice->sayQ(sp2_SIX);
            aVoice->sayQ(sp2__TEEN);
            break;
        case 17:
            aVoice->sayQ(sp2_SEVEN);
            aVoice->sayQ(sp2__TEEN);
            break;
        case 18:
            aVoice->sayQ(sp2_EIGHT);
            aVoice->sayQ(sp2__TEEN);
            break;
        case 19:
            aVoice->sayQ(sp2_NINE);
            aVoice->sayQ(sp2__TEEN);
            break;
        }
    }
    return (aVoice->free);
}

#define LENGT_OF_FLOAT_STRING 14
int8_t sayQFloat(Talkie * aVoice, float aFloat, int aDecimalPlaces, bool aSuppressLeadingZero, bool aSuppressTrailingZero) {
    // First the integer part
    long tIntegerPart = aFloat;
    if (tIntegerPart != 0 || !aSuppressLeadingZero) {
        sayQNumber(aVoice, tIntegerPart);
    }
    if (aDecimalPlaces > 0) {
        // convert to string, this avoids rounding errors like 0.654 * 10 = 6,5399
        char tFloatString[LENGT_OF_FLOAT_STRING];
        dtostrf(aFloat, 8, aDecimalPlaces, tFloatString);
        int i;
        // find decimal point in string
        for (i = 0; i < (LENGT_OF_FLOAT_STRING - 1); ++i) {
            if (tFloatString[i] == '.') {
                i++;
                break;
            }
        }
        // output decimal places digits if available
        if (i < LENGT_OF_FLOAT_STRING - 2) {
            aVoice->sayQ(sp2_POINT);
            for (int j = 0; j < aDecimalPlaces; ++j) {
                // suppress zero at last position
                if (!(tFloatString[i] == '0' && aSuppressTrailingZero && j == aDecimalPlaces - 1)) {
                    sayQNumber(aVoice, tFloatString[i] - '0');
                    // check for end of string
                    i++;
                    if (tFloatString[i] == '\0') {
                        break;
                    }
                }
            }
        }
    }
    return (aVoice->free);
}
