/*
 * TalkieUtils.h
 * Based on the Talkie library. https://github.com/going-digital/Talkie.
 * Copyright 2011 Peter Knight
 *
 *  SUMMARY
 *  Talkie is a speech library for Arduino.
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

#ifndef TALKIEUTILS_H_
#define TALKIEUTILS_H_

#include "Talkie.h"

int8_t sayQNumber(Talkie * voice, long n);
int8_t sayQFloat(Talkie * voice, float aFloat, int aDecimalPlaces, bool aSuppressLeadingZero = true, bool aSuppressTrailingZero =
        false);
int8_t sayQVoltageMilliVolts(Talkie * aVoice, long aMilliVolt);
int8_t sayQVoltageVolts(Talkie * aVoice, float aVolt);
int8_t sayQPause(Talkie * aVoice);
int8_t sayQTimeout(Talkie * aVoice);

#endif /* TALKIEUTILS_H_ */
