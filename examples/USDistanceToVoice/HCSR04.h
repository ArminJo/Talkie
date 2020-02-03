/*
 * HCSR04.h
 *
 *  Copyright (C) 2018-2020  Armin Joachimsmeyer
 *  Email: armin.joachimsmeyer@gmail.com
 *
 *  This file is part of Arduino-Utils https://github.com/ArminJo/Arduino-Utils.
 *
 *  Arduino-Utils is free software: you can redistribute it and/or modify
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

#include <stdint.h>

#ifndef HCSR04_H_
#define HCSR04_H_

#define US_DISTANCE_DEFAULT_TIMEOUT_MICROS 20000
#define US_DISTANCE_TIMEOUT_MICROS_FOR_1_METER 5825  // Timeout of 5825 is 1 meter
#define US_DISTANCE_TIMEOUT_MICROS_FOR_2_METER 11650 // Timeout of 11650 is 2 meter
#define US_DISTANCE_TIMEOUT_MICROS_FOR_3_METER 17475 // Timeout of 17475 is 3 meter
#define US_DISTANCE_DEFAULT_TIMEOUT_CENTIMETER 343   // Timeout of 20000L is 3.43 meter

void initUSDistancePins(uint8_t aTriggerOutPin, uint8_t aEchoInPin);
unsigned int getUSDistance(unsigned int aTimeoutMicros = US_DISTANCE_DEFAULT_TIMEOUT_MICROS);
unsigned int getCentimeterFromUSMicroSeconds(unsigned int aDistanceMicros);
unsigned int getUSDistanceAsCentiMeter(unsigned int aTimeoutMicros = US_DISTANCE_DEFAULT_TIMEOUT_MICROS);
unsigned int getUSDistanceAsCentiMeterWithCentimeterTimeout(unsigned int aTimeoutCentimeter);
void testUSSensor(uint16_t aSecondsToTest);

#if (defined(USE_PIN_CHANGE_INTERRUPT_D0_TO_D7) | defined(USE_PIN_CHANGE_INTERRUPT_D8_TO_D13) | defined(USE_PIN_CHANGE_INTERRUPT_A0_TO_A5))
/*
 * Non blocking version
 */
void startUSDistanceAsCentiMeterWithCentimeterTimeoutNonBlocking(unsigned int aTimeoutCentimeter);
bool isUSDistanceMeasureFinished();
extern unsigned int sUSDistanceCentimeter;
extern volatile unsigned long sUSPulseMicros;
#endif

#endif // HCSR04_H_

#pragma once
