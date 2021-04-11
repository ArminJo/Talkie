/*
 * TalkieLPC.h
 *
 * Values can be found on https://github.com/mamedev/mame/blob/master/src/devices/sound/tms5110r.hxx
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

#ifndef SRC_LIB_TALKIE_TALKIELPC_H_
#define SRC_LIB_TALKIE_TALKIELPC_H_

#include <inttypes.h>

// 353 bytes used for parameters
#if !defined(USE_10_BIT_KOEFFICIENT_VALUES) // from TI5220
// parameters based on TI2802 values
static const uint8_t tmsEnergy[0x10] PROGMEM = {0, 2, 3, 4, 5, 7, 10, 15, 20, 32, 41, 57, 81, 114, 161, 255};
static const uint8_t tmsPeriod[0x40] PROGMEM = {0, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
    36, 37, 38, 39, 40, 41, 42, 43, 45, 47, 49, 51, 53, 54, 57, 59, 61, 63, 66, 69, 71, 73, 77, 79, 81, 85, 87, 92, 95, 99, 102,
    106, 110, 115, 119, 123, 128, 133, 138, 143, 149, 154, 160};

#  ifdef FAST_8BIT_MODE // for K1 and K2
/*
 * Use 8bit coefficients K1 and K2.
 * Saves 10 microseconds (40 instead of 50 us) for a 16 MHz ATmega
 * has almost the same quality, except of a few "dropouts" e.g. in the word "thousand"
 */
static const int8_t tmsK1[0x20] PROGMEM = {-125, -124, -124, -124, -123, -123, -122, -120, -119, -118, -117, -116, -115, -113,
    -111, -109, -103, -95, -84, -72, -57, -39, -20, 0, 20, 39, 57, 72, 85, 95, 103, 109};
static const int8_t tmsK2[0x20] PROGMEM = {-82, -75, -68, -61, -52, -44, -34, -24, -14, -4, 6, 16, 26, 36, 45, 54, 62, 70, 77, 83,
    89, 94, 98, 102, 106, 109, 112, 114, 116, 118, 119, 127};
#  else
/*
 * 16 bit coefficients K1 and K2.
 */
static const int16_t tmsK1[0x20] PROGMEM = {-32064, -31872, -31808, -31680, -31552, -31424, -31232, -30848, -30592, -30336, -30016,
    -29696, -29376, -28928, -28480, -27968, -26368, -24256, -21632, -18368, -14528, -10048, -5184, 0, 5184, 10048, 14528, 18368,
    21632, 24256, 26368, 27968};
static const int16_t tmsK2[0x20] PROGMEM = {-20992, -19328, -17536, -15552, -13440, -11200, -8768, -6272, -3712, -1088, 1536, 4160,
    6720, 9216, 11584, 13824, 15936, 17856, 19648, 21248, 22656, 24000, 25152, 26176, 27072, 27840, 28544, 29120, 29632, 30080,
    30464, 32384};
#  endif

static const int8_t tmsK3[0x10] PROGMEM = {-110, -97, -83, -70, -56, -43, -29, -16, -2, 11, 25, 38, 52, 65, 79, 92};
static const int8_t tmsK4[0x10] PROGMEM = {-82, -68, -54, -40, -26, -12, 1, 15, 29, 43, 57, 71, 85, 99, 113, 126};
static const int8_t tmsK5[0x10] PROGMEM = {-82, -70, -59, -47, -35, -24, -12, -1, 11, 23, 34, 46, 57, 69, 81, 92};
static const int8_t tmsK6[0x10] PROGMEM = {-64, -53, -42, -31, -20, -9, 3, 14, 25, 36, 47, 58, 69, 80, 91, 102};
static const int8_t tmsK7[0x10] PROGMEM = {-77, -65, -53, -41, -29, -17, -5, 7, 19, 31, 43, 55, 67, 79, 90, 102};
static const int8_t tmsK8[0x08] PROGMEM = {-64, -40, -16, 7, 31, 55, 79, 102};
static const int8_t tmsK9[0x08] PROGMEM = {-64, -44, -24, -4, 16, 37, 57, 77};
static const int8_t tmsK10[0x08] PROGMEM = {-51, -33, -15, 4, 22, 32, 59, 77};

#else
/*
 * In my opinion, the quality here is no way better than for 8 bit values even with 10 bit DAC.
 * The only improvement is that tmsK1 and tmsK2 are also 10 bit now, but this is not of interest for implementation in software :-(.
 * Original values for energy, period and K1 to K10 from TI5220 with 10 bit resolution for K1 to K10 | TI_5110_5220_LPC
 */
static const uint8_t tmsEnergy[0x10] PROGMEM = {0, 1, 2, 3, 4, 6, 8, 11, 16, 23, 33, 47, 63, 85, 114, 0}; // TI_028X_LATER_ENERGY
static const uint8_t tmsPeriod[0x40] PROGMEM = {0, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37,
    38, 39, 40, 41, 42, 44, 46, 48, 50, 52, 53, 56, 58, 60, 62, 65, 68, 70, 72, 76, 78, 80, 84, 86,
    91, 94, 98, 101, 105, 109, 114, 118, 122, 127, 132, 137, 142, 148, 153, 159}; // TI_5220_PITCH

static const int16_t tmsK1[0x20] PROGMEM = {-501, -498, -497, -495, -493, -491, -488, -482, -478, -474, -469, -464, -459, -452, -445, -437, -412, -380, -339, -288, -227, -158, -81, -1, 80, 157, 226, 287, 337, 379, 411, 436};
static const int16_t tmsK2[0x20] PROGMEM = {-328, -303, -274, -244, -211, -175, -138, -99, -59, -18, 24, 64, 105, 143, 180, 215, 248, 278, 306, 331, 354, 374, 392, 408, 422, 435, 445, 455, 463, 470, 476, 506};

static const int16_t tmsK3[0x10] PROGMEM = {-441, -387, -333, -279, -225, -171, -117, -63, -9, 45, 98, 152, 206, 260, 314, 368};
static const int16_t tmsK4[0x10] PROGMEM = {-328, -273, -217, -161, -106, -50, 5, 61, 116, 172, 228, 283, 339, 394, 450, 506};
static const int16_t tmsK5[0x10] PROGMEM = {-328, -282, -235, -189, -142, -96, -50, -3, 43, 90, 136, 182, 229, 275, 322, 368};
static const int16_t tmsK6[0x10] PROGMEM = {-256, -212, -168, -123, -79, -35, 10, 54, 98, 143, 187, 232, 276, 320, 365, 409};
static const int16_t tmsK7[0x10] PROGMEM = {-308, -260, -212, -164, -117, -69, -21, 27, 75, 122, 170, 218, 266, 314, 361, 409};
static const int16_t tmsK8[0x08] PROGMEM = {-256, -161, -66, 29, 124, 219, 314, 409};
static const int16_t tmsK9[0x08] PROGMEM = {-256, -176, -96, -15, 65, 146, 226, 307};
static const int16_t tmsK10[0x08] PROGMEM = {-205, -132, -59, 14, 87, 160, 234, 307};
#endif

// The patented one
//static uint8_t chirp[] = { 0x00, 0x2a, 0xd4, 0x32, 0xb2, 0x12, 0x25, 0x14, 0x02, 0xe1, 0xc5, 0x02, 0x5f, 0x5a, 0x05, 0x0f, 0x26,
//        0xfc, 0xa5, 0xa5, 0xd6, 0xdd, 0xdc, 0xfc, 0x25, 0x2b, 0x22, 0x21, 0x0f, 0xff, 0xf8, 0xee, 0xed, 0xef, 0xf7, 0xf6, 0xfa,
//        0x00, 0x03, 0x02, 0x01 };
// The one used in TI chips > TI2802
static int8_t chirp[] = { 0x00, 0x03, 0x0f, 0x28, 0x4c, 0x6c, 0x71, 0x50, 0x25, 0x26, 0x4c, 0x44, 0x1a, 0x32, 0x3b, 0x13, 0x37,
        0x1a, 0x25, 0x1f, 0x1d };

/*
 * Interpolation shift coefficients | TI_INTERP
 * For future use
 */
#if !defined(__AVR__)
//static int8_t interploation_shift[] = { 0, 3, 3, 3, 2, 2, 1, 1 };
#endif

#endif /* SRC_LIB_TALKIE_TALKIELPC_H_ */
