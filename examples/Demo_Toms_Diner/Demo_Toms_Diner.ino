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

#include "Talkie.h"
#include "Vocab_Toms_Diner.h"

Talkie voice;

void setup() {
  voice.say(spDINER);
}
void loop() {
}