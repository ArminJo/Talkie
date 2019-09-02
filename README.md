# Talkie

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Build Status](https://travis-ci.org/ArminJo/Talkie.svg?branch=master)](https://travis-ci.org/ArminJo/Talkie)

## Speech library for Arduino
The original version can be found [here](https://github.com/going-digital/Talkie)
A good explanation of the TMS5220 operation and the LPC frame format can be found [here](https://github.com/mamedev/mame/blob/master/src/devices/sound/tms5220.txt)

Youtube Demonstration of Talkie Voltmeter example

[![Demonstration of Talkie Voltmeter example](https://img.youtube.com/vi/6jXkugZTwCs/0.jpg)](https://www.youtube.com/watch?v=6jXkugZTwCs)

## Improvements to the original and to the non blocking version of PaulStoffregen
- Improved code so Talkie now runs on **8MHz** Arduino (with millis() interrupt disabled while talking).
- Fixed the ISR_RATIO Bug for plain Arduino.
- Added utility functions sayQNumber(), sayQFloat(), sayQVoltageMilliVolts() extracted from the examples.
- Inverted output at pin 11 is enabled by default to increase volume for direct attached piezo or speaker.
- Added comments and did refactoring to better understand the functionality.
- Added compatibility to Arduino Tone library by stopping timer1 interrupts at every end of speech.
- Extracted initializeHardware() and terminateHardware() functions for easy adapting to other platforms.
- Currently supporting:
  - **ATmega328** as found on the **Uno** and **Nano** bords.
  - **ATmega32U4** as found on the **Leonardo** and **CircuitPlaygound** boards.
  - **ARM0** (but not tested) as found on the **SAMD**, **Teensy** and **Particle** boards.

## Hints
- Connect the speaker to digital pin 3 and 11 of Arduino. They are enabled as non inverted and inverted outputs by default to increase volume for direct attached piezo or speaker. 
- As speaker I use the speakers from old earphones or headphones, which have ca. 32 Ohm, directly without a series resistor. The headphone speaker tend to be much louder, especially when they stay in their original housings.
- The Library uses Timer 1 and Timer 2, so libraries like Tone, Servo, analogWrite(), and some other libraries cannot be used while speaking.
- After a call to say... you can use tone() again.
- To use Servo write() after a call to say... you must detach() and attach() the servo before first write() in order to initialize the timer again for Servo.
- Porting to ATtinys is not possible, since they lack the hardware multiplication. ( Believe me, I tried it! )

## Own vocabulary
To create LPC data you can use [Qboxpro](http://ftp.whtech.com/pc%20utilities/qboxpro.zip), an unsupported old Windows application running under XP, which can produce Talkie compatible data streams. The missing BWCC.DLL (Borland Windows Custom Control Library) can be found e.g. [here](http://www.download-dll.com/dll-BWCC.dll.html).
The process is described [here](http://furrtek.free.fr/index.php?a=speakandspell&ss=9&i=2) and goes like this:
 - Create a new project using the following project parameters : Byte / 8 Khz / 5220 coding table
 - Goto Project and add the audio file
 - Choose process using : medium bit rate and pressing OK
 - Edit concatenation : insert concatenation after by adding a name; then insert phrase and press ok
 - Format it by choosing the first line in the format menu : LPC 10V, 4UV

Another way to generate the LPC data is to use the pyton script at https://github.com/ptwz/python_wizard

### Schematic for voltmeter example
![Fritzing schematic for voltmeter example](https://github.com/ArminJo/Talkie/blob/master/extras/TalkieVoltmeter_Steckplatine.png)

# Modifying library properties
To access the Arduino library files from a sketch, you have to first use `Sketch/Show Sketch Folder (Ctrl+K)` in the Arduino IDE.<br/>
Then navigate to the parallel `libraries` folder and select the library you want to access.<br/>
The library files itself are located in the `src` sub-directory.<br/>
If you did not yet store the example as your own sketch, then with Ctrl+K you are instantly in the right library folder.

# Revision History
### Version 1.0.0
Initial Arduino library version

## Travis CI
The Talkie library examples are built on Travis CI for the following boards:

- Arduino Uno
- Arduino Leonardo
- Arduino Mega 2560
- Arduino cplayClassic

## Requests for modifications / extensions
Please write me a PM including your motivation/problem if you need a modification or an extension.

#### If you find this library useful, please give it a star.
