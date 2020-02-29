# [Talkie](https://github.com/ArminJo/Talkie)
### [Version 1.0.3](https://github.com/ArminJo/Talkie/releases)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Installation instructions](https://www.ardu-badge.com/badge/Talkie.svg?)](https://www.ardu-badge.com/Talkie)
[![Commits since latest](https://img.shields.io/github/commits-since/ArminJo/Talkie/latest)](https://github.com/ArminJo/Talkie/commits/master)
[![Build Status](https://github.com/ArminJo/Talkie/workflows/LibraryBuild/badge.svg)](https://github.com/ArminJo/Talkie/actions)
[![Hit Counter](https://hitcounter.pythonanywhere.com/count/tag.svg?url=https%3A%2F%2Fgithub.com%2FArminJo%2FTalkie)](https://github.com/brentvollebregt/hit-counter)

## Speech library for Arduino
The original version can be found [here](https://github.com/going-digital/Talkie).<br/>
A good explanation of the TMS5220 operation and the LPC frame format can be found [here](https://github.com/mamedev/mame/blob/master/src/devices/sound/tms5220.txt).

YouTube Demonstration of Talkie voltmeter example

[![Demonstration of Talkie voltmeter example](https://img.youtube.com/vi/6jXkugZTwCs/0.jpg)](https://www.youtube.com/watch?v=6jXkugZTwCs)

## Improvements to the original and to the non blocking version of PaulStoffregen
- Improved code so Talkie now runs on **8 MHz** Arduino (with millis() interrupt disabled while talking).
- Fixed the ISR_RATIO Bug for plain Arduino.
- Added utility functions sayQNumber(), sayQFloat(), sayQVoltageMilliVolts() extracted from the examples.
- Inverted output at pin 11 is enabled by default to increase volume for direct attached piezo or speaker.
- Added comments and did refactoring to better understand the functionality.
- Added compatibility to Arduino Tone library by stopping timer1 interrupts at every end of speech.
- Extracted initializeHardware() and terminateHardware() functions for easy adapting to other platforms.
- Currently supporting:
  - **ATmega328** as found on the **Uno** and **Nano** boards.
  - **ATmega2560** as found on the **MEGA 2560** board.
  - **ATmega32U4** as found on the **Leonardo** and **CircuitPlaygound** boards.
  - **ARM0** (but not tested) as found on the **SAMD**, **Teensy** and **Particle** boards.
  
## Output pins for different ATmegas
 - ATmega328 (Uno and Nano): non inverted at pin 3, inverted at pin 11.
 - ATmega2560: non inverted at pin 6, inverted at pin 7.
 - ATmega32U4 (Leonardo): non inverted at pin 10, inverted at pin 9.
 - ATmega32U4 (CircuitPlaygound): only non inverted at pin 5.

## Timer usage
**Timer 1** (Servo timer) is used at all ATmegas for updating voice output data at 8 kHz.
**Timer 2** (Tone timer) on ATmega328 and **Timer 4** on ATmega2560 + ATmega32U4 is used to generate the 8 bit PWM output.

## Hints
- As **default** both inverted and not inverted outputs are enabled to **increase volume** if speaker is attached between them.
- I use the speakers from old earphones or headphones, which have approximately 16 to 32 Ohm, directly without a series resistor. The headphone speaker tend to be much louder, especially when they stay in their original housings.
- The library uses Timer 1 and Timer 2 on ATmega328, so libraries like Tone, Servo, analogWrite(), and some other libraries cannot be used while speaking.
- After a call to `say...()` you can use `tone()` again.
- To use Servo `write()` after a call to say... you must `detach()` and `attach()` the servo before first `write()` in order to initialize the timer again for Servo.
- If you want to use **SPI** functions on ATmega328 **while Talkie is speaking**, then disable Talkies usage of pin 11 by `Talkie Voice(true, false);` instead of `Talkie Voice;` **or** `Voice.doNotUseUseInvertedOutput();`.
- Porting to ATtinys is not possible, since they lack the hardware multiplication. ( Believe me, I tried it! )

## Own vocabulary
To create LPC data you can use the [python_wizard](https://github.com/ptwz/python_wizard) or [BlueWizard](https://github.com/patrick99e99/BlueWizard) for Mac OS X.

Another way to create LPC data is to use [Qboxpro](http://ftp.whtech.com/pc%20utilities/qboxpro.zip), an unsupported old Windows application running under XP, which can produce Talkie compatible data streams. The missing BWCC.DLL (Borland Windows Custom Control Library) can be found e.g. [here](http://www.download-dll.com/dll-BWCC.dll.html).
The process is described [here](http://furrtek.free.fr/index.php?a=speakandspell&ss=9&i=2) and goes like this:
 - Create a new project using the following project parameters : Byte / 8 KHz / 5220 coding table
 - Goto Project and add the audio file
 - Choose process using : medium bit rate and pressing OK
 - Edit concatenation : insert concatenation after by adding a name; then insert phrase and press OK
 - Format it by choosing the first line in the format menu : LPC 10V, 4UV

## Schematic for voltmeter example
![Fritzing schematic for voltmeter example](https://github.com/ArminJo/Talkie/blob/master/extras/TalkieVoltmeter_Steckplatine.png)

# Modifying library properties
To access the Arduino library files from a sketch, you have to first use *Sketch/Show Sketch Folder (Ctrl+K)* in the Arduino IDE.<br/>
Then navigate to the parallel `libraries` folder and select the library you want to access.<br/>
The library files itself are located in the `src` sub-directory.<br/>
If you did not yet store the example as your own sketch, then with *Ctrl+K* you are instantly in the right library folder.
## Consider to use [Sloeber](http://eclipse.baeyens.it/stable.php?OS=Windows) as IDE
If you are using Sloeber as your IDE, you can easily define global symbols at *Properties/Arduino/CompileOptions*.<br/>
![Sloeber settings](https://github.com/ArminJo/ServoEasing/blob/master/pictures/SloeberDefineSymbols.png)

# Revision History
### Version 1.0.3
- Added function `sayQTimeout()` in *TalkieUtils.cpp*.
- Added example *USDistanceToVoice*.
- Added function `sayQVoltageVolts()`.
### Version 1.0.2
- ATmega2560 supported and tested
- Always set pins to input when finishing, to avoid a click.
### Version 1.0.1
- Added SPI compatibility by not resetting pin 11 to input if SPI is detected
- Added new constructor `Talkie(bool aUseNonInvertedOutputPin, bool aUseInvertedOutputPin);`
### Version 1.0.0
- Initial Arduino library version

# CI
Since Travis CI is unreliable and slow, the library examples are now tested with GitHub Actions for the following boards:

- arduino:avr:uno
- arduino:avr:leonardo
- arduino:avr:mega
- arduino:sam:arduino_due_x
- esp8266:esp8266:huzzah:eesz=4M3M,xtal=80
- esp32:esp32:featheresp32:FlashFreq=80
- STM32:stm32:GenF1:pnum=BLUEPILL_F103C8

## Requests for modifications / extensions
Please write me a PM including your motivation/problem if you need a modification or an extension.

#### If you find this library useful, please give it a star.
