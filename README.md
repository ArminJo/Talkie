# [Talkie](https://github.com/ArminJo/Talkie)
Available as Arduino library "Talkie"

### [Version 1.3.1](https://github.com/ArminJo/Talkie/releases)

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

YouTube Intoduction by [Gadget Reboot](https://www.youtube.com/channel/UCwiKHTegfDe33K5wnmyULog)

[![Intoduction by Gadget Reboot](https://img.youtube.com/vi/O_yl5kcRO5w/0.jpg)](https://www.youtube.com/watch?v=O_yl5kcRO5w)

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
  - **ARM M0** (Tested for Arduino Zero) as found on the **SAMD**, **Teensy** and **Particle** boards.
  - **ESP32**. ESP8266 is theoretical possible using FRC2, but for now Arduino shares the FRC1 timer between interrupts and PWM.
  - **ARM M3** (Tested for BluePill) for Roger Clarks as well as STM core.
  
## Pin mapping table for different platforms
| Platform | Normal | Inverted | 8kHz timer | PWM timer | Remarks |
|-|-|-|-|-|-|
| AVR (Uno and Nano) | 3 | 11     | 1     |    2 |
| ATmega2560  | 6/PH3    | 7/PH4  | 1     |    4 |
| Leonardo    | 9/PB5    | 10/PB6 | 1     |    4 |
| ProMicro    | 5/PC6    | %      | 1     |    4 |
| Esplora     | 6/PD7    | %      | 1     |    4 |
| Zero (SAMD) | A0       | %      | TC5   | DAC0 |
| ESP32       | 25       | %      | hw_timer_t    | DAC0 |
| BluePill    | 3        | %      | timer3        | analogWrite | Roger Clarks core |
| BluePill    | PA3      | %      | timer4        | analogWrite | STM core |
| Teensy      | 12/14721 | %      | IntervalTimer | analogWrite |

## Timer usage
**Timer 1** (Servo timer) is used at all ATmegas for updating voice output data at 8 kHz.
**Timer 2** (Tone timer) on ATmega328 (62500 Hz / 16 탎) and **Timer 4** on ATmega2560 + ATmega32U4 (5 탎) is used to generate the 8 bit PWM output.

## Hints
- As **default** both inverted and not inverted outputs are enabled to **increase volume** if speaker is attached between them.
- The library uses Timer 1 and Timer 2 on ATmega328, so libraries like Tone, Servo, analogWrite(), and some other libraries cannot be used while speaking.
- After a call to `say...()` you can use `tone()` again.
- using [Sloeber]ervo `write()` after a call to say... you must `detach()` and `attach()` the servo before first `write()` in order to initialize the timer again for Servo.
- If you want to use **SPI** functions on ATmega328 **while Talkie is speaking**, then disable Talkies usage of pin 11 by `Talkie Voice(true, false);` instead of `Talkie Voice;` **or** `Voice.doNotUseInvertedOutput();`.
- Porting to ATtinys is not possible, since they lack the hardware multiplication. ( Believe me, I tried it! )
- I use the speakers from old earphones or headphones, which have approximately 16 to 32 Ohm, directly without a series resistor on my ATmegas. The headphone speaker tend to be much louder, especially when they stay in their original housings. If you do not connect the speaker between non inverted and inverted output, you must use a series capacitor of 1 to 10 uF do block the DC current. Look for the right polarity. The AC current is proportional to the rectance of the speaker, not its resistance in Ohm, and it is between 10 and 40 mA. The latter is definitely out of specification for ATmegas but quite loud -what you hear is what you supply- and running for hours on my desk. If you are not sure, just use a piezo speaker or a power amplifier.

## Own vocabulary
To create LPC data you can use the [python_wizard](https://github.com/ptwz/python_wizard) like described [here](https://youtu.be/KQseCA0nftI) or the [BlueWizard](https://github.com/patrick99e99/BlueWizard) for Mac OS X.

Another way to create LPC data is to use [Qboxpro](http://ftp.whtech.com/pc%20utilities/qboxpro.zip), an unsupported old Windows application running under XP, which can produce Talkie compatible data streams. The missing BWCC.DLL (Borland Windows Custom Control Library) can be found e.g. [here](http://www.download-dll.com/dll-BWCC.dll.html).
The process is described [here](http://furrtek.free.fr/index.php?a=speakandspell&ss=9&i=2) and goes like this:
 - Create a new project using the following project parameters : Byte / 8 KHz / 5220 coding table
 - Goto Project and add the audio file
 - Choose process using : medium bit rate and pressing OK
 - Edit concatenation : insert concatenation after by adding a name; then insert phrase and press OK
 - Format it by choosing the first line in the format menu : LPC 10V, 4UV

## Schematic for voltmeter example
![Fritzing schematic for voltmeter example](https://github.com/ArminJo/Talkie/blob/master/extras/TalkieVoltmeter_Steckplatine.png)

# OUTPUT FILTER
```
    C to avoid clicks  Low pass 1600Hz  DC decoupling (optional)
                      _____
  D3 >------||-------| 10k |---+----------||-------> to Power amplifier
           100nF      -----    |         10nF
                              ---
                              --- 10 nF
                               |
                               |
                               _ GND
 ```

# Compile options / macros for this library
To customize the software to different car extensions, there are some compile options / macros available.<br/>
Modify it by commenting them out or in, or change the values if applicable. Or define the macro with the -D compiler option for global compile (the latter is not possible with the Arduino IDE, so consider using [Sloeber](https://eclipse.baeyens.it).<br/>
| Option | Default | File | Description |
|-|-|-|-|
| `NO_COMPATIBILITY_FOR_TONE_LIB_REQUIRED` | disabled | Talkie.h | If you do not use the Arduino Tone library, then activating can save up to 844 bytes program size. |
| `FAST_8BIT_MODE` | disabled | Talkie.h | If defined we use 8bit instead of 16 bit coefficients K1 and K2. This saves 10 microseconds (40 instead of 50 us) for a 16 MHz ATmega and has almost the same quality, except of a few "dropouts" e.g. in the word "thousand". |

### Modifying compile options with Arduino IDE
First, use *Sketch > Show Sketch Folder (Ctrl+K)*.<br/>
If you did not yet stored the example as your own sketch, then you are instantly in the right library folder.<br/>
Otherwise you have to navigate to the parallel `libraries` folder and select the library you want to access.<br/>
In both cases the library files itself are located in the `src` directory.<br/>

### Modifying compile options with Sloeber IDE
If you are using Sloeber as your IDE, you can easily define global symbols with *Properties > Arduino > CompileOptions*.<br/>
![Sloeber settings](https://github.com/ArminJo/ServoEasing/blob/master/pictures/SloeberDefineSymbols.png)

# BluePill cores
There are two cores for the PluePill.
- The original Arduino_STM32 by Roger Clark; http://dan.drown.org/stm32duino/package_STM32duino_index.json
- The CMSIS based STM32duino by ST Microsystems; https://github.com/stm32duino/BoardManagerFiles/raw/master/STM32/package_stm_index.json
Generation of the high speed PWM is **complicated** for Roger Clark core and **easy** for the STM core.
Program size for VoltmeterSayQ.cpp is **21 kByte** for Roger Clark core and **32 kByte** for STM core.
The 8 kHz interrupt handling requires **8 탎** for Roger Clark core and **12 탎** for STM core.

# Revision History
### Version 1.3.1
- Updated _1_Voltmeter example and renamed example names.
- Improved SAMD support.

### Version 1.3.0
- Removed blocking wait for ATmega32U4 Serial in examples.
- 10 bit Coefficients are working now, but they do not sound better :-(.
- Tested on an ESP32.
- Tested on a BluePill.

### Version 1.2.0
- Corrected wrong function name doNotUseUseInvertedOutput().
- Added functions `digitalWriteNonInvertedOutput()` and `digitalWriteInvertedOutput()`.
 
### Version 1.1.0
- SAMD support.
- ESP32 support.
- Teensy support.
- Version number.
- Renamed *.c to *.cpp files.
- Added function `sayQTimeout()` in *TalkieUtils.cpp*.
- Added example *USDistanceToVoice*.
- Added function `sayQVoltageVolts()`.
- Improved end handling to minimize clicks.

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
