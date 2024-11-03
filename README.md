<div align = center>

# [Talkie](https://github.com/ArminJo/Talkie)
Speech library for Arduino

[![Badge License: GPLv3](https://img.shields.io/badge/License-GPLv3-brightgreen.svg)](https://www.gnu.org/licenses/gpl-3.0)
 &nbsp; &nbsp; 
[![Badge Version](https://img.shields.io/github/v/release/ArminJo/Talkie?include_prereleases&color=yellow&logo=DocuSign&logoColor=white)](https://github.com/ArminJo/Talkie/releases/latest)
 &nbsp; &nbsp; 
[![Badge Commits since latest](https://img.shields.io/github/commits-since/ArminJo/Talkie/latest?color=yellow)](https://github.com/ArminJo/Talkie/commits/master)
 &nbsp; &nbsp; 
[![Badge Build Status](https://github.com/ArminJo/Talkie/workflows/LibraryBuild/badge.svg)](https://github.com/ArminJo/Talkie/actions)
 &nbsp; &nbsp; 
![Badge Hit Counter](https://visitor-badge.laobi.icu/badge?page_id=ArminJo_Talkie)
<br/>
<br/>
[![Stand With Ukraine](https://raw.githubusercontent.com/vshymanskyy/StandWithUkraine/main/badges/StandWithUkraine.svg)](https://stand-with-ukraine.pp.ua)

Available as [Arduino library "Talkie"](https://www.arduinolibraries.info/libraries/talkie).

[![Button Install](https://img.shields.io/badge/Install-brightgreen?logoColor=white&logo=GitBook)](https://www.ardu-badge.com/Talkie)
 &nbsp; &nbsp; 
[![Button Changelog](https://img.shields.io/badge/Changelog-blue?logoColor=white&logo=AzureArtifacts)](https://github.com/ArminJo/Talkie?tab=readme-ov-file#revision-history)

<br/>
The original version can be found [here](https://github.com/going-digital/Talkie).<br/>
A good explanation of the TMS5220 operation and the LPC frame format can be found [here](https://github.com/mamedev/mame/blob/master/src/devices/sound/tms5220.txt).

</div>

#### If you find this library useful, please give it a star.

&#x1F30E; [Google Translate](https://translate.google.com/translate?sl=en&u=https://github.com/ArminJo/Talkie)

<br/>

# YouTube Videos
| Demonstration of Talkie voltmeter example | Intoduction by [Gadget Reboot](https://www.youtube.com/channel/UCwiKHTegfDe33K5wnmyULog) |
| :-: | :-: |
| [![Demonstration of Talkie voltmeter example](https://img.youtube.com/vi/6jXkugZTwCs/0.jpg)](https://www.youtube.com/watch?v=6jXkugZTwCs) | [![Intoduction by Gadget Reboot](https://img.youtube.com/vi/O_yl5kcRO5w/0.jpg)](https://www.youtube.com/watch?v=O_yl5kcRO5w) |

<br/>

# Improvements to the original and to the non blocking version of PaulStoffregen
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

<br/>

# Pin and timer mapping table for different platforms
| Platform | Pin normal | Pin inverted | 8kHz timer  | PWM timer | Remarks |
|-|-|-|-|-|-|
| AVR (Uno and Nano) | Pin 3 | Pin 11     | timer1   | timer2 |
| ATmega2560  | Pin 6/PH3    | Pin 7/PH4  | timer1   | timer4 |
| Leonardo    | Pin 9/PB5    | Pin 10/PB6 | timer1   | timer4 |
| ProMicro    | Pin 5/PC6    | %      | timer1       | timer4 |
| Esplora     | Pin 6/PD7    | %      | timer1       | timer4 |
| Zero (SAMD) | A0           | %      | TC5          | DAC0   |
| ESP32       | Pin 25       | %      | hw_timer_t   | DAC0   |
| BluePill    | Pin 3        | %      | timer3       | analogWrite | Roger Clarks core |
| BluePill    | PA3          | %      | timer4       | analogWrite | STM core |
| Teensy      | Pin 12/14721 | %     | IntervalTimer | analogWrite |

# Timer usage
**Timer 1** (Servo timer) is used at all ATmegas for updating voice output data at 8 kHz.<br/>
**Timer 2** (Tone timer) on ATmega328 (62500 Hz / 16 &micro;s) or **Timer 4** on ATmega2560 + ATmega32U4 (5 &micro;s) is used to generate the 8 bit PWM output.<br/>
Both timers are therefore not available for other libraries / purposes, like servo control or tone output.

# Differences between BluePill cores
There are **two cores for the BluePill**.
- The `STM32F1` by Roger Clark; http://dan.drown.org/stm32duino/package_STM32duino_index.json
- The CMSIS based `stm32` by ST Microsystems; https://github.com/stm32duino/BoardManagerFiles/raw/master/STM32/package_stm_index.json

Generation of the high speed PWM is **complicated** for Roger Clark core and **easy** for the STM core.
Program size for VoltmeterSayQ.cpp is **21 kByte** for Roger Clark core and **32 kByte** for STM core.
The 8 kHz interrupt handling requires **8 &micro;s** for Roger Clark core and **12 &micro;s** for STM core.

<br/>

# Hints
- As **default** both inverted and not inverted outputs are enabled to **increase volume** if speaker is attached between them.
- The library uses Timer 1 and Timer 2 on ATmega328, so libraries like Tone, Servo, analogWrite(), and some other libraries cannot be used while speaking.
- After a call to `say...()` you can use `tone()` again.
- using [Sloeber]ervo `write()` after a call to say... you must `detach()` and `attach()` the servo before first `write()` in order to initialize the timer again for Servo.
- If you want to use **SPI** functions on ATmega328 **while Talkie is speaking**, then disable Talkies usage of pin 11 by `Talkie Voice(true, false);` instead of `Talkie Voice;` **or** `Voice.doNotUseInvertedOutput();`.
- Porting to ATtinys is not possible, since they lack the hardware multiplication. ( Believe me, I tried it! )
- I use the **speakers from old earphones or headphones**, which have approximately **16 to 32 &ohm;**, directly without a series resistor on my ATmegas. The headphone speaker tend to be much louder, especially when they stay in their original housings. If you do not connect the speaker between non inverted and inverted output, you must use a series capacitor of 1 to 10 uF do block the DC current. Look for the right polarity. The AC current is proportional to the rectance of the speaker, not its resistance in &ohm;, and it is between 10 and 40 mA. The latter is definitely out of specification for ATmegas but quite loud -what you hear is what you supply- and running for hours on my desk. If you are not sure, just use a piezo speaker or a power amplifier.

<br/>

# Predefined vocabulary
The predefined vocabulary can be found in the [Vocab\_\*.h files](https://github.com/ArminJo/Talkie/blob/master/src), especially in [Vocab_US_Large.h](https://github.com/ArminJo/Talkie/blob/master/src/Vocab_US_Large.h).

# Own vocabulary
To create LPC data you can use the [python_wizard](https://github.com/ptwz/python_wizard) or the [BlueWizard](https://github.com/patrick99e99/BlueWizard) for Mac OS X.

Another way to create LPC data is to use [Qboxpro](http://ftp.whtech.com/pc%20utilities/qboxpro.zip), an unsupported old Windows application running under XP, which can produce Talkie compatible data streams. The missing BWCC.DLL (Borland Windows Custom Control Library) can be found e.g. [here](http://www.download-dll.com/dll-BWCC.dll.html).
The process is described [here](http://furrtek.free.fr/index.php?a=speakandspell&ss=9&i=2) and goes like this:
 - Create a new project using the following project parameters : byte / 8 KHz / 5220 coding table
 - Goto Project and add the audio file
 - Choose process using : medium bit rate and pressing OK
 - Edit concatenation : insert concatenation after by adding a name; then insert phrase and press OK
 - Format it by choosing the first line in the format menu : LPC 10V, 4UV

<br/>

# OUTPUT FILTER
```
C to avoid clicks | Low pass 1600Hz
                   _____
  D3 >-----||-----|_____|-----+-----> to Power amplifier
         100nF      10k       |
                             ---
                             --- 10 nF
                              |
                             _|_ GND
 ```

# Compile options / macros for this library
To customize the software to different requirements, there are some compile options / macros available.<br/>
Modify them by enabling / disabling them, or change the values if applicable.

| Name | Default value | File | Description |
|-|-:|-|-|
| `NO_COMPATIBILITY_FOR_TONE_LIB_REQUIRED` | disabled | Talkie.h | If you do not use the Arduino Tone library, then activating can save up to 844 bytes program size. |
| `FAST_8BIT_MODE` | disabled | Talkie.h | If defined we use 8bit instead of 16 bit coefficients K1 and K2. This saves 10 microseconds (40 instead of 50 us) for a 16 MHz ATmega and has almost the same quality, except of a few "dropouts" e.g. in the word "thousand". |
| `ENABLE_PITCH` | disabled | Talkie.h | If defined we interprete second parameter `aSampleRateForPitch` of `say()` and `SayQ()`. This requires around 160 bytes of program space and few time consuming divisions. If disabled, the parameter `aSampleRateForPitch` is just ignored. |
| `SAMPLE_RATE_DEFAULT` | 8000 | Talkie.h | f you want to globally set pitch for Talkie, you can change this value, this saves the overhead implied by activating `ENABLE_PITCH` |

### Changing include (*.h) files with Arduino IDE
First, use *Sketch > Show Sketch Folder (Ctrl+K)*.<br/>
If you have not yet saved the example as your own sketch, then you are instantly in the right library folder.<br/>
Otherwise you have to navigate to the parallel `libraries` folder and select the library you want to access.<br/>
In both cases the library source and include files are located in the libraries `src` directory.<br/>
The modification must be renewed for each new library version!

### Modifying compile options / macros with PlatformIO
If you are using PlatformIO, you can define the macros in the *[platformio.ini](https://docs.platformio.org/en/latest/projectconf/section_env_build.html)* file with `build_flags = -D MACRO_NAME` or `build_flags = -D MACRO_NAME=macroValue`.

### Modifying compile options / macros with Sloeber IDE
If you are using [Sloeber](https://eclipse.baeyens.it) as your IDE, you can easily define global symbols with *Properties > Arduino > CompileOptions*.<br/>
![Sloeber settings](https://github.com/Arduino-IRremote/Arduino-IRremote/blob/master/pictures/SloeberDefineSymbols.png)

# Schematic for voltmeter example
![Fritzing schematic for voltmeter example](https://github.com/ArminJo/Talkie/blob/master/extras/TalkieVoltmeter_Steckplatine.png)

# Links
Talkie implementation from 2017, based on Peter Knights version [extended with pitch, speed and bending](https://github.com/technologiescollege/ArduinoTechnoEduc/tree/master/portable/sketchbook/libraries/Talkietz).

# Revision History
### Version 1.4.0
- Adding parameter `aSampleRateForPitch` and macro `ENABLE_PITCH`.

### Version 1.3.3
- Adding support for SAMD51 and ESP32 core 3.x.

### Version 1.3.2
- Fixed ESP32 timer bug.

### Version 1.3.1
- Updated _1_Voltmeter example and renamed example names.
- Improved SAMD support.

### Version 1.3.0
- Removed blocking wait for ATmega32U4 Serial in examples.
- 10 bit Coefficients are working now, but they do not sound better :disappointed:.
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
The library examples are tested with GitHub Actions for the following boards:

- arduino:avr:uno
- arduino:avr:leonardo
- arduino:avr:mega
- arduino:sam:arduino_due_x
- esp8266:esp8266:huzzah:eesz=4M3M,xtal=80
- esp32:esp32:featheresp32:FlashFreq=80
- STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8
