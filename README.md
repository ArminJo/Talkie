# Talkie

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

Speech library for Arduino
The original version can be found [here](https://github.com/going-digital/Talkie)

The actual version can be downloaded directly from GitHub [here](https://github.com/ArminJo/Talkie/blob/master/extras/Talkie.zip?raw=true)

Youtube Demonstration of Talkie Voltmeter example

[![Demonstration of Talkie Voltmeter example](https://img.youtube.com/vi/6jXkugZTwCs/0.jpg)](https://www.youtube.com/watch?v=6jXkugZTwCs)

## Improvements to the original and to the non blocking version of PaulStoffregen
- Improved code so Talkie now runs on **8MHz** Arduino (with millis() interrupt disabled while talking)
- Fix the ISR_RATIO Bug for plain Arduino
- Added utility functions, extracted from the examples like sayQNumber(), sayQFloat(), sayQVoltageMilliVolts().
- Added a lot of comments and do refactoring to better understand the functionality
- Added compatibility to Arduino Tone library by stopping timer1 interrupts at every end of speech.
- Extracted initializeHardware() and terminateHardware() functions.
- Supports **ATmega328** as found on the **Uno** and **Nano** bords, **ATmega32U4** as found on the **Leonardo** and **CircuitPlaygound** boards.

* The Library uses Timer 1 and Timer 2 while speaking, which can break Servo, analogWrite(), and some other libraries.

Porting to ATtiny is not possible, since they lack the hardware multiplication. ( Believe me, I tried it :-( )
