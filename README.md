# Talkie

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

## Speech library for Arduino
The original version can be found [here](https://github.com/going-digital/Talkie)

Youtube Demonstration of Talkie Voltmeter example

[![Demonstration of Talkie Voltmeter example](https://img.youtube.com/vi/6jXkugZTwCs/0.jpg)](https://www.youtube.com/watch?v=6jXkugZTwCs)

## Improvements to the original and to the non blocking version of PaulStoffregen
- Improved code so Talkie now runs on **8MHz** Arduino (with millis() interrupt disabled while talking).
- Fixed the ISR_RATIO Bug for plain Arduino.
- Added utility functions sayQNumber(), sayQFloat(), sayQVoltageMilliVolts() extracted from the examples.
- Added comments and did refactoring to better understand the functionality.
- Added compatibility to Arduino Tone library by stopping timer1 interrupts at every end of speech.
- Extracted initializeHardware() and terminateHardware() functions for easy adapting to other platforms.
- Currently supporting **ATmega328** as found on the **Uno** and **Nano** bords, **ATmega32U4** as found on the **Leonardo** and **CircuitPlaygound** boards.

## Hints
- Connect the speaker to digital pin 9 and 10 of Arduino. 
- As speaker I use the speakers from old earphones or headphones, which have around 32 Ohm, directly without a series resistor. The headphone speaker tend to be much louder, especially when they stay in their original housings.
- The Library uses Timer 1 and Timer 2, so libraries like Tone, Servo, analogWrite(), and some other libraries cannot be used while speaking.
- After a call to say... you can use tone() again.
- To use Servo write() after a call to say... you must detach() and attach() the servo before in order to initialize the timer again for Servo.
- Porting to ATtinys is not possible, since they lack the hardware multiplication. ( Believe me, I tried it :-( )

### Schematic for voltmeter example
![Fritzing schematic for voltmeter example](https://github.com/ArminJo/Talkie/blob/master/extras/TalkieVoltmeter_Steckplatine.png)
