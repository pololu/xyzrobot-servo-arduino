# XYZrobotServo: Pololu's Arduino library for the XYZrobot Smart Servo A1-16

Version: 1.1.0<br>
Release date: 2017-11-17<br>
[![Build Status](https://travis-ci.org/pololu/xyzrobot-servo-arduino.svg?branch=master)](https://travis-ci.org/pololu/xyzrobot-servo-arduino)<br>
[www.pololu.com](https://www.pololu.com/)

## Summary

This is a library for the Arduino IDE that helps interface with the [A1-16 smart servo][servo] from XYZrobot over serial.  It works with the following products:

- [XYZrobot Smart Servo A1-16][servo]
- [XYZrobot 6 DOF Robotic Arm Kit][arm]
- [XYZrobot Bolide Y-01 Advanced Humanoid Robot DIY Kit][robot]

## Supported platforms

This library works on any Arduino-compatible board, but it works best on boards
based on the ATmega32U4, such as our [A-Star 32U4 controllers][a-star], [Arduino
Leonardo][leo], and [Arduino Micro][micro] due to the issues explained in the
"Issues with receiving data" section below.

## Issues with receiving data

This library can be used without reading any data back from the servos (see the
SetPosition, and SetSpeed examples).  However, if you want to read data from a
servo&mdash;such as its speed, position, or current consumption&mdash;then there
are some issues to consider.

**Receiving data with SoftwareSerial:** On platforms that do not have a free
hardware serial port, such as the Arduino Uno and most other ATmega328P-based
Arduino-compatible boards, the examples in this library use the the
SoftwareSerial library to send and receive serial data.  Unfortunately, the
SoftwareSerial library cannot reliably receive data at 115200 baud, and that is
the default baud rate used by the servos.

To get your sketch to receive data reliably with SoftwareSerial, you will have
to lower the baud rate of the servos to 57600 or lower using this library's
ChangeBaud example.

Sending data with SoftwareSerial is not a problem.

**Enabling a pull-up on RX:** To receive data, a pull-up is needed on your
board's RX line because the servos do not pull their TX lines high while idle.
Without a pull-up, the RX line would be floating and your system would receive
junk bytes before the expected response packet from the servo.

If you are using SoftwareSerial, the RX pull-up is probably enabled
automatically, but it depends on what version of the Arduino IDE you are using
and what board you are using.  If you are using hardware serial, the pull-up is
probably not enabled automatically, so you will either need to add a line of
code to enable it, or add an external pull-up resistor to your setup.

To enable a pull-up resistor on your RX pin, run this line of code after setting
up the serial port:

    pinMode(rxPinNumber, INPUT_PULLUP);

The `rxPinNumber` in the code above should be the number of your RX pin, for
example `0`.

If you are using the hardware serial port on an ATmega32U4-based board, this
library's examples enable the pull-up on pin 0 for you.

## Connecting the hardware

You will need to connect an appropriate power source to your servo or servos.
See the datasheet for your servo for information on the voltage and current
requirements of the power supply, and information about which pins to connect it
to.

To control the servo from your microcontroller board, you must connect the GND
of the microcontroller board to the GND pin of the servo.  You must also connect
your board's TX line to the RX line of the servo.  If you want to receive data
from the servo, you must connect your board's RX line to the TX line of the
servo.

The example sketches for this library use a hardware serial port on your Arduino
if one is available: if your Arduino environment defines
`SERIAL_PORT_HARDWARE_OPEN`, the examples will use that port.  Otherwise, it
uses SoftwareSerial on pins 10 (RX) and 11 (TX).  Therefore, the serial pins to
use depend on which board you are using.

| Microcontroller Board | Hardware serial? | MCU RX pin | MCU TX pin |
|-----------------------|------------------|------------|------------|
| A-Star 32U4           |        Yes       |      0     |      1     |
| Arduino Leonardo      |        Yes       |      0     |      1     |
| Arduino Micro         |        Yes       |      0     |      1     |
| Arduino Mega 2560     |        Yes       |     19     |     18     |
| Arduino Due           |        Yes       |     19**   |     18     |
| Arduino Uno           |        No        |     10     |     11     |
| Arduino Yun           |        No        |     10     |     11     |

** The Due's serial port is 3.3&nbsp;V, so you should not connect it directly to
the servo's 5&nbsp;V TX line.  You could use a voltage divider or level shifter.


## Installing this library

If you are using version 1.6.2 or later of the [Arduino software (IDE)][ide],
you can use the Library Manager to install this library:

1. In the Arduino IDE, open the "Sketch" menu, select "Include Library", then
   "Manage Libraries...".
2. Search for "XYZrobotServo".
3. Click the XYZrobotServo entry in the list.
4. Click "Install".

If this does not work, you can manually install the library:

1. Download the
   [latest release archive from GitHub](https://github.com/pololu/xyzrobot-servo-arduino/releases)
   and decompress it.
2. Rename the folder "xyzrobot-servo-arduino-xxxx" to "XYZrobotServo".
3. Drag the "XYZrobotServo" folder into the "libraries" directory inside your
   Arduino sketchbook directory. You can view your sketchbook location by
   opening the "File" menu and selecting "Preferences" in the Arduino IDE. If
   there is not already a "libraries" folder in that location, you should make
   the folder yourself.
4. After installing the library, restart the Arduino IDE.


## Finding the examples

Several example sketches are available that show how to use the library. You can
access them from the Arduino IDE by opening the "File" menu, selecting
"Examples", and then selecting "XYZrobotServo". If you cannot find these
examples, the library was probably installed incorrectly and you should retry
the installation instructions above.


## Detecting servos

The first example we recommend running is the DetectServos example, which will
detect all the servos that are connected to your board by attempting to read
status information from them.  If your servos are powered and connected
correctly, it determines the baud rate, ID, and ACK policy of each servo, and
prints it out to the Serial Monitor in the Arduino IDE.

However, if you have not connected your board's RX line, this example will not
work, so you should skip this step.


## Configuring your servos

To make sure that this library and its examples can communicate properly with
each of your servos, you might need to change some of the configuration options
in the servo's non-volatile EEPROM memory.

- **Baud rate:** This library's examples use 115200 baud, which is the default
baud rate used by the servos.  If your servo is using a different baud rate, you
should either reconfigure your servo or change the baud rate used in the
examples.
- **Servo ID:** We recommend setting the IDs of your servos to a consecutive
sequence starting at 1.  If you have the [robot arm][arm], the IDs are probably set
correctly already, so you don't need to worry about this.
- **ACK policy:** The *ACK policy* setting determines which commands the servo
responds to.  This library and most of its examples assume that the servos are
using their default ACK policy of 1, which means the servos only respond to the
EEPROM Read, RAM Read, and STAT commands.  If your servos are using a different
ACK policy, that could cause issues.

You should run the DetectServos example describe above before attempting to
reconfigure any of these parameters.  If you need to change a servo's ID, see
the ChangeId example.  If you need to change a servo's baud rate, see the
ChangeBaud example.  We do not yet have an example showing how to change the ACK
policy.


## Getting your servos to move

After you have configured your servos, you should try running a simple example
to make sure you can move your servos.  If you have the [XYZrobot arm][arm], you
should run the RobotArmTest example.  If you have a standalone servo, you should
run the SetPosition or SetSpeed example.


## Documentation

For complete documentation of this library, see the comments in `XYZrobotServo.h`.

[servo]: https://www.pololu.com/product/3400
[arm]: https://www.pololu.com/product/2743
[robot]: https://www.pololu.com/product/2734
[a-star]: https://www.pololu.com/category/149/a-star-programmable-controllers
[leo]: https://www.pololu.com/product/2192
[micro]: https://www.pololu.com/product/2188

## Version history

* 1.1.0 (2017-11-17):
  - Added the BlinkAll example.
  - Better support for the Arduino Uno in the examples:
    - Fixed all examples that use the Serial Monitor to initialize its baud
      rate to 115200 instead of leaving it uninitialized.
    - Fixed the ChangeBaud example so that it does not try to read data at 115200
      baud using software serial, since that is unreliable.
    - Fixed the ChangeBaud and ChangeId examples to work better on boards
      that automatically reset when the Serial Monitor is opened.
      These examples now require user input from the serial monitor.
  - This release contains no changes to the actual library code.
* 1.0.0 (2017-10-13): Original release.
