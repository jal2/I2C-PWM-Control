I2C PWM Control
===============

This project provides the sources for a simple i2c controlled
PWM source based on an ATtiny85.
I need this to control the backlight of a Lenovo Thinkpad L430 - with some hardware mods in the L430 itself.

Call
```
	make help
```
to see all the make targets for building and flashing.

Pin Configuration
-----------------
```
   PB0 - SDA (DIL: pin 7)
   PB1 - PWM (OC0B) (DIL: pin 6)
   PB2 - SCL (DIL: pin 5)
   PB3 - LED (DIL: pin 2)
```
The LED must be connected via a resistor to ground and will light for approx. two seconds each time
a new PWM value is set.

Fuses
-----
The ATtiny shall run at 8MHz in order to have some margin for 100 or even 400kHz i2c operation.

Connect USBASP 10pin to the ATtiny85
------------------------------------
```
	USBASP		ATtiny25/45/85
MOSI	1		5
MISO	9		6
/RES	5		1
SCK	7		7
Vcc	2		8
GND	4,6,8,10	4
```
Be sure to jumper the USBASP for the correct voltage (5 or 3.3V) if you supply
the ATtiny from some other external device.

