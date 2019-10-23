/*

This file is part of Arduino Turnout
Copyright (C) 2017-2018 Eric Thorstenson

Arduino Turnout is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Arduino Turnout is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.

*/

/*

RGB LED

A class for managing an RGB LED.

Summary:

This class provides an object for managing an RGB LED, with seven pre-defined colors and a flash
mode.

Example usage:

	RgbLed led(LedPinR, LedPinG, LedPinB);       // create the led object using the specified output pins.
	led.SetLED(RgbLed::BLUE, RgbLed::FLASH);     // set the led to flashing blue.
	led.Update();                                // update the state of the led based on the current time.

Details:

An led object is created using the specified three output pins for the red, green, and blue elements
of the led. One of seven colors can be specified using the ColorType enum. The led mode (on, off, or
flash) can be set using the ModeType enum. If the led is set on or off, the constituent elements are
simply turned on or off. If the led is set to flash, the elements are turned on and off when the 
on/off intervals have elapsed. The Update() method must be called regularly to manage the timing for
the flashing.

*/



#ifndef _RGB_LED_h
#define _RGB_LED_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class RgbLed
{
public:
	enum ModeType {OFF, ON, FLASH};   // mode of the LED
	enum ColorType {RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE};    // available colors

public:
	RgbLed(byte PinR, byte PinG, byte PinB);
	void SetLED(bool State);
	void SetLED(ColorType C, ModeType T);
	void SetLED(ColorType C, ModeType T, int On, int Off);
	void Update(unsigned long CurrentMillis);
	void Update();

private:
	byte pinR;            // the number of the red pin
	byte pinG;            // the number of the green pin
	byte pinB;            // the number of the blue pin
	unsigned int onTime = 500;     // milliseconds of on-time
	unsigned int offTime = 500;    // milliseconds of off-time

	// these maintain the state for a flashing LED
	bool ledState = LOW;                   // ledState used to set the LED
	unsigned long previousMillis = 0;      // will store last time LED was updated

	// constituent colors for the seven color types
	// order must correspond to the ordering in the ColorType enum
	const bool redState[7] = {HIGH, LOW, LOW, HIGH, LOW, HIGH, HIGH};
	const bool greenState[7] = {LOW, HIGH, LOW, HIGH, HIGH, LOW, HIGH};
	const bool blueState[7] = {LOW, LOW, HIGH, LOW, HIGH, HIGH, HIGH};

	ModeType ledMode;		// type of LED
	ColorType ledColor;     // color of LED

private:
	void TurnColorsOn();
	void TurnColorsOff();
};

#endif
