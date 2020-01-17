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

DCC bitstream capture

A simple class providing control of an output pin.

Example usage:

	OutputPin outputPin;         // create an instance of OutputPin
	outputPin.SetPin(HIGH);      // set the pin HIGH
	outputPin.GetState();        // get the current state of the pin

*/


#ifndef _OUTPUTPIN_h
#define _OUTPUTPIN_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class OutputPin
{
public:
	OutputPin(byte Pin);
	void SetPin(bool State);
	bool GetState();

private:
	byte pin;
	bool state;
};

#endif
