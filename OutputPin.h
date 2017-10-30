
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
