
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
	explicit OutputPin(byte Pin);
	void SetPin(bool State);
	bool GetState();

private:
	byte pin;
	bool state;
};

#endif
