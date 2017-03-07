// Relay.h

#ifndef _RELAY_h
#define _RELAY_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class RelayClass
{
public:
	RelayClass(byte Pin);
	void SetRelay(bool State);
	bool GetState();

private:
	byte pin;
	bool state;
};

#endif
