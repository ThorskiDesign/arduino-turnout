
#include "Relay.h"

RelayClass::RelayClass(byte Pin)
{
	pin = Pin;
    state = LOW;
	pinMode(pin, OUTPUT);
}

void RelayClass::SetRelay(bool State)
{
	state = State;
	digitalWrite(pin, state);
}

bool RelayClass::GetState()
{
	return state;
}
