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

#include "OutputPin.h"

OutputPin::OutputPin(byte Pin)
{
	pin = Pin;
    state = LOW;
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
}

void OutputPin::SetPin(bool State)
{
	state = State;
	digitalWrite(pin, state);
}

bool OutputPin::GetState()
{
	return state;
}
