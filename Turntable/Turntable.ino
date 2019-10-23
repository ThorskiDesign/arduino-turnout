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


#include "TurntableMgr.h"


TurntableMgr TurntableManager;


void setup()
{
#ifdef _DEBUG
	// for timing tests
	pinMode(0, OUTPUT);
	pinMode(1, OUTPUT);

	Serial.begin(115200);
	//delay(1000);   // delay for Serial.print in factory reset (??)
#endif

	// initialize the turntable manager
	TurntableManager.Initialize();
}


void loop()
{
	// this checks for new bitsteam data, and updates the display and stepper motor
	TurntableManager.Update();
}
