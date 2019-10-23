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

Event Timer

A simple timer class to trigger a callback after a specified duration.

Example usage:

		EventTimer timer;                 // create an instance of an event timer.
		timer.StartTimer(250);            // start the timer with the spcified duration.
		timer.SetTimerHandler(handler);   // set the handler to call when the duration has elapsed.

*/


#ifndef _EVENTTIMER_h
#define _EVENTTIMER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

class EventTimer
{
public:
    typedef void (*EventTimerHandlerFunc)();

    EventTimer();
	void StartTimer(unsigned int Duration);
	void Update(unsigned long CurrentMillis);
	void Update();
	bool IsActive();
	void SetTimerHandler(EventTimerHandlerFunc Handler);

private:
	unsigned long startTime = 0;
	unsigned long duration = 0;
	bool isActive = false;
	EventTimerHandlerFunc timerHandler = 0;   // pointer to handler for the timer event
};

#endif
