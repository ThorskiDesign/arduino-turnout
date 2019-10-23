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

#include "EventTimer.h"


// constructor
EventTimer::EventTimer()
{
}


// start a timer with the specified duration
void EventTimer::StartTimer(unsigned int Duration)
{
	duration = Duration;
	startTime = millis();
	isActive = true;
}


// check if the timer has elapsed, should be called in millis interrupt or similar
void EventTimer::Update(unsigned long CurrentMillis)
{
	// has the duration elapsed
	if (isActive && (CurrentMillis - startTime > duration))
	{
		// disable timer
		isActive = false;

		// raise event
		if (timerHandler) timerHandler();
	}
}


// in case we want to update without specifying millis
void EventTimer::Update()
{
	Update(millis());
}


// get the current state of the timer
bool EventTimer::IsActive() { return isActive; }


// set the handler for the timer event
void EventTimer::SetTimerHandler(EventTimerHandlerFunc Handler) { timerHandler = Handler; }
