
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
void EventTimer::SetTimerHandler(void (*Handler)()) { timerHandler = Handler; }
