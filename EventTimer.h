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
