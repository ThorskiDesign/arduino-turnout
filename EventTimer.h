// EventTimer.h

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
