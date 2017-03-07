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
	EventTimer();
	void StartTimer(unsigned int Duration);
	void Update(unsigned long CurrentMillis);
	void Update();
	bool IsActive();
	void SetTimerHandler(void (*Handler)());

private:
	unsigned long stopTime = 0;
	unsigned long duration = 0;
	bool isActive = false;
	void (*timerHandler)();   // pointer to handler for the timer event
};

#endif
