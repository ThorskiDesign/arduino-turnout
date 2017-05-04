/*

Simple Queue

A simple queue class for use with the BitStream library to capture interrupt timestamps for the
DCC signal.

Summary:

This class provides a queue of unsigned int for storing 16bit timer counts from Timer1. The Put method
is intended to be called from an ISR, and is kept very simple to keep it quick. It increments the write
counter, increments the queue size, and then stores the current value to the array. The Get method 
increments the read counter, decrements the queue size, and then returns that value from the array. 
No checking of the queuesize is performed during the put - calling libs must ensure that the queue is 
emptied in a timely manner, for example by a process running in the main() loop.

Example Usage:

	SimpleQueue simpleQueue;                    // create an instance of the simple queue
	simpleQueue.Put(x);                         // add an unsigned int to the queue
	while (simpleQueue.Size() > 0)              // get values from the queue until empty
		unsigned int y = simpleQueue.Get();
	simpleQueue.Reset();                        // reset the queue size and read/write counters

*/



#ifndef _SIMPLEQUEUE_h
#define _SIMPLEQUEUE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#define maxIndex 20

class SimpleQueue
{
	volatile unsigned int values[maxIndex + 1];
	volatile byte queueSize = 0;
	volatile byte writeIndex = 0;
	byte readIndex = 0;

public:
	SimpleQueue();
	void Put(unsigned int val);
	unsigned int Get();
	byte Size();
	void Reset();
};

#endif

