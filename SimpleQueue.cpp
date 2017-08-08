
#include "SimpleQueue.h"

// constructor
SimpleQueue::SimpleQueue()
{
	for (int i = 0; i < maxIndex + 1; i++)
		values[i] = 0;
}


// add an unsigned int to the queue.
void SimpleQueue::Put(unsigned int val)
{
	// get the next index to write
	if (writeIndex < maxIndex)
		writeIndex++;
	else
		writeIndex = 0;

	queueSize++;
	values[writeIndex] = val;
}


// get an unsigned int from the queue.
unsigned int SimpleQueue::Get()
{
	unsigned int returnVal = 0;

	byte oldSREG = SREG;   // store the current irq state, then disable
	cli();

	// if there are items in the queue
	if (queueSize > 0)
	{
		// get the next index to read
		if (readIndex < maxIndex)
			readIndex++;
		else
			readIndex = 0;

		queueSize--;
		returnVal = values[readIndex];
	}

	SREG = oldSREG;    // restore previous irq state

	return returnVal;
}


// get the current size of the queue.
byte SimpleQueue::Size()
{
	byte returnVal = 0;

	byte oldSREG = SREG;   // store the current irq state, then disable
	cli();

	returnVal = queueSize;

	SREG = oldSREG;    // restore previous irq state

	return returnVal;
}


// reset the queue size and read/write counters.
void SimpleQueue::Reset()
{
	byte oldSREG = SREG;   // store the current irq state, then disable
	cli();

	queueSize = 0;
	readIndex = 0;
	writeIndex = 0;

	for (int i = 0; i < maxIndex + 1; i++)
		values[i] = 0;

	SREG = oldSREG;    // restore previous irq state
}
