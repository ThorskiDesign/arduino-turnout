
#include "SimpleQueue.h"

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
	// check if queue empty
	if (queueSize == 0)
		return 0;

	// get the next index to read
	if (readIndex < maxIndex)
		readIndex++;
	else
		readIndex = 0;

	queueSize--;
	return values[readIndex];
}


// get the current size of the queue.
byte SimpleQueue::Size()
{
	return queueSize;
}


// reset the queue size and read/write counters.
void SimpleQueue::Reset()
{
	queueSize = 0;
	readIndex = 0;
	writeIndex = 0;
}
