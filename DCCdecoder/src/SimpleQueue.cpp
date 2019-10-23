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
