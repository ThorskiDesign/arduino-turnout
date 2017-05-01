
#include "Bitstream.h"

//// define/initialize static vars
//unsigned int BitStream::timeOneMin = DCC_DEFAULT_ONE_MIN * CLOCK_SCALE_FACTOR;
//unsigned int BitStream::timeOneMax = DCC_DEFAULT_ONE_MAX * CLOCK_SCALE_FACTOR;
//unsigned int BitStream::timeZeroMin = DCC_DEFAULT_ZERO_MIN * CLOCK_SCALE_FACTOR;
//unsigned int BitStream::timeZeroMax = DCC_DEFAULT_ZERO_MAX * CLOCK_SCALE_FACTOR;
//byte BitStream::maxBitErrors = 10;
//
//byte BitStream::interruptPin;
//BitStream::State BitStream::state = SUSPEND;
//boolean BitStream::lastHalfBit = 0;
//byte BitStream::bitErrorCount = 0;
//byte BitStream::queueSize = 0;
//boolean BitStream::endOfBit = false;
//
//unsigned int BitStream::lastInterruptCount = 0;    // 16 bit int for proper overflow with TCNT1
//unsigned long BitStream::bitData = 0;
//
//BitStream::DataFullHandler BitStream::dataFullHandler;
//BitStream::ErrorHandler BitStream::errorHandler;


// define/initialize static vars
boolean BitStream::lastPinState = 0;
SimpleQueue BitStream::simpleQueue;


// set up the bitstream capture
BitStream::BitStream(byte intPin, boolean withPullup)
{
	interruptPin = intPin;

	// configure pins
	if (withPullup)
	{
		pinMode(interruptPin, INPUT_PULLUP);
	}
	else
	{
		pinMode(interruptPin, INPUT);
	}
}


// set up the bitstream capture with non-default timings
BitStream::BitStream(byte interruptPin, boolean withPullup, unsigned int OneMin, unsigned int OneMax,
	unsigned int ZeroMin, unsigned int ZeroMax, byte MaxErrors) : BitStream(interruptPin, withPullup)
{
	timeOneMin = OneMin * CLOCK_SCALE_FACTOR;
	timeOneMax = OneMax * CLOCK_SCALE_FACTOR;
	timeZeroMin = ZeroMin * CLOCK_SCALE_FACTOR;
	timeZeroMax = ZeroMax * CLOCK_SCALE_FACTOR;
	maxBitErrors = MaxErrors;
}


// set the handler for the data full event
void BitStream::SetDataFullHandler(DataFullHandler Handler)
{
	dataFullHandler = Handler;
}


// set the handler for error events
void BitStream::SetErrorHandler(ErrorHandler Handler)
{
	errorHandler = Handler;
}


// suspend processing of interrupts
void BitStream::Suspend()
{
	noInterrupts();
	state = SUSPEND;
	detachInterrupt(digitalPinToInterrupt(interruptPin));
	interrupts();
}


// begin or resume processing interrupts
void BitStream::Resume()
{
	if (state != SUSPEND) return;   // skip resume if we aren't already suspended

	noInterrupts();

	// Configure timer1 for use in getting interrupt timing, just need a looping counter.
	// Set up with 8 prescaler. this gives 0.5 us resolution with an overflow every 32.7ms.
	TCCR1A = 0;    // reset the registers initially
	TCCR1B = 0;
	TCCR1C = 0;
	TCNT1 = 0;
	TIMSK1 = 0;
	TCCR1B |= (1 << 1);   // set CS11 bit for 8 prescaler

	// reset state and bitstream vars
	state = NORMAL;
	lastHalfBit = 0;
	endOfBit = false;
	bitErrorCount = 0;
	simpleQueue.Reset();    // reset the queue of DCC timestamps

	// initialize the outgoing queue
	queueSize = 0;
	bitData = 0;

	// set starting time and attach interrupt
	lastInterruptCount = TCNT1;
	attachInterrupt(digitalPinToInterrupt(interruptPin), GetIrqTimestamp, CHANGE);

	interrupts();
}


// process the queued DCC timestamps to see if they represent a one or a zero.
void BitStream::ProcessTimestamps()
{
	//if (simpleQueue.Size()>0) HW_DEBUG_PULSE();
	//if (simpleQueue.Size()>1) HW_DEBUG_PULSE();
	//if (simpleQueue.Size()>2) HW_DEBUG_PULSE();
	//if (simpleQueue.Size()>3) HW_DEBUG_PULSE();
	//if (simpleQueue.Size()>4) HW_DEBUG_PULSE();
	//if (simpleQueue.Size()>5) HW_DEBUG_PULSE();

	while (simpleQueue.Size() > 0)
	{
		// get the current timestamp to check
		unsigned int currentCount = simpleQueue.Get();

		// get the period between the current and last timestamps
		unsigned int period = currentCount - lastInterruptCount;

		// does the period give a 1 or a 0?
		boolean isOne = (period >= timeOneMin && period <= timeOneMax);
		boolean isZero = (period >= timeZeroMin && period <= timeZeroMax);

		// check for valid half bit
		if (isOne || isZero)
		{
			// now we have validated we have a 1 or 0, so isOne represents our current half bit

			// check if the current and previous half bits match
			if (isOne == lastHalfBit)
			{
				if (endOfBit)             // is this the ending half bit?
				{
					QueuePut(isOne);      // add the bit to the queue
					endOfBit = false;
					bitErrorCount = 0;    // reset error count after full valid bit
				}
				else
				{
					endOfBit = true;      // this wasn't the ending half bit, so the next one will be
				}
			}
			else
			{
				endOfBit = true;   // transitioned from 1 to 0 or vice versa, so the next half bit is the bit end
			}

			// save the time of the last valid interrupt and half bit
			lastInterruptCount = currentCount;
			lastHalfBit = isOne;
		}
		else    // didn't get a valid half bit, so begin error handling
		{
			// callback error handler
			if (errorHandler)
				errorHandler(ERR_INVALID_HALF_BIT);

			bitErrorCount++;        // increment error count
			if (bitErrorCount > maxBitErrors)
			{
				// normally assume an error is an intermittent event, and don't reset lastInterrupt time,
				// so that the next 'real' interrupt still has a valid starting point.
				// but, if we have > maxBitErros, assume something bad happened and set it here,
				// so that the subsequent period is based on the latest irq time.
				lastInterruptCount = currentCount;
				simpleQueue.Reset();

				// callback error handler
				if (errorHandler)
					errorHandler(ERR_SEQUENTIAL_ERROR_LIMIT);
			}
		}

		// 6 - 8 us per timestamp

	}     // end while
}


// add a bit to the queue, performing callback and reset if full
void BitStream::QueuePut(boolean newBit)
{
	// method executes in ~2 us, unless we are at max queue size, which adds ~3us.

	// shift the previous bits left and add the new bit
	bitData = (bitData << 1) + newBit;    // approx 1.5 us

	// perform callback and reset if queue is full
	// this adds about 3us when we hit it
	queueSize++;
	if (queueSize > maxBitIndex)
	{
		if (dataFullHandler)
			dataFullHandler(bitData);
		queueSize = 0;
		bitData = 0;
	}
}


void BitStream::GetIrqTimestamp()    // static
{
	boolean pinState = HW_IRQ_PORT();                 // this takes ~0.2 us
	if (pinState == lastPinState) return;
	lastPinState = pinState;

	simpleQueue.Put(TCNT1);

	// 2.5 microseconds, with pin state check, to add new timestamp to queue
}
