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

#include "Bitstream.h"

// define/initialize static vars
boolean BitStream::lastPinState = 0;
SimpleQueue BitStream::simpleQueue;

BitStream::BitStream()
{
	pinMode(HWirqPin, INPUT_PULLUP);
	pinMode(ICRPin, INPUT_PULLUP);
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

	stateFunctionPointer = 0;

#if defined(TIMER1_ICR_0PS) || defined(TIMER1_ICR_8PS)
	TIMSK1 = 0;                                             // disable input capture interrupt
#else
	detachInterrupt(digitalPinToInterrupt(HWirqPin));   // disable the h/w interrupt
#endif

	interrupts();
}


// begin or resume processing interrupts
void BitStream::Resume()
{
	noInterrupts();

	// initialize the outgoing queue
	queueSize = 0;
	bitData = 0;

	// set the startup state
	simpleQueue.Reset();    // reset the queue of DCC timestamps
	stateFunctionPointer = &BitStream::StateStartup;

#if defined(TIMER1_HW_0PS)
	TCCR1A = 0;    // reset the registers initially
	TCCR1B = 0;
	TCCR1C = 0;
	TCNT1 = 0;
	TIMSK1 = 0;
	TCCR1B |= (1 << 0);   // set CS10 bit for no prescaler (0.0625 us resolution, overflow every 4 ms)
	attachInterrupt(digitalPinToInterrupt(HWirqPin), GetTimestamp, CHANGE);   // enable h/w interrupt
#endif
#if defined(TIMER1_HW_8PS)
	TCCR1A = 0;    // reset the registers initially
	TCCR1B = 0;
	TCCR1C = 0;
	TCNT1 = 0;
	TIMSK1 = 0;
	TCCR1B |= (1 << 1);   // set CS11 bit for 8 prescaler (0.5 us resolution, overflow every 32 ms)
	attachInterrupt(digitalPinToInterrupt(HWirqPin), GetTimestamp, CHANGE);   // enable h/w interrupt
#endif
#if defined(TIMER1_ICR_0PS)
	TCCR1A = 0;    // reset the registers initially
	TCCR1B = 0;
	TCCR1C = 0;
	TCNT1 = 0;
	TIMSK1 = 0;
	TCCR1B |= (1 << 0);  // set CS10 bit for no prescaler (0.0625 us resolution, overflow every 4 ms)
	TCCR1B |= (1 << 6);  // set input capture edge select bit for rising
	TCCR1B |= (1 << 7);  // set input capture noise canceler
	TIMSK1 |= (1 << 5);  // enable input capture interrupt
#endif
#if defined(TIMER1_ICR_8PS)
	TCCR1A = 0;    // reset the registers initially
	TCCR1B = 0;
	TCCR1C = 0;
	TCNT1 = 0;
	TIMSK1 = 0;
	TCCR1B |= (1 << 1);   // set CS11 bit for 8 prescaler (0.5 us resolution, overflow every 32 ms)
	TCCR1B |= (1 << 6);  // set input capture edge select bit for rising
	TCCR1B |= (1 << 7);  // set input capture noise canceler
	TIMSK1 |= (1 << 5);  // enable input capture interrupt
#endif
#if defined(TIMER2_HW_8PS)
	TCCR2A = 0;
	TCCR2B = 0;
	TCNT2 = 0;
	TIMSK2 = 0;
	TCCR2B |= (1 << 1);  	// set CS21 bit for 8 prescaler (0.5 us resolution, overflow every 127.5 us)
	attachInterrupt(digitalPinToInterrupt(HWirqPin), GetTimestamp, CHANGE);   // enable h/w interrupt
#endif
#if defined(TIMER2_HW_32PS)
	TCCR2A = 0;
	TCCR2B = 0;
	TCNT2 = 0;
	TIMSK2 = 0;
	TCCR2B |= (1 << 0); // set CS20 and CS21 bits for 32 prescaler (2.0 us resolution, overflow every 0.5 ms)
	TCCR2B |= (1 << 1);
	attachInterrupt(digitalPinToInterrupt(HWirqPin), GetTimestamp, CHANGE);   // enable h/w interrupt
#endif
#if defined(TIMER_ARM_HW_8PS)
	ArmTimerSetup();
	attachInterrupt(digitalPinToInterrupt(HWirqPin), GetTimestamp, CHANGE);   // enable h/w interrupt
#endif


	interrupts();
}


// initialize state and get the value for the first half bit
void BitStream::StateStartup()
{
	// check for valid half bit and use it to initialize lastHalfBit
	// this effectively ignores any initial bit errors during startup, etc.
	if (isOne || isZero)
	{
		// initialize state vars
		endOfBit = false;
		bitErrorCount = 0;

		// set half bit and begin loooking for transition
		lastHalfBit = isOne;
		stateFunctionPointer = &BitStream::StateSeek;
	}
}


// find the first 1/0 or 0/1 transition, after initializing lastHalfBit
void BitStream::StateSeek()
{
	// check for valid half bit
	if (isOne || isZero)
	{
		// check for transition from previous to current half bits
		if (isOne != lastHalfBit)
		{
			endOfBit = true;   // transitioned from 1 to 0 or vice versa, so the next half bit is the bit end
			stateFunctionPointer = &BitStream::StateNormal;
		}

		// save the last half bit
		lastHalfBit = isOne;
	}
	else
	{
		// error occurred looking for transition, go back to startup
		stateFunctionPointer = &BitStream::StateStartup;
	}
}


// normal processing for half bits
void BitStream::StateNormal()
{
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

		// save the last half bit
		lastHalfBit = isOne;
	}
	else
	{
		HandleError();    // didn't get a valid 1 or 0, process the error
	}
}


// handle errors that occur during normal processing
void BitStream::HandleError()
{
	// classify the error
	byte errorNum = 0;
	if (period < timeOneMin) errorNum = ERR_INVALID_HALF_BIT_LOW;
	if ((period > timeOneMax) && (period < timeZeroMin)) errorNum = ERR_INVALID_HALF_BIT_MID;
	if (period > timeZeroMax) errorNum = ERR_INVALID_HALF_BIT_HIGH;

	// callback error handler
	if (errorHandler)
		errorHandler(errorNum);

	bitErrorCount++;        // increment error count
	if (bitErrorCount > maxBitErrors)
	{
		// exceeded max bit errors, go back to startup state
		stateFunctionPointer = &BitStream::StateStartup;

		// callback error handler
		if (errorHandler)
			errorHandler(ERR_SEQUENTIAL_ERROR_LIMIT);
	}
}


// process the queued DCC timestamps to see if they represent a one or a zero.
void BitStream::ProcessTimestamps()
{
#ifdef _DEBUG
	// generate debugging pulses on scope to monitor simpleQueue size.
	for (int i = 0; i < simpleQueue.Size(); i++)
	{
		HW_DEBUG_PULSE_18();
	}
#endif // DEBUG

	while (simpleQueue.Size() > 0)
	{
		// get the current timestamp to check
		currentCount = simpleQueue.Get();

		// get the period between the current and last timestamps
		period = currentCount - lastInterruptCount;

		// TODO: why doesn't ARM subtraction overflow work like AVR above
		#if defined(TIMER_ARM_HW_8PS)
		if (currentCount < lastInterruptCount)
			period = currentCount + (0xFFFF - lastInterruptCount);   // make sure we get the right period on overflow on ARM
		#endif

		// does the period give a 1 or a 0?
		isOne = (period >= timeOneMin && period <= timeOneMax);
		isZero = (period >= timeZeroMin && period <= timeZeroMax);

		// perform the current state function
		if (stateFunctionPointer)
			(*this.*stateFunctionPointer)();

		// save the time of the last interrupt
		lastInterruptCount = currentCount;
	}
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


// get pulse timings using hardware interrupt
void BitStream::GetTimestamp()    // static
{
	// get the timer count before we do anything else
#if defined (TIMER1_HW_0PS) || defined(TIMER1_ICR_0PS) || defined(TIMER1_HW_8PS) || defined(TIMER1_ICR_8PS)
	const unsigned int count = TCNT1;
#endif
#if defined(TIMER2_HW_8PS) || defined(TIMER2_HW_32PS)
	const byte count = TCNT2;
#endif
#if defined(TIMER_ARM_HW_8PS)
	//TcCount16* TC = (TcCount16*) TC3;           // get the timer structure. TODO: get the real structure we need here.
	//const unsigned int count = TC->COUNT.reg;
	const unsigned int count = ((TcCount16*)TC3)->COUNT.reg;
#endif

	// timestamp assignment complete ~3 us after DCC state change

	// check pinstate for change (TODO: why are there spurious IRQs here?
	#if !defined(TIMER_ARM_HW_8PS)
	const boolean pinState = HW_IRQ_PORT();                 // this takes ~0.2 us
	if (pinState == lastPinState) return;
	lastPinState = pinState;
	#endif
	
	// add the timestamp to the queue
	simpleQueue.Put(count);

	// 2.5 microseconds, with pin state check, to add new timestamp to queue
}


// get pulse timings using input capture register
// TODO: use portable ISR macro so we don't have to ifdef this for compilation on arm
#if defined(TIMER1_ICR_0PS) || defined(TIMER1_ICR_8PS)
ISR(TIMER1_CAPT_vect)        // static, global
{
	const unsigned int capture = ICR1;    // store the capture register before we do anything else
	TCCR1B ^= 0x40;                 // toggle the edge select bit,  (1<<6) = 0x40

	BitStream::simpleQueue.Put(capture);      // add the value in the input capture register to the queue
}
#endif


#if defined(TIMER_ARM_HW_8PS)
void BitStream::ArmTimerSetup()
{
	// initially from https://github.com/maxbader/arduino_tools/blob/master/libraries/timer_m0_tc_counter/timer_m0_tc_counter.ino#L22-L56
	// see also servo.h for samd boards

	// Enable clock for TC 
	//REG_GCLK_CLKCTRL =  (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3));
	GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC2_TC3));
	while (GCLK->STATUS.bit.SYNCBUSY == 1); // wait for sync 

	// The type cast must fit with the selected timer mode 
	TcCount16* TC = (TcCount16*)TC3; // get timer struct

	TC->CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCCx
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

	TC->CTRLA.reg |= TC_CTRLA_MODE_COUNT16;  // Set Timer counter Mode to 16 bits
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

	TC->CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8;   // Set prescaler
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 

	// Enable TC
	TC->CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC->STATUS.bit.SYNCBUSY == 1); // wait for sync 
}
#endif
