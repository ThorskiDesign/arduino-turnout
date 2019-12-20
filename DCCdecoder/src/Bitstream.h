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

/*

DCC bitstream capture

A class designed to capture the raw bit sequence of a DCC signal, as defined here:
http://www.nmra.org/sites/default/files/standards/sandrp/pdf/s-9.1_electrical_standards_2006.pdf

Summary:

A hardware interrupt or input capture register is configured so that each time the signal transitions,
an ISR is called. In the ISR, the time of the interrupt in timer counts is stored in a queue.
When the timestamps are retrieved from the queue, the time between the current timestamp and the
previous timestamp is used to determine whether the current half of the bit indicates a 0 or a 1.
After a valid bit is found, it is added to an output queue. When the output queue is full, a callback
is performed to provide the data.

Example usage:

	BitStream bitStream();                              // create the bitstream object
	bitStream.Resume();                                 // start the bitstream capture
	bitStream.Suspend();                                // stop the bitstream capture
	bitStream.ProcessTimeStamps();					    // process any DCC timestamps in the queue

Details:

Timestamps for the DCC signal transitions are captured and then inspected to determine if they
represent a one or a zero bit. The caputure can be configured to use either the input capture
register or a hardware interrupt. In either case, the ISR is called on each transition of the DCC
signal and captures timestamps of the DCC events in a queue. The edge select bit of the capture
register is toggled each time in the ISR, so that both rising and falling edges are captured. The
hardware interrupt is similarly configured on CHANGE.

Six combinations of interrupt type, timer, and prescaler are available. Different ISRs are used for
hardware interrupt vs. the input capture register. Calcualtion of the pulse width is done using
unsigned int or byte depending on the selection of timer1 or timer2, so that overflows are handled
correctly. A clock scale factor specifies the number of timer counts per microsecond and is set
according to the selected prescaler. Standard DCC timings are used when using the input capture
register. Slightly wider timings are more reliable for the hardware interrupt due to the effect of
other ISRs that may be running.

The inspection of the timestamps is performed in three states. In the startup state, pulses are
inspected to find the first valid half bit. After this, processing proceeds to the seek state, where
the bits are examined for a transition from 1 to 0 or from 0 to 1, in order to establish which half bit
of each pair is the ending half bit. At this point, the state is synced to the bitstream and normal
processing begins. In the normal mode, the timestamp queue is examined for a matching pair of half
bits that makes up a full bit. When a complete bit is found, it is added to the output queue. Error
checking is performed on each half bit. If a half bit does not fall within the valid ranges for a
0 or 1, a callback is triggered and a counter incremented. After a configurable number of consecutive
bit errors, another callback is triggered and processing reverts to the startup state. The bit error
count is reset after each complete bit.

Suspend/Resume methods allow starting, stopping, or resetting the bitstream capture, depending
on outside factors (for example, during times when the signal may be degraded, or when other higher
priority processing needs to take place). The input capture or hardware interrupt is disabled when
suspended, and enabled when resumed. The timer is configured in the Resume method, so that it can be
used for other purposes (e.g. servo) when the bitstream capture is suspended.

The output queue is an unsigned long, into which 32 bits are stored as they are received. The queue is
shifted left each time a bit is added, so the bits are stored left to right in the order in which
the are received. After 32 bits have been stored, a callback is triggered, and the queue is reset.

*/


#ifndef _BITSTREAM_h
#define _BITSTREAM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


#include "SimpleQueue.h"


// defines for direct port access and hardware debugging pulses
#define HW_IRQ_PORT() PIND & 0x04                                                       // direct access h/w port pin 2

// make sure pinmode is set to output for pins 18 and 19 to use these
#define HW_DEBUG_PULSE_18() { PORTC = PORTC | (1 << 4); PORTC = PORTC & ~(1 << 4); }    // pulse pin 18
#define HW_DEBUG_PULSE_18_ON() PORTC = PORTC | (1 << 4)                                 // set pin 18 high
#define HW_DEBUG_PULSE_18_OFF() PORTC = PORTC & ~(1 << 4)                               // set pin 18 low
#define HW_DEBUG_PULSE_19() { PORTC = PORTC | (1 << 5); PORTC = PORTC & ~(1 << 5); }    // pulse pin 19
#define HW_DEBUG_PULSE_19_ON() PORTC = PORTC | (1 << 5)                                 // set pin 19 high
#define HW_DEBUG_PULSE_19_OFF() PORTC = PORTC & ~(1 << 5)                               // set pin 19 low

enum : byte
{
	ERR_INVALID_HALF_BIT = 1,
	ERR_INVALID_HALF_BIT_LOW = 2,
	ERR_INVALID_HALF_BIT_MID = 3,
	ERR_INVALID_HALF_BIT_HIGH = 4,
	ERR_SEQUENTIAL_ERROR_LIMIT = 10,
};

// set the timer/prescaler combination to use
//#define TIMER1_HW_0PS    // use timer1 hardware irq with no prescaler
//#define TIMER1_HW_8PS    // use timer1 hardware irq with 8 prescaler
//#define TIMER1_ICR_0PS   // use timer1 input capture register with no prescaler
//#define TIMER1_ICR_8PS   // use timer1 input capture register with 8 prescaler
//#define TIMER2_HW_8PS    // use timer2 hardware irq with 8 prescaler
//#define TIMER2_HW_32PS   // use timer2 hardware irq with 32 prescaler
#define TIMER_ARM_HW_8PS   // use timer on arm with hardware irq with 8 prescaler

// use standard DCC timings for ICR
#if defined(TIMER1_ICR_0PS) || defined(TIMER1_ICR_8PS)
enum : byte
{
	DCC_DEFAULT_ONE_MIN = 52,
	DCC_DEFAULT_ONE_MAX = 64,
	DCC_DEFAULT_ZERO_MIN = 90,
	DCC_DEFAULT_ZERO_MAX = 110,     // 110 us for normal bit, 10000 us to allow zero-stretching
};
#endif

// use wider timings for hardware IRQ
#if defined (TIMER1_HW_0PS) || defined(TIMER1_HW_8PS) || defined (TIMER2_HW_8PS) || defined(TIMER2_HW_32PS) || defined(TIMER_ARM_HW_8PS)
enum : byte
{
	DCC_DEFAULT_ONE_MIN = 48,
	DCC_DEFAULT_ONE_MAX = 68,
	DCC_DEFAULT_ZERO_MIN = 88,
	DCC_DEFAULT_ZERO_MAX = 120,
};
#endif

// TODO: convert these to enums, make sure data types are right
// set clock scale factor based on prescaler (number of clock ticks per microsecond)
#if defined(TIMER1_HW_0PS) || defined(TIMER1_ICR_0PS)
enum : uint16_t { CLOCK_SCALE_FACTOR = 16U };   // no prescaler at 16 Mhz gives a 0.0625 us interval
#endif

#if defined(TIMER1_HW_8PS) || defined(TIMER1_ICR_8PS) || defined(TIMER2_HW_8PS)
enum : uint16_t { CLOCK_SCALE_FACTOR = 2U };    // 8 prescaler at 16 Mhz gives a 0.5 us interval
#endif

#if defined(TIMER2_HW_32PS)
#define CLOCK_SCALE_FACTOR 0.5F  // 32 prescaler at 16 Mhz gives a 2.0 us interval
#endif

#if defined(TIMER_ARM_HW_8PS)
enum : uint16_t { CLOCK_SCALE_FACTOR = 6U };   // 8 prescaler at 48 MHz gives a 0.167 us interval
#endif


class BitStream
{
public:
	typedef void(*DataFullHandler)(unsigned long BitData);
	typedef void(*ErrorHandler)(byte ErrorCode);

	// create the bitstream object
	BitStream();

	// configure the callback handlers
	void SetDataFullHandler(DataFullHandler Handler);
	void SetErrorHandler(ErrorHandler Handler);

	// suspend or resume the bitstream capture
	void Suspend();
	void Resume();

	// process the raw timestamp queue
	void ProcessTimestamps();

	static SimpleQueue simpleQueue;         // queue for the DCC timestamps

private:
	// Hardware assignments
	enum : byte
	{
		HWirqPin = 2,
		ICRPin = 8,
	};

	// state pointer and functions
	typedef void(BitStream::*StateFunctionPointer)();
	StateFunctionPointer stateFunctionPointer = 0;
	void StateStartup();
	void StateSeek();
	void StateNormal();
	void HandleError();

	// declare these as byte for 8 bit timers, unsigned int for 16 bit timers
	#if defined (TIMER1_HW_0PS) || defined(TIMER1_ICR_0PS) || defined(TIMER1_HW_8PS) || defined(TIMER1_ICR_8PS) || defined(TIMER_ARM_HW_8PS)
	unsigned int currentCount = 0;          // timer count for the last pulse
	unsigned int period = 0;                // period of the current pulse
	unsigned int lastInterruptCount = 0;    // Timer1 count at the last interrupt
	#endif
	#if defined(TIMER2_HW_8PS) || defined(TIMER2_HW_32PS)
	byte currentCount = 0;          // timer count for the last pulse
	byte period = 0;                // period of the current pulse
	byte lastInterruptCount = 0;    // Timer1 count at the last interrupt
	#endif

		// bitstream capture vars
	boolean isOne = false;                  // pulse is within the limits for a 1
	boolean isZero = false;                 // pulse is within the limits for a 0
	boolean lastHalfBit = 0;                // the last half bit captured
	boolean endOfBit = false;               // second half-bit indicator

	// DCC microsecond 0 & 1 timings
	enum : uint16_t
	{
		timeOneMin = DCC_DEFAULT_ONE_MIN * CLOCK_SCALE_FACTOR,
		timeOneMax = DCC_DEFAULT_ONE_MAX * CLOCK_SCALE_FACTOR,
		timeZeroMin = DCC_DEFAULT_ZERO_MIN * CLOCK_SCALE_FACTOR,
		timeZeroMax = DCC_DEFAULT_ZERO_MAX * CLOCK_SCALE_FACTOR,
	};

	// Event handlers
	DataFullHandler dataFullHandler = 0;    // handler for the data full event
	ErrorHandler errorHandler = 0;          // handler for errors

	// Interrupt and error variables
	byte bitErrorCount = 0;                 // current number of sequential bit errors
	byte maxBitErrors = 5;                  // max number of bit errors before we revert to startup state
	static boolean lastPinState;            // last state of the IRQ pin

	// Output queue structure
	enum : byte { maxBitIndex = 31 };            // 32 bits total to store in unsigned long
	byte queueSize = 0;                     // current size of the queue
	unsigned long bitData = 0;              // stores the bitstream

	// private methods
	void QueuePut(boolean newBit);          // adds a bit to the queue
	static void GetTimestamp();		        // get and queue the timestamp from a hw interrupt
	void ArmTimerSetup();
};

#endif
