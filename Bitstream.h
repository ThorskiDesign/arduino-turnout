
/* 

DCC bitstream capture

A class designed to capture the raw bit sequence of a DCC signal, as defined here:
http://www.nmra.org/sites/default/files/standards/sandrp/pdf/s-9.1_electrical_standards_2006.pdf

Summary:

An input capture register is configured so that each time the signal transitions, the ISR is called. 
In the ISR, the time of the interrupt in Timer1 counts is stored in a queue as an unsigned int. 
When the timestamps are retrieved from the queue, the time between the current timestamp and the 
previous timestamp is used to determine whether the current half of the bit indicates a 0 or a 1. 
After a valid bit is found, it is added to an output queue. When the output queue is full, a callback 
is performed to provide the data.

Example usage:

    BitStream bitStream(DCCPin, false);                 // create bitstream using ICR and default timings
    BitStream bitStream(DCCPin, false, true 48, 68, 88, 10000, 10);    // create bitstream using non-standard timings
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

The inspection of the timestamps is performed in three states. In the startup state, pulses are
inspected to find the first valid half bit. After this, processing proceeds to the seek state, where
the bits are examined for a transition from 1 to 0 or from 0 to 1, in order to establish which half bit
of each pair is the ending half bit. At this point, the state is synced to the bitstream and normal 
processing begins. In the normal mode, the timestamp queue is examined for a matching pair of half 
bits that makes up a full bit. When a complete bit is found, it is added to the output queue. Error 
checking is performed on each half bit. If a half bit does not fall within the valid ranges for a 
0 or 1, a callback is triggered and a counter incremented. After a configurable number of consecutive 
bit errors, another callback is triggered and processing reverts to the startup state. The bit error 
count is reset after each complete bit. The timestamp queue, period calculations, etc. are done 
using unsigned int so that the Timer1 overflow is handled transparently.

Suspend/Resume methods allow starting, stopping, or resetting the bitstream capture, depending
on outside factors (for example, during times when the signal may be degraded, or when other higher
priority processing needs to take place). The input capture or hardware interrupt is disabled when 
suspended, and enabled when resumed. Timer1 is configured in the Resume method, so that it can be 
used for other purposes (e.g. servo) when the bitstream capture is suspended.

The output queue is an unsigned long, into which 32 bits are stored as they are received. The queue is
shifted left each time a bit is added, so the bits are stored left to right in the order in which 
the are received. After 32 bits have been stored, a callback is triggered, and the queue is reset.

Notes on hardware interrupt timing: The hardware interrupt ISR in NORMAL mode takes 2.5 microseconds
to run, so any pulses faster than that will get lost or have questionable timestamps. Additionally, 
other interrupts may affect the timing of this interrupt. The timing based on the input capture register 
(used by default) is not subject to these effects. 

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
#define HW_IRQ_PORT() PIND & 0x04                                                    // direct access h/w port pin 2
//#define HW_DEBUG_PULSE() { PORTD = PORTD | (1 << 0); PORTD = PORTD & ~(1 << 0); }    // pulse pin 0
//#define HW_DEBUG_PULSE_ON() PORTD = PORTD | (1 << 0)                                 // set pin 0 high
//#define HW_DEBUG_PULSE_OFF() PORTD = PORTD & ~(1 << 0)                               // set pin 0 low
#define HW_DEBUG_PULSE() { PORTC = PORTC | (1 << 4); PORTC = PORTC & ~(1 << 4); }    // pulse pin 18
#define HW_DEBUG_PULSE_ON() PORTC = PORTC | (1 << 4)                                 // set pin 18 high
#define HW_DEBUG_PULSE_OFF() PORTC = PORTC & ~(1 << 4)                               // set pin 18 low

#define ERR_INVALID_HALF_BIT             1
#define ERR_INVALID_HALF_BIT_LOW         2
#define ERR_INVALID_HALF_BIT_MID         3
#define ERR_INVALID_HALF_BIT_HIGH        4
#define ERR_SEQUENTIAL_ERROR_LIMIT       10

#define DCC_DEFAULT_ONE_MIN				52
#define DCC_DEFAULT_ONE_MAX				64
#define DCC_DEFAULT_ZERO_MIN			90
#define DCC_DEFAULT_ZERO_MAX			120    // 110 us for normal bit, 10000 us to allow zero-stretching

//#define CLOCK_SCALE_FACTOR				2U       // number of clock ticks per microsecond
//                                                 // 8 prescaler gives a 0.5 us interval
#define CLOCK_SCALE_FACTOR				16U      // number of clock ticks per microsecond
												 // no prescaler gives a 0.0625 us interval


class BitStream
{
public:
    typedef void (*DataFullHandler)(unsigned long BitData);
    typedef void (*ErrorHandler)(byte ErrorCode);

	// set up the bitstream capture using the ICR and default timings
	BitStream(byte InterruptPin, boolean WithPullup);

    // set up the bitstream capture
    BitStream(byte interruptPin, boolean withPullup, boolean useICR);

    // set up the bitstream capture with non-default timings
    BitStream(byte interruptPin, boolean withPullup, boolean useICR,
		unsigned int OneMin, unsigned int OneMax, unsigned int ZeroMin, unsigned int ZeroMax, byte MaxErrors);

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
	// state pointer and functions
	typedef void(BitStream::*StateFunctionPointer)();
	StateFunctionPointer stateFunctionPointer = 0;
	void StateStartup();
	void StateSeek();
	void StateNormal();
	void HandleError();

	// bitstream capture vars
	unsigned int currentCount = 0;          // timer count for the last pulse
	unsigned int period = 0;                // period of the current pulse
	boolean isOne = false;                  // pulse is within the limits for a 1
	boolean isZero = false;                 // pulse is within the limits for a 0
	unsigned int lastInterruptCount = 0;    // Timer1 count at the last interrupt
	boolean lastHalfBit = 0;                // the last half bit captured
	boolean endOfBit = false;               // second half-bit indicator

	// DCC microsecond 0 & 1 timings
    unsigned int timeOneMin = DCC_DEFAULT_ONE_MIN * CLOCK_SCALE_FACTOR;
    unsigned int timeOneMax = DCC_DEFAULT_ONE_MAX * CLOCK_SCALE_FACTOR;
    unsigned int timeZeroMin = DCC_DEFAULT_ZERO_MIN * CLOCK_SCALE_FACTOR;
    unsigned int timeZeroMax = DCC_DEFAULT_ZERO_MAX * CLOCK_SCALE_FACTOR;

    // Event handlers
    DataFullHandler dataFullHandler = 0;    // handler for the data full event
    ErrorHandler errorHandler = 0;          // handler for errors

    // Interrupt and error variables
	boolean useICR = true;                  // use the input capture register rather than the hardware interrupt
    byte interruptPin;                      // the pin for the hardware irq
    byte bitErrorCount = 0;                 // current number of sequential bit errors
    byte maxBitErrors = 5;                  // max number of bit errors before we revert to startup state
	static boolean lastPinState;            // last state of the IRQ pin

    // Output queue structure
    const byte maxBitIndex = 31;            // 32 bits total to store in unsigned long
    byte queueSize = 0;                     // current size of the queue
    unsigned long bitData = 0;              // stores the bitstream

    // private methods
    void QueuePut(boolean newBit);          // adds a bit to the queue
	static void GetIrqTimestamp();		    // get and queue the timestamp of an interrupt
};

#endif
