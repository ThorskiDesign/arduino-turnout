
/* 

DCC bitstream capture

A class designed to capture the raw bit sequence of a DCC signal, as defined here:
http://www.nmra.org/sites/default/files/standards/sandrp/pdf/s-9.1_electrical_standards_2006.pdf

Summary:

A hardware interrupt is configured on CHANGE, so each time the signal transitions, the ISR is called.
The time between the current interrupt and the previous interrupt is used to determine whether the
current half of the bit indicates a 0 or a 1. After a valid bit is found, it is added to the queue.
When the queue is full, a callback is performed to provide the data.

Example usage:

    BitStream bitStream;
    bitStream(DCCPin, false);                           // bitstream capture object, default timings
    bitStream(DCCPin, false, 48, 68, 88, 10000, 10);    // bitstream capture object, non-standard timings
    bitStream.Resume();                                 // start the bitstream capture

Details:

The class operates in two main states, ACQUIRE and NORMAL. In the ACQUIRE mode, the bitstream is 
examined for a 1 half bit, followed by a zero half bit, which allows determination of whether the 
bit pairs end high or low (depending on the polarity of the DCC signal input). A configurable number
of error-free 1-to-0 transitions are required to avoid incorrectly detecting a single spurious event.
After the final transision is found, the bit ending (high or low) is assigned, and NORMAL operation
begins.

In the NORMAL mode, the bitstream is examined for a matching pair of high and low half bits that
makes up a full bit. When a complete bit is found, it is added to the queue.

Error checking is performed on each half bit. If a half bit does not fall within the valid ranges for
a 0 or 1, a callback is triggered and a counter incremented. After a configurable number of 
consecutive bit errors, a callback is triggered and operation falls back to the ACQUIRE mode. The
bit error count is reset after a successful acquisition, and after each complete bit in NORMAL mode.

Suspend/Resume methods allow starting, stopping, or resetting the bitstream capture, depending
on outside factors (for example, during times when the signal may be degraded, or when other higher
priority processing needs to take place). The hardware interrupt is detached when suspended, and
attached when resumed.

The queue is an unsigned long, into which 32 bits are stored as they are received. The queue is
shifted left each time a bit is added, so the bits are stored left to right in the order in which 
the are received. After 32 bits have been stored, a callback is triggered, and the queue is reset.

Notes on hardware timing: The ISR in NORMAL mode takes between 7 and 11 microseconds to run. The
micros() call makes up about 3 of those, and only has a resolution of 4 us. Additionally, any other
interrupts will affect the timing of this interrupt. The 0/1 timing limits default to the DCC spec,
however wider limits will result in reduced bit errors if other interrupts are occuring.

TODO: revise the interrupt timing to use a dedicated timer read (e.g., TIMER2) rather than the
micros call, to provide better resolution and reduce the time in the ISR.

*/


#ifndef _BITSTREAM_h
#define _BITSTREAM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


// defines for direct port access and hardware debugging pulses
#define HW_IRQ_PORT() PIND & 0x04                                                    // direct access h/w port pin 2
#define HW_DEBUG_PULSE() { PORTB = PORTB | (1 << 4); PORTB = PORTB & ~(1 << 4); }    // pulse pin 12
#define HW_DEBUG_PULSE_ON() PORTB = PORTB | (1 << 4)                                 // set pin 12 high
#define HW_DEBUG_PULSE_OFF() PORTB = PORTB & ~(1 << 4)                               // set pin 12 low

#define ERR_INVALID_HALF_BIT             1
#define ERR_SEQUENTIAL_ERROR_LIMIT       2


class BitStream
{
public:
    typedef void (*DataFullHandler)(unsigned long BitData);
    typedef void (*ErrorHandler)(byte ErrorCode);

    // set up the bitstream capture
    BitStream(byte interruptPin, boolean withPullup);

    // set up the bitstream capture with non-default timings
    BitStream(byte interruptPin, boolean withPullup,
        unsigned int OneMin, unsigned int OneMax, unsigned int ZeroMin, unsigned int ZeroMax, byte MaxErrors);

    // configure the callback handlers
    void SetDataFullHandler(DataFullHandler Handler);
    void SetErrorHandler(ErrorHandler Handler);

    // suspend or resume the bitstream capture
    void Suspend();
    void Resume();

private:
    // states
    enum State
    {
        ACQUIRE,
        NORMAL,
        SUSPEND
    };

    // DCC microsecond 0 & 1 timings
    static unsigned int timeOneMin;
    static unsigned int timeOneMax;
    static unsigned int timeZeroMin;
    static unsigned int timeZeroMax;          // 110 us for normal bit, 10000 us to allow zero-stretching
    static byte maxBitErrors;                 // max number of bit errors before we revert to acquire state

    // Event handlers
    static DataFullHandler dataFullHandler;        // handler for the data full event
    static ErrorHandler errorHandler;              // handler for errors

    // Interrupt and state variables
    static byte interruptPin;                               // the pin for the hardware irq
    volatile static unsigned long lastInterruptTime;        // time of last interrupt
    volatile static boolean lastHalfBit;                    // the last half bit captured
    volatile static boolean bitEndsHighOrLow;               // do the bits end with a high or low half bit
    volatile static State state;                            // current state of the acquisition
    volatile static byte bitErrorCount;                     // current number of sequential bit errors
    static boolean candidateBitEnding;             // first identified bit ending to be verified
    static byte bitEndingMatchCount;               // number of matching bit endings recorded
    static const byte bitEndingMinimumMatches = 10;         // number of matches required before we consider a 
                                                            // canditate bit ending valid

    // Queue structure
    static const byte maxBitIndex = 31;            // 32 bits total to store in unsigned long
    volatile static byte queueSize;                // current size of the queue
    volatile static unsigned long bitData;         // stores the bitstream

    // private methods
    static void HandleInterrupt();                 // handles the hardware interrupt
    static void QueuePut(boolean newBit);          // adds a bit to the queue
};

#endif

