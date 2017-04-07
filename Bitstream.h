// Bitstream.h

#ifndef _BITSTREAM_h
#define _BITSTREAM_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif



// defines for direct port access and hardware debugging pulses
#define HW_IRQ_PORT() PIND & 0x04                                                    // direct access h/w port pin 2
#define HW_DEBUG_PULSE() { PORTB = PORTB | (1 << 3); PORTB = PORTB & ~(1 << 3); }    // pulse pin 11
#define HW_DEBUG_PULSE_ON() PORTB = PORTB | (1 << 3)                                 // set pin 11 high
#define HW_DEBUG_PULSE_OFF() PORTB = PORTB & ~(1 << 3)                               // set pin 11 low

#define ERR_INVALID_HALF_BIT             1
#define ERR_SEQUENTIAL_ERROR_LIMIT       2


typedef void (*DataFullHandler)(unsigned long BitData);
typedef void (*ErrorHandler)(byte ErrorCode);


// states
enum State
{
    ACQUIRE,
    NORMAL,
};


class BitStream
{
public:
    BitStream(byte interruptPin, boolean withPullup);
    BitStream(byte interruptPin, boolean withPullup,
        unsigned int OneMin, unsigned int OneMax, unsigned int ZeroMin, unsigned int ZeroMax, byte MaxErrors);
    void SetDataFullHandler(DataFullHandler Handler);
    void SetErrorHandler(ErrorHandler Handler);

private:

    // Default DCC microsecond 0 & 1 timings
    static unsigned int timeOneMin;
    static unsigned int timeOneMax;
    static unsigned int timeZeroMin;
    static unsigned int timeZeroMax;          // 110 us for normal bit, 10000 us to allow zero-stretching
    static byte maxBitErrors;                 // max number of bit errors before we revert to acquire state

    // Event handlers
    static DataFullHandler dataFullHandler;        // handler for the timer event
    static ErrorHandler errorHandler;              // handler for errors

    // Interrupt and state variables
    static unsigned long lastInterruptTime;        // time of last interrupt
    static byte lastHalfBit;                       // the last half bit captured
    static byte bitEndsHighOrLow;                  // do the bits end with a high or low half bit
    static State state;                            // current state of the acquisition
    static byte bitErrorCount;                     // current number of sequential bit errors

    // Queue structure
    static const byte maxBitIndex = 31;            // 32 bits total to store in unsigned long
    static byte queueSize;                         // current size of the queue
    static unsigned long bitData;                  // stores the bitstream

    // private methods
    static void HandleInterrupt();                 // handles the hardware interrupt
    static void QueuePut(boolean newBit);
};

#endif

