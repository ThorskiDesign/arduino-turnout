
#include "Bitstream.h"


// define/initialize static vars
unsigned int BitStream::timeOneMin = 52;
unsigned int BitStream::timeOneMax = 64;
unsigned int BitStream::timeZeroMin = 90;
unsigned int BitStream::timeZeroMax = 10000;   // 110 us for normal bit, 10000 us to allow zero-stretching
byte BitStream::maxBitErrors = 10;

byte BitStream::interruptPin;
volatile BitStream::State BitStream::state = SUSPEND;
volatile boolean BitStream::lastHalfBit = 0;
volatile boolean BitStream::bitEndsHighOrLow = 0;
volatile byte BitStream::bitErrorCount = 0;
volatile byte BitStream::queueSize = 0;
boolean BitStream::candidateBitEnding;
byte BitStream::bitEndingMatchCount;

volatile unsigned long BitStream::lastInterruptTime = 0;
volatile unsigned long BitStream::bitData = 0;

BitStream::DataFullHandler BitStream::dataFullHandler;
BitStream::ErrorHandler BitStream::errorHandler;


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

    //// TODO: configure timer2 for use in getting interrupt timing
    //noInterrupts();
    //TCCR2A = 0;  // zero the regisers initially
    //TCCR2B = 0;
    //TIMSK2 = 0;

    //// configure for 8 prescaler. this gives 0.5 us resolution with a max interval of 127.5 us
    //// for DCC spec timings, the resulting counter limits are  1: 104 to 128, 0: 180 or greater
    //TCCR2B |= (1 << 1);   // configure for 8 prescaler
    //TCNT2 = 0;
    //interrupts();
}


// set up the bitstream capture with non-default timings
BitStream::BitStream(byte interruptPin, boolean withPullup,
    unsigned int OneMin, unsigned int OneMax, unsigned int ZeroMin, unsigned int ZeroMax, byte MaxErrors)
{
    timeOneMin = OneMin;
    timeOneMax = OneMax;
    timeZeroMin = ZeroMin;
    timeZeroMax = ZeroMax;
    maxBitErrors = MaxErrors;

    BitStream(interruptPin, withPullup);
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

    // reset state and bitstream vars
    state = ACQUIRE;
    lastHalfBit = 0;
    bitEndsHighOrLow = 0;
    bitErrorCount = 0;

    // initialize the queue
    queueSize = 0;
    bitData = 0;

    // set starting time and attach interrupt
    lastInterruptTime = micros();
    attachInterrupt(digitalPinToInterrupt(interruptPin), HandleInterrupt, CHANGE);

    interrupts();
}


// add a bit to the queue, performing callback and reset if full
void BitStream::QueuePut(boolean newBit)   // static
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


// process the interrupts
void BitStream::HandleInterrupt()   // static
{
    // get entry time and period
    unsigned long irqTime = micros();    // this takes ~3 us, only has 4 us resolution
    unsigned long period = irqTime - lastInterruptTime;

    // compute one or zero
    boolean isOne = (period >= timeOneMin && period <= timeOneMax);
    boolean isZero = (period >= timeZeroMin && period <= timeZeroMax);

    // check for invalid half bit
    if (!isOne && !isZero)
    {
        // callback error handler
        if (errorHandler)
            errorHandler(ERR_INVALID_HALF_BIT);

        bitErrorCount++;        // increment error count
        if (bitErrorCount > maxBitErrors)
        {
            // normally we assume an error is an intermittent event, and don't reset lastInterrupt time,
            // so that the next 'real' interrupt still has a valid starting point.
            // but, if we have > maxBitErros, assume something bad happened and set it here,
            // so that the subsequent acquire period is based on the latest irq time.
            lastInterruptTime = irqTime;
            state = ACQUIRE;

            // callback error handler
            if (errorHandler)
                errorHandler(ERR_SEQUENTIAL_ERROR_LIMIT);
        }

        return;   // no further processing without a valid one our zero
    }

    // at this point, we have validated we have a 1 or 0, so isOne represents our current half bit

    // perform the state operations
    switch (state)
    {
    case ACQUIRE:
        {
            // look for a one halfbit followed by a zero halfbit
            if (lastHalfBit == 1 && isOne == 0)
            {
                // if this is our first acquisition, get the candidate bit ending
                if (bitEndingMatchCount == 0)
                {
                    // get current pin state. since we check this after the irq that ends the
                    // zero half bit, this represents the state that the bits will end on
                    candidateBitEnding = (HW_IRQ_PORT()) ? 1 : 0;

                    bitEndingMatchCount++;   // not really a match, but start the next steps
                }
                else     // subsequent acquisitions
                {
                    // if we have no errors and the next bit end matches the candidate bit end
                    boolean currentPortReading = (HW_IRQ_PORT()) ? 1 : 0;
                    if (bitErrorCount == 0 && currentPortReading == candidateBitEnding)
                    {
                        if (bitEndingMatchCount < bitEndingMinimumMatches)    // still need more matches
                        {
                            bitEndingMatchCount++;
                        }
                        else      // final matching acquisition
                        {
                            // we have validated the candidata bit ending, so assign it
                            bitEndsHighOrLow = candidateBitEnding;
                            bitEndingMatchCount = 0;

                            // begin normal processing
                            state = NORMAL;
                            bitErrorCount = 0;
                        }
                    }
                    else    // we had an intervening error or a non-match, start over
                    {
                        bitEndingMatchCount = 0;
                        bitErrorCount = 0;
                    }
                }
            }
            break;
        }

    case NORMAL:
        {
            // get current pin state
            boolean pinState = HW_IRQ_PORT();                 // this takes ~0.2 us
            //boolean pinState = digitalRead(interruptPin);   // this takes ~2.5 us

            // check if the current and previous half bits match and we're at the ending half bit.
            // note check of pinState is inverted since we are getting the pinState after the
            // irq that makes up the end of the bit.
            if ((isOne == lastHalfBit) && (pinState != bitEndsHighOrLow))
            {
                QueuePut(isOne);      // add the bit to the queue
                bitErrorCount = 0;    // reset error count after full valid bit
            }
            break;
        }
    }

    // save the time of the last valid interrupt and half bit
    lastInterruptTime = irqTime;
    lastHalfBit = isOne;

    // approx 7-11 us to complete, using direct port reads
}
