
/*

DCC Packet Builder

A class to build and validate DCC packets from a bitstream.

Summary:

Building the DCC packets is initiated by calling the ProcessIncomingBits method, passing it a 
long int containing 32 bits from the bitstream. The bits are processed in turn, starting with
searching for the preamble, and then progressing to building the packets. After a complete 
packet is built, the Execute method is called, which performs a checksum, checks for repeat
packets, and finally performs a callback with the completed packet. Callbacks provide error
handling in the case of incorrect packet lengths or failed checksums.

Example Usage:

	DCCpacket;                              // DCCpacket object, default settings
	DCCpacket dccpacket(true,false,100);    // with checksum, repeat packet filtering, and repeat interval

Details:

In the READPREAMBLE state, the incoming bitstream is searched for a series of consecutive 1 bits.
After finding the preamble, the state changes to READPACKET. In this state, eight bits are read,
followed by checking the next bit to determine if the packet has ended. When a 1 bit is read here,
indicating the end of the packet, control passes to the Execute method.

The Execute method performs two optional checks on the packet. A checksum is performed per the
DCC spec using the last data byte. If the checksum passes, the packet is then checked to determine
if it has been repeated within a given time interval. If the packet passes both of these checks,
a callback is performed with the completed packet. After a packet is built and executed, the Reset
method resets the packet data and the state reverts to READPREAMBLE.

The IsRepeatPacket method checks for repeat packets within a certain time interval, returning true 
if a match is found. A log is maintained of recent packets. The log is updated on entry to the
method to remove packets that are outside the specified time interval. Packets that are still within
the time interval are moved toward the start of the log. This process not only removes old packets,
but also ensures that commonly received packets (e.g., idle packets) are at the front of the list.
The current packet is then checked against the list. If a match is found, the timestamp on the log
entry is updated, and the method returns true. If the packet is not found in the list, the packet
and its timestamp are added to the log, and the method returns false.

*/


#ifndef _DCCPACKET_h
#define _DCCPACKET_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


// defines for hardware debugging pulses
#define HW_DEBUG_PULSE() { PORTB = PORTB | (1 << 4); PORTB = PORTB & ~(1 << 4); }    // pulse pin 12
#define HW_DEBUG_PULSE_ON() PORTB = PORTB | (1 << 4)                                 // set pin 12 high
#define HW_DEBUG_PULSE_OFF() PORTB = PORTB & ~(1 << 4)                               // set pin 12 low

// Min and max valid packet lengths
#define PACKET_LEN_MIN              2   // zero indexed
#define PACKET_LEN_MAX              5   // zero indexed
#define PREAMBLE_MIN               10   // minimum number of 1's to signal valid preamble
#define MAX_PACKET_LOG_SIZE        25   // max number of packets to check for repeats

#define ERR_PACKET_TOO_LONG         1
#define ERR_PACKET_TOO_SHORT        2
#define ERR_FAILED_CHECKSUM         3
#define ERR_EXCEEDED_HISTORY_SIZE   4



class DCCpacket
{

public:
    typedef void (*PacketCompleteHandler)(byte *Packet, byte PacketSize);
    typedef void (*PacketErrorHandler)(byte ErrorCode);

    DCCpacket();
    DCCpacket(boolean EnableChecksum, boolean FilterRepeats, unsigned int FilterInterval);
    void ProcessIncomingBits(unsigned long incomingBits);
    void SetPacketCompleteHandler(PacketCompleteHandler Handler);
    void SetPacketErrorHandler(PacketErrorHandler Handler);
    void EnableChecksum(boolean Enable);
    void FilterRepeatPackets(boolean Filter);

private:
    // states
    enum State
    {
        READPREAMBLE,
        READPACKET,
    };

    struct LogPacket
    {
        byte packetSize;
        byte packetData[PACKET_LEN_MAX + 1];
        unsigned long packetTime;
    };

    // private functions
    void ReadPreamble();
    void ReadPacket();
    void Execute();
    void Reset();
    boolean IsRepeatPacket();

    // callback handlers
    PacketCompleteHandler packetCompleteHandler;
    PacketErrorHandler packetErrorHandler;

    // state and packet vars
    unsigned long dataBits;        // the source bit data
    State state;                   // current processing state
    byte packetIndex;              // packet byte that we're on
    byte packetMask;               // mask for assigning bits to packet bytes
    byte packet[PACKET_LEN_MAX + 1];   // packet data
    boolean currentBit;               // the current bit extracted from the input stream
    byte preambleBitCount;         // count of consecutive 1's we've found while looking for preamble

    boolean enableChecksum;        // require valid checksum in order to return packet
    boolean filterRepeatPackets;   // filter out repeated packets, sending only the first in the given interval
    unsigned int filterInterval;   // time period (ms) within which packets are considered repeats
    LogPacket packetLog[MAX_PACKET_LOG_SIZE];   // history of packets to check for repeats
};


#endif
