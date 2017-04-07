// DCCpacket.h

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


typedef void (*PacketCompleteHandler)(byte *Packet, byte PacketSize);
typedef void (*PacketErrorHandler)(byte ErrorCode);


class DCCpacket
{

public:
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
    State state;                   // current processing state
    byte packetIndex;              // packet byte that we're on
    byte packetMask;               // mask for assigning bits to packet bytes
    byte packet[PACKET_LEN_MAX + 1];   // packet data
    byte currentBit;               // the current bit extracted from the input stream
    byte preambleBitCount;         // count of consecutive 1's we've found while looking for preamble

    boolean enableChecksum;        // require valid checksum in order to return packet
    boolean filterRepeatPackets;   // filter out repeated packets, sending only the first in the given interval
    unsigned int filterInterval;   // time period (ms) within which packets are considered repeats
    LogPacket packetLog[MAX_PACKET_LOG_SIZE];   // history of packets to check for repeats
};


#endif
