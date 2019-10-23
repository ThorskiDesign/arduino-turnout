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

	DCCpacket dccpacket;                            // DCCpacket object, default settings
	DCCpacket dccpacket(true,false,100);            // with checksum, repeat packet filtering, and repeat interval
	dccpacket.ProcessIncomingBits(incomingBits);    // process 32 bits of bitstream data

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
    DCCpacket(bool EnableChecksum, bool FilterRepeats, unsigned int FilterInterval);
    void ProcessIncomingBits(unsigned long incomingBits);
    void SetPacketCompleteHandler(PacketCompleteHandler Handler);
    void SetPacketErrorHandler(PacketErrorHandler Handler);
    void EnableChecksum(bool Enable);
    void FilterRepeatPackets(bool Filter);

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
	bool IsRepeatPacket();

    // callback handlers
    PacketCompleteHandler packetCompleteHandler = 0;
    PacketErrorHandler packetErrorHandler = 0;

    // state and packet vars
    unsigned long dataBits = 0;         // the source bit data
    State state = READPREAMBLE;         // current processing state
    byte packetIndex = 0;               // packet byte that we're on
    byte packetMask = 0x80;             // mask for assigning bits to packet bytes
    byte packet[PACKET_LEN_MAX + 1];    // packet data
	bool currentBit = 0;                // the current bit extracted from the input stream
    byte preambleBitCount = 0;          // count of consecutive 1's we've found while looking for preamble

    bool enableChecksum = true;                // require valid checksum in order to return packet
    bool filterRepeatPackets = true;           // filter out repeated packets, sending only the first in the given interval
    unsigned int filterInterval = 250;         // time period (ms) within which packets are considered repeats
	LogPacket packetLog[MAX_PACKET_LOG_SIZE];  // history of packets to check for repeats
};


#endif
