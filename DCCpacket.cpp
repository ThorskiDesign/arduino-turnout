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

#include "DCCpacket.h"


// set up the packet builder
DCCpacket::DCCpacket()
{
    packetLog[0].packetSize = 0;  // initialize packet history
    packetLog[0].packetTime = 0;
}


// set up the packet builder with specified checksum and filter settings
DCCpacket::DCCpacket(bool EnableChecksum, bool FilterRepeats, unsigned int FilterInterval) : DCCpacket()
{
    enableChecksum = EnableChecksum;         // require valid checksum in order to return packet
    filterRepeatPackets = FilterRepeats;     // filter out repeated packets, sending only the first in the given interval
    filterInterval = FilterInterval;         // time period (ms) within which packets are considered repeats
}


void DCCpacket::SetPacketCompleteHandler(PacketCompleteHandler Handler)
{
    packetCompleteHandler = Handler;
}


void DCCpacket::SetPacketErrorHandler(PacketErrorHandler Handler)
{
    packetErrorHandler = Handler;
}


void DCCpacket::EnableChecksum(bool Enable)
{
    enableChecksum = Enable;
}


void DCCpacket::FilterRepeatPackets(bool Filter)
{
    filterRepeatPackets = Filter;
}


// process an incoming sequence of 32 bits, stored in an unsigned long
void DCCpacket::ProcessIncomingBits(unsigned long incomingBits)
{
    // We get a new set of bits from the DCC bitstream about every 5ms.
    // It takes approx 120-140 us to process a set of bits, excluding callbacks and repeat filtering,
    // or 150-250 us with repeat filtering enabled.

    dataBits = incomingBits;

    // process each bit in turn
    for (unsigned long mask = 0x80000000; mask; mask >>=1)
    {
        // get the current bit
        currentBit = (dataBits & mask) ? 1 : 0;

        // process depending on the state we're in
        switch (state)
        {
        case READPREAMBLE:
            ReadPreamble();
            break;
        case READPACKET:
            ReadPacket();
            break;
        }
    }
}


// look for the packet preamble. this is a series of at least 10 1's, followed by a zero.
void DCCpacket::ReadPreamble()
{
    if (currentBit == 1)     // if it's a 1, bump our count of consecutive 1's
    {
        preambleBitCount++;
    }
    else                     // if it's a 0...
    {
        if (preambleBitCount >= PREAMBLE_MIN)   // we have the minimum number of 1's plus the trailing zero
        {
            state = READPACKET;                 // begin reading the packet
            preambleBitCount = 0;
        }
        else
        {
            preambleBitCount = 0;       // we don't have a valid preamble, start over.
        }
    }
}


// assemble the packet data from the incoming bits
void DCCpacket::ReadPacket()
{
    // assemble eight bits, decrementing the packet mask each time
    if (packetMask)
    {
        if (currentBit == 1)    // write the current bit
            packet[packetIndex] |= packetMask;

        packetMask >>= 1;      // advance the packet mask
    }

    // after 8 bits, packetMask == 0
    // zero bit here indicates more data, 1 indicates end of packet
    else
    {
        if (currentBit == 1)
        {
            if (packetIndex >= PACKET_LEN_MIN && packetIndex <= PACKET_LEN_MAX)
            {
                // we have a valid length packet with a proper ending on a 1, go
                // process it
                Execute();
            }
            else   // packet ended on a 1 but is incorrect length
            {
                if (packetErrorHandler && (packetIndex < PACKET_LEN_MIN)) packetErrorHandler (ERR_PACKET_TOO_SHORT);
                if (packetErrorHandler && (packetIndex > PACKET_LEN_MIN)) packetErrorHandler (ERR_PACKET_TOO_LONG);
                Reset();
            }
        }
        else   // zero bit indicates more data
        {
            // advance to the next packet and reset the mask
            packetIndex++;
            packetMask = 0x80;

            // if packet index is too high, reset
            if (packetIndex > PACKET_LEN_MAX)
            {
                if (packetErrorHandler)
                    packetErrorHandler(ERR_PACKET_TOO_LONG);
                Reset();
            }
        }
    }
}


// verify the checksum of the completed packet, check for repeat packets,
// and then perform the callback to process it
void DCCpacket::Execute()
{
    // initialize as true so we can just skip checksum if disabled
    bool checksumOk = true;

    // verify checksum if enabled
    if (enableChecksum)
    {
        byte errorDectection = packet[0] ^ packet[1];             // initial xor of address and 1st instruction byte
        for (int i = 2; i < packetIndex; i++) 
            errorDectection ^= packet[i];                         // xor additional instruction bytes
        checksumOk = (errorDectection == packet[packetIndex]);
    }

    // if we pass the checksum
    if(checksumOk)
    {
        // if check for repeats is enabled, and it's a repeat packet, skip the callback
        if (!(filterRepeatPackets && IsRepeatPacket()))
        {
            // execute callback for complete valid packet
            if (packetCompleteHandler)
                packetCompleteHandler(packet, packetIndex + 1);   // return the size of the packet, not the final index
        }
    }
    else   // check sum error
    {
        if (packetErrorHandler)
            packetErrorHandler(ERR_FAILED_CHECKSUM);
    }

    // reset and start looking for preamble again.
    Reset();
}


// reset packet and counter data, and start looking for next preamble.
void DCCpacket::Reset()
{
    // Reset packet data
    packet[0] = packet[1] = packet[2] = packet[3] = packet[4] = packet[5] = 0;
    packetIndex = 0;
    packetMask = 0x80;

    // start looking for preamble again
    state = READPREAMBLE;
}


// check for repeat packets within a certain time interval. returns true if a match is found.
// updating of the packet history removes packets that are outside the time interval, and ensures that
// the most common packets are at the front of the list
bool DCCpacket::IsRepeatPacket()
{
    // 30-60 us to process this function

    const unsigned long currentMillis = millis();

    // remove packets that have timed out and compact history
    byte newEntryCount = 0;
    byte oldEntryCount = 0;
    while (packetLog[oldEntryCount].packetSize > 0)
    {
        // if packet is still within the time interval, add it to compacted history
        if (currentMillis - packetLog[oldEntryCount].packetTime < filterInterval)
        {
            if (newEntryCount != oldEntryCount)   // avoid overhead of copying to same location
                packetLog[newEntryCount] = packetLog[oldEntryCount];
            newEntryCount++;
        }
        oldEntryCount++;
    }

    // set size of next entry to zero to flag end of history data
    packetLog[newEntryCount].packetSize = 0;

    // now check current packet against packet history
    newEntryCount = 0;
    while (packetLog[newEntryCount].packetSize > 0)
    {
        // check if packet matches
        if (packetLog[newEntryCount].packetSize == packetIndex + 1)    // check size first, that's quick and easy
        {
            // now check each byte in turn
            bool matchFound = true;
            for (int i=0; i < packetIndex + 1; i++)
                if (packetLog[newEntryCount].packetData[i] != packet[i])
                    matchFound = false;

            // if packet matches, update timestamp on matching log entry and return true
            if (matchFound)
            {
                packetLog[newEntryCount].packetTime = currentMillis;
                return true;
            }
        }

        newEntryCount++;    // continue checking with next history entry
    }

    // packet doesn't match any entries, so it is a new packet.

    // check number of entries in history log
    if (newEntryCount < MAX_PACKET_LOG_SIZE)
    {
        // add the packet to the history log
        packetLog[newEntryCount].packetSize = packetIndex + 1;
        packetLog[newEntryCount].packetTime = currentMillis;
        for (int i=0; i < packetIndex + 1; i++)
            packetLog[newEntryCount].packetData[i] = packet[i];
    }
    else   // raise error for exceeding max history size
    {
        if (packetErrorHandler)
            packetErrorHandler(ERR_EXCEEDED_HISTORY_SIZE);
    }

    return false;
}
