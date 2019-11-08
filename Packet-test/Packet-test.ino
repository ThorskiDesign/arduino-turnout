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
#include "Bitstream.h"


typedef struct
{
	int count;
	byte validBytes;
	byte data[6];
} DCCPacket;

// The dcc decoder object and global data
//
int gPacketCount = 0;
int gIdlePacketCount = 0;
int gLongestPreamble = 0;
int gErrorCount = 0;
int gErrorLog[25];
int gBitErrorCount = 0;
int gBitErrorLog[25];

DCCPacket gPackets[25];

static unsigned long lastMillis = millis();


// Bitstream setup ==========================================================================

// global stuff
BitStream bitStream;
DCCpacket dccpacket(true, true, 250);


void BitStreamHandler(unsigned long incomingBits)
{
	dccpacket.ProcessIncomingBits(incomingBits);
}


void BitErrorHandler(byte errorCode)
{
	if (gBitErrorCount < 25)
		gBitErrorLog[gBitErrorCount] = errorCode;
	gBitErrorCount++;
	//Serial.println(errorCode, DEC);
}



// dcc packet builder setup ============================================================

void PacketErrorHandler(byte errorCode)
{
	if (gErrorCount < 25)
		gErrorLog[gErrorCount] = errorCode;
	gErrorCount++;
}

void RawPacketHandler(byte *packetBytes, byte byteCount)
{
	// Bump global packet count
	++gPacketCount;

	// Walk table and look for a matching packet
	for (int i = 0; i < (int)(sizeof(gPackets) / sizeof(gPackets[0])); ++i)
	{
		if (gPackets[i].validBytes)
		{
			// Not an empty slot. Does this slot match this packet? If so, bump count.
			if (gPackets[i].validBytes == byteCount)
			{
				char isPacket = true;
				for (int j = 0; j < byteCount; j++)
				{
					if (gPackets[i].data[j] != packetBytes[j])
					{
						isPacket = false;
						break;
					}
				}
				if (isPacket)
				{
					gPackets[i].count++;
					return;
				}
			}
		}
		else
		{
			// Empty slot, just copy over data
			gPackets[i].count++;
			gPackets[i].validBytes = byteCount;
			for (int j = 0; j < byteCount; j++)
			{
				gPackets[i].data[j] = packetBytes[j];
			}
			return;
		}
	}
}



// Min and max valid packet lengths
#define kPACKET_LEN_MIN               3
#define kPACKET_LEN_MAX               6

// Helper to make packet strings
char* MakePacketString(char* buffer60Bytes, byte byteCount, byte* packet)
{
	buffer60Bytes[0] = 0;
	if (byteCount >= kPACKET_LEN_MIN && byteCount <= kPACKET_LEN_MAX)
	{
		int i = 0;
		for (byte byt = 0; byt < byteCount; ++byt)
		{
			byte bit = 0x80;
			while (bit)
			{
				buffer60Bytes[i++] = (packet[byt] & bit) ? '1' : '0';
				bit = bit >> 1;
			}
			buffer60Bytes[i++] = ' ';
		}
		buffer60Bytes[--i] = 0;
	}
	return buffer60Bytes;
}



void DumpAndResetTable()
{
	char buffer60Bytes[60];

	Serial.print("Total Packet Count: ");
	Serial.println(gPacketCount, DEC);

	Serial.print("Idle Packet Count:  ");
	Serial.println(gIdlePacketCount, DEC);

	Serial.print("Bit Error Count:  ");
	Serial.println(gBitErrorCount, DEC);

	Serial.print("Bit Errors:  ");
	int numErrors = (gBitErrorCount < 25) ? gBitErrorCount : 25;
	for (int i = 0; i < numErrors; i++)
	{
		Serial.print(gBitErrorLog[i], DEC);
		Serial.print(" ");
	}
	Serial.println();

	Serial.print("Packet Error Count:  ");
	Serial.println(gErrorCount, DEC);

	Serial.print("Packet Errors:  ");
	numErrors = (gErrorCount < 25) ? gErrorCount : 25;
	for (int i = 0; i < numErrors; i++)
	{
		Serial.print(gErrorLog[i], DEC);
		Serial.print(" ");
	}
	Serial.println();

	Serial.println("Count    Packet_Data");
	for (int i = 0; i < (int)(sizeof(gPackets) / sizeof(gPackets[0])); ++i)
	{
		if (gPackets[i].validBytes > 0)
		{
			Serial.print(gPackets[i].count, DEC);
			if (gPackets[i].count < 10)
			{
				Serial.print("        ");
			}
			else {
				if (gPackets[i].count < 100)
				{
					Serial.print("       ");
				}
				else {
					Serial.print("      ");
				}
			}
			Serial.println(MakePacketString(buffer60Bytes, gPackets[i].validBytes, &gPackets[i].data[0]));
		}
		gPackets[i].validBytes = 0;
		gPackets[i].count = 0;
	}
	Serial.println("============================================");

	gPacketCount = 0;
	gIdlePacketCount = 0;
	gLongestPreamble = 0;
	gErrorCount = 0;
	gBitErrorCount = 0;
}



// Setup  =================================================================
//
void setup()
{
	// for testing timing
	pinMode(18, OUTPUT);
	pinMode(19, OUTPUT);

	Serial.begin(115200);

	bitStream.SetDataFullHandler(&BitStreamHandler);
	bitStream.SetErrorHandler(&BitErrorHandler);

	dccpacket.SetPacketCompleteHandler(&RawPacketHandler);
	dccpacket.SetPacketErrorHandler(&PacketErrorHandler);

	bitStream.Resume();    // start the bitstream capture
}



// Main loop   =======================================================================
volatile double y = 0;
void loop()
{
	// add a delay doing some processing...  
	// approx 500 us, puts ~10 timestamps in the queue
	//HW_DEBUG_PULSE_19_ON();
	//double x;
	//for (unsigned int i = 0; i < 4; i++)
	//{
	//	x += sin(i);
	//}
	//y = x;
	//HW_DEBUG_PULSE_19_OFF();

	bitStream.ProcessTimestamps();

	unsigned long currentMillis = millis();

	if (currentMillis - lastMillis > 2000)
	{
		bitStream.Suspend();
		DumpAndResetTable();
		lastMillis = millis();
		bitStream.Resume();
	}
}
