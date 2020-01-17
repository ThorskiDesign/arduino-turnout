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

#include "DCCdecoder.h"

// static pointer for callbacks
DCCdecoder* DCCdecoder::currentInstance = 0;

// Constructors

DCCdecoder::DCCdecoder()
{
	// pointer for callback functions
	currentInstance = this;
	
    // packet vars
	for (byte i = 0; i < PACKET_LEN_MAX; i++)
		packet[i] = 0;

	// set callbacks for the bitstream capture
	bitStream.SetDataFullHandler(WrapperBitStream);
	bitStream.SetErrorHandler(WrapperBitStreamError);

	// set callbacks for the packet builder
	dccPacket.SetPacketCompleteHandler(WrapperDCCPacket);
	dccPacket.SetPacketErrorHandler(WrapperDCCPacketError);
}

DCCdecoder::DCCdecoder(DecoderSettings settings) : DCCdecoder()
{
	decoderSettings = settings;
}

bool DCCdecoder::SetAddress(uint16_t address)
{
	decoderSettings.baseAddress = address;
	return true;
}

bool DCCdecoder::UpdateSettings(DecoderSettings settings)
{
	decoderSettings = settings;
	return true;
}


// Set callback handlers  ==========================================================

void DCCdecoder::SetBaselineControlPacketHandler(BasicControlHandler handler) { basicControlHandler = handler; }
void DCCdecoder::SetBasicAccessoryDecoderPacketHandler(BasicAccHandler handler) { basicAccHandler = handler; }
void DCCdecoder::SetBasicAccessoryPomPacketHandler(AccPomHandler handler) { basicAccPomHandler = handler; }
void DCCdecoder::SetLegacyAccessoryPomPacketHandler(AccPomHandler handler) { legacyAccPomHandler = handler; }
void DCCdecoder::SetExtendedAccessoryDecoderPacketHandler(ExtendedAccHandler handler) { extendedAccHandler = handler; }
void DCCdecoder::SetExtendedAccessoryPomPacketHandler(AccPomHandler handler) { extAccPomHandler = handler; }
void DCCdecoder::SetIdlePacketHandler(IdleResetHandler handler) { idleHandler = handler; }
void DCCdecoder::SetResetPacketHandler(IdleResetHandler handler) { resetHandler = handler; }

void DCCdecoder::SetBitstreamErrorHandler(BitstreamErrorHandler handler) { bitstreamErrorHandler = handler; }
void DCCdecoder::SetBitstreamMaxErrorHandler(BitstreamErrorHandler handler) { bitstreamMaxErrorHandler = handler; }
void DCCdecoder::SetPacketErrorHandler(PacketErrorHandler handler) { packetErrorHandler = handler; }
void DCCdecoder::SetPacketMaxErrorHandler(PacketErrorHandler handler) { packetMaxErrorHandler = handler; }
void DCCdecoder::SetDecodingErrorHandler(DecodingErrorHandler handler) { decodingErrorHandler = handler; }


// Bitstream control  ==========================================================================

void DCCdecoder::ProcessTimeStamps()
{
	// process the timestamps in the bitstream
	bitStream.ProcessTimestamps();

	// check/reset error counts
	const unsigned long currentMillis = millis();
	if (currentMillis - lastMillis > 1000)
	{
		
#ifdef _DEBUG
		Serial.print("Bit Error Count: ");
		Serial.print(bitErrorCount, DEC);
		Serial.print("     Packet Error Count: ");
		Serial.println(packetErrorCount, DEC);
#endif

		// check bit errors and raise event if necessary
		if ((bitErrorCount > maxBitErrors) && bitstreamMaxErrorHandler) bitstreamMaxErrorHandler(lastBitError);

		// if we see repeated packet errors, reset bitstream capture
		if (packetErrorCount > maxPacketErrors)
		{
			// assume we lost sync on the bitstream, reset the bitstream capture
			bitStream.Suspend();
			bitStream.Resume();

			// raise max packet error event
			if (packetMaxErrorHandler) packetMaxErrorHandler(lastPacketError);
		}

		lastMillis = currentMillis;
		bitErrorCount = 0;
		packetErrorCount = 0;
	}
}

void DCCdecoder::SuspendBitstream()
{
	bitStream.Suspend();
}

void DCCdecoder::ResumeBitstream()
{
	// reset error counts
	bitErrorCount = 0;
	packetErrorCount = 0;
	lastMillis = 0;

	bitStream.Resume();
}



// Packet processing   =========================================================================

// process an incoming packet
// we assume this is a valid, checksummed packet, for example from DCCpacket class
void DCCdecoder::ProcessPacket(byte *packetData, byte size)
{
	// assign params to class vars
    packetSize = size;
    for (byte i=0; i<packetSize; i++)
        packet[i] = packetData[i];

    // Determine the basic packet type - loop through packet specs and check against current packet
    byte i = 0;
    bool packetIdentified = false;

	while (i < numPacketTypes && !packetIdentified)
	{
		if ((packet[0] & packetSpec[i].specMask) == packetSpec[i].specAns)
			packetIdentified = true;
		i++;
	}

    // assign the packet type we identified
    if (packetIdentified)
    {
        packetType = packetSpec[i - 1].packetType;
    }
    else    // exit with error if we can't identify the packet
    {
        if (decodingErrorHandler)
            decodingErrorHandler(DCC_ERR_UNKNOWN_PACKET);
        return;
    }

    // Process the packet depending on its type
    switch (packetType)
    {
    case IDLEPKT:
        ProcessIdlePacket();
        break;
    case BROADCAST:
        ProcessBroadcastPacket();
        break;
    case LOCO_SHORT:
        //ProcessShortLocoPacket();
        break;
    case LOCO_LONG:
        //ProcessLongLocoPacket();
        break;
    case ACCBROADCAST:
        ProcessAccBroadcastPacket();
        break;
    case ACCESSORY:
        ProcessAccPacket();
        break;
    }
}



// Process an idle packet
void DCCdecoder::ProcessIdlePacket()
{
    if(idleHandler)
        (idleHandler)(packetSize,packet);
}


// Process a broadcast packet
void DCCdecoder::ProcessBroadcastPacket()
{
    //  reset packet
    if (packet[1] == 0x00)
    {
        if(resetHandler)
            (resetHandler)(packetSize,packet);
    }

    //  broadcast stop packet
    if ((packet[1] & 0xCE) == 0x40)
    {
        // TODO: handle broadcast stop here
    }

    // general broadcast packet
    // TODO: handle general broadcast packet here
}


// Process a loco packet with a short address
void DCCdecoder::ProcessShortLocoPacket() {}


// Process a loco packet with a long address
void DCCdecoder::ProcessLongLocoPacket() { }


// Process an accessory broadcast packet
void DCCdecoder::ProcessAccBroadcastPacket()
{
    // TODO: Do we want separate callbacks for these, as opposed to returning 0 address?

    // basic acc packet
    if ((packet[1] & 0xF0) == 0x80)
    {
        if (basicAccHandler)
            basicAccHandler(0, 0, (packet[1] & 0x08)>>3, (packet[1] & 0x01));
        return;
    }

    // extended acc packet
    if ((packet[1] & 0xFF) == 0x07)
    {
        if (extendedAccHandler)
            extendedAccHandler(0, 0, packet[2] & 0x1F);
        return;
    }

    // we got here somehow with an incorrectly identified packet
    if (decodingErrorHandler)
        decodingErrorHandler(DCC_ERR_UNKNOWN_PACKET);
}


// Process an accessory packet
void DCCdecoder::ProcessAccPacket()
{
    // combine packet bytes for comparison against packet specs
	const uint16_t comp = ((uint16_t)packet[1] << 8) | packet[2];

    // loop through packet specs and check against current packet to identify it
    byte i = 0;
    bool packetIdentified = false;
	while (i < numAccPacketTypes && !packetIdentified)
	{
		if ((comp & accPacketSpec[i].specMask) == accPacketSpec[i].specAns)
			packetIdentified = true;
		i++;
	}

    // assign the packet type we identified
    AccPacketType accType;
    if (packetIdentified)
    {
        accType = accPacketSpec[i - 1].accPacketType;
    }
    else    // exit with error if we can't identify the packet
    {
        if (decodingErrorHandler)
            decodingErrorHandler(DCC_ERR_UNKNOWN_PACKET);
        return;
    }

    // Ged board and output addresses
    int hiAddr = (~packet[1] & 0x70) << 2;
    int lowAddr = packet[0] & 0x3F;
    int boardAddress = (hiAddr | lowAddr) - 1;

    int outAddr = (packet[1] & 0x06) >> 1;
    int outputAddress = ((boardAddress << 2) | outAddr) + 1;
    if (accType == LEGACYPOM) outputAddress = (boardAddress << 2) + 1;

    // process the packet types
    boolean isPacketForThisAddress = (outputAddress == decoderSettings.baseAddress);   // TODO: implememnt multiple address support
    if (isPacketForThisAddress || decoderSettings.returnAllPackets)
    {
        switch (accType)
        {
        case BASIC:
            if (basicAccHandler)
                // Call BasicAccHandler                             Activate bit     last data bit of packet 2
                basicAccHandler(boardAddress, outputAddress, (packet[1] & 0x08)>>3, (packet[1] & 0x01));
            break;
        case EXTENDED:
            if (extendedAccHandler)
                // Call ExtAccHandler                               data bits
                extendedAccHandler(boardAddress, outputAddress, packet[2] & 0x1F);
            break;
        case BASICPOM:
            if (basicAccPomHandler)
            {
                byte instType = (packet[2] & 0x0C)>>2;   // instruction type
                int cv = ((packet[2] & 0x03) << 8) + packet[3] + 1;   // cv 10 bit address, add one for zero index
                byte data = packet[4];

                // Call Basic Acc Pom Handler
                basicAccPomHandler(boardAddress, outputAddress, instType, cv, data);
            }
            break;
        case EXTENDEDPOM:
            if (extAccPomHandler)
            {
                byte instType = (packet[2] & 0x0C)>>2;   // instruction type
                int cv = ((packet[2] & 0x03) << 8) + packet[3] + 1;   // cv 10 bit address, add one for zero index
                byte data = packet[4];

                // Call Ext Acc Pom Handler
                extAccPomHandler(boardAddress, outputAddress, instType, cv, data);
            }
            break;
        case LEGACYPOM:
            if (legacyAccPomHandler)
            {
                byte instType = 0;   // no instruction type for legacy packets
                int cv = ((packet[1] & 0x03) << 8) + packet[2] + 1;   // cv 10 bit address, add one for zero index
                byte data = packet[3];

                // Call Legacy Acc Pom Handler
                legacyAccPomHandler(boardAddress, outputAddress, instType, cv, data);
            }
            break;

        }    // end switch
    }     // end if
}


void DCCdecoder::BitStreamError(byte errorCode)
{
	bitErrorCount++;
	lastBitError = errorCode;
	if (bitstreamErrorHandler) bitstreamErrorHandler(errorCode);
}

void DCCdecoder::PacketError(byte errorCode)
{
	packetErrorCount++;
	lastPacketError = errorCode;
	if (packetErrorHandler) packetErrorHandler(errorCode);
}


// wrappers for callbacks in bitstream and packet objects ===================================================

// this is called from the bitstream capture when there are 32 bits to process.
void DCCdecoder::WrapperBitStream(unsigned long incomingBits)
{
	currentInstance->dccPacket.ProcessIncomingBits(incomingBits);
}

void DCCdecoder::WrapperBitStreamError(byte errorCode)
{
	currentInstance->BitStreamError(errorCode);
}


// this is called by the packet builder when a complete packet is ready, to kick off the actual decoding
void DCCdecoder::WrapperDCCPacket(byte *packetData, byte size)
{
	// kick off the packet processor
	currentInstance->ProcessPacket(packetData, size);
}

void DCCdecoder::WrapperDCCPacketError(byte errorCode)
{
	currentInstance->PacketError(errorCode);
}
