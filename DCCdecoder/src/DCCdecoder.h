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

DCC Decoder

A class to decode DCC packets, as defined here:
https://www.nmra.org/sites/default/files/s-92-2004-07.pdf
https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf

Summary:

This class decodes a DCC packet as described in the NMRA specs above. It determines the packet type,
then processes it to extract the address and specific packet data. Callbacks are provided for each of
the main packet types.

Example Usage:

	DCCdecoder::DecoderSettings settings =
	{
		1,      // base address
		0,      // optional addresses
		{0,0},
		true,   // return all packets
	};

	DCCdecoder dcc;                        // create an instance of the DCCdecoder
	DCCdecoder dcc { settings };

	dcc.UpdateSettings(settings);          // configure the dcc decoder

Details:

The DCCdecoder class provides the overall management of the bitstream and packet processors. The
bitstream object handles the raw bitstream capture, and provides data to the packet processor in 
intervals. The packet processor performs validity checks and provides assembled packets to the
deocder. The deocder then examines the packets to determine the packet type and its data. A decoder
address may be configured and stored so that only relevant packets are returned in the callbacks.

The ProcessTimeStamps() method should be called regularly to check for and process dcc timestamps in
the queue. Packet decoding begins when the ProcessPacket is called with packet data. The packet is
inspected to determine its type, after which specific methods are called to decode it accordingly.
Each method gets the DCC address, packet data, and any other information from the packet, and then
performs a callback to pass the decoded data back to the calling library. Packets to addresses other
than the configured address are ignored by default. Broadcast packets are returned with a value of 0
in the address field. Packet data is assumed to be a valid, checksummed packet, for example from the
DCCpacket class.

Idle, accessory, extended accessory, and broadcast packet types are supported. The basic packet type
is determined by masking bits of the packet and conparing to the expected patterns as defined in the
NMRA spec. Packet specs are ordered beginning with the most common, except where the expected bit
patterns require a certain sequence. Decoding of the packet data is handled specifically for each
packet type. Accessory packet types are further categorized in a similar manner. Basic and extended
packets are supported, as are basic program on main, extended program on main, and legacy program on
main.

TODO: The library currently only implements placeholders for locomotive functionality.

*/


#include "Bitstream.h"
#include "DCCpacket.h"


#ifndef _DCCDECODER_h
#define _DCCDECODER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


class DCCdecoder
{
public:
	// callback function typedefs
	typedef void(*IdleResetHandler)(byte byteCount, byte* packetBytes);
	typedef void(*BasicControlHandler)(int address, int speed, int direction);
	typedef void(*BasicAccHandler)(int boardAddress, int outputAddress, byte activate, byte data);
	typedef void(*ExtendedAccHandler)(int boardAddress, int outputAddress, byte data);
	typedef void(*AccPomHandler)(int boardAddress, int outputAddress, byte instructionType, int cv, byte data);

	typedef void(*BitstreamErrorHandler)(byte ErrorCode);
	typedef void(*PacketErrorHandler)(byte ErrorCode);
	typedef void(*DecodingErrorHandler)(byte ErrorCode);

	// Basic decoder setup
	struct DecoderSettings
	{
		uint16_t baseAddress;
		byte altAddresses;
		uint16_t altAddress[2];    // TODO: implement multiple decoder addresses
		bool returnAllPackets;
	};

	DCCdecoder();
	explicit DCCdecoder(DecoderSettings settings);
	bool SetAddress(uint16_t address);
	bool UpdateSettings(DecoderSettings settings);

	// decoder and bitstream control
	void ProcessTimeStamps();          // call this regularly for the bitstream object to check
									   // and process dcc timestamps in the queue
	void SuspendBitstream();
	void ResumeBitstream();

	// set packet and other event handlers
	void SetIdlePacketHandler(IdleResetHandler handler);
	void SetResetPacketHandler(IdleResetHandler handler);
	void SetBaselineControlPacketHandler(BasicControlHandler handler);
	void SetBasicAccessoryDecoderPacketHandler(BasicAccHandler handler);
	void SetBasicAccessoryPomPacketHandler(AccPomHandler handler);
	void SetExtendedAccessoryDecoderPacketHandler(ExtendedAccHandler handler);
	void SetExtendedAccessoryPomPacketHandler(AccPomHandler handler);
	void SetLegacyAccessoryPomPacketHandler(AccPomHandler handler);

	void SetBitstreamErrorHandler(BitstreamErrorHandler handler);
	void SetBitstreamMaxErrorHandler(BitstreamErrorHandler handler);
	void SetPacketErrorHandler(PacketErrorHandler handler);
	void SetPacketMaxErrorHandler(PacketErrorHandler handler);
	void SetDecodingErrorHandler(DecodingErrorHandler handler);

private:

	// decoder constants
	enum LocalConstants : uint16_t
	{
		DCC_ERR_UNKNOWN_PACKET = 101,    // DCC_Decoder results/errors
		PACKET_LEN_MIN = 3,              // Min and max valid packet lengths
		PACKET_LEN_MAX = 6,
	};

	DecoderSettings decoderSettings =
	{
		1,      // base address
		0,      // optional addresses
		{0,0},
		false,   // return all packets
	};

	// DCC packet types and identifying specs
	enum PacketType : byte
	{
		IDLEPKT,
		BROADCAST,
		LOCO_SHORT,
		LOCO_LONG,
		ACCBROADCAST,
		ACCESSORY,
	};

	struct PacketSpec
	{
		PacketType packetType;
		byte specMask;
		byte specAns;
	};

	// Order these starting with the expected most common packet types, but note dependencies
	const PacketSpec packetSpec[6] =
	{
		{ IDLEPKT,       0xFF, 0xFF  },     // 11111111
		{ LOCO_LONG,     0xC0, 0xC0  },     // 11AAAAAA    must follow IDLEPKT
		{ ACCBROADCAST,  0xFF, 0xBF  },     // 10111111
		{ ACCESSORY,     0xC0, 0x80  },     // 10AAAAAA    must follow ACCBROADCAST
		{ BROADCAST,     0xFF, 0x00  },     // 00000000
		{ LOCO_SHORT,    0x80, 0x00  },     // 0AAAAAAA    must follow BROADCAST
	};

	enum : byte { numPacketTypes = sizeof(packetSpec) / sizeof(PacketSpec) };

	// accessory packet types and specs
	enum AccPacketType : byte
	{
		BASIC,
		BASICPOM,
		EXTENDED,
		EXTENDEDPOM,
		LEGACYPOM,
	};

	struct AccPacketSpec
	{
		AccPacketType accPacketType;
		uint16_t specMask;
		uint16_t specAns;
	};

	// Order these starting with the expected most common packet types, but note dependencies
	const AccPacketSpec accPacketSpec[5] =
	{
		{ BASICPOM,    (0x80U << 8) + 0xF0, (0x80U << 8) + 0xE0 },
		{ BASIC,       (0x80U << 8) + 0x00, (0x80U << 8) + 0x00 },   // must follow BASICPOM
		{ EXTENDEDPOM, (0x89U << 8) + 0xF0, (0x01U << 8) + 0xE0 },
		{ EXTENDED,    (0x89U << 8) + 0xE0, (0x01U << 8) + 0x00 },   // must follow EXTENDEDPOM
		{ LEGACYPOM,   (0x8CU << 8) + 0x00, (0x0CU << 8) + 0x00 },
	};

	enum : byte { numAccPacketTypes = sizeof(accPacketSpec) / sizeof(AccPacketSpec) };

	// DCC bitstream and packet processors
	BitStream bitStream;
	DCCpacket dccPacket{ true, true, 250 };

	// bitstream and packet builder related
	byte bitErrorCount = 0;
	byte packetErrorCount = 0;
	enum MaxErrors : byte
	{
		maxBitErrors = 10,       // number of bit errors before indication
		maxPacketErrors = 10,     // number of packet errors before bitstream reset
	};
	unsigned long lastMillis = 0;         // for tracking refresh interval for error counts

	// process an incoming packet
	void ProcessPacket(byte *packetData, byte packetSize);

	// packet vars
	byte packet[PACKET_LEN_MAX];          // the packet bytes
	byte packetSize = 0;                   // the current packet size
	PacketType packetType = IDLEPKT;        // the packet type
	byte lastBitError;
	byte lastPacketError;

	// Packet processors
	void ProcessIdlePacket();
	void ProcessBroadcastPacket();
	void ProcessShortLocoPacket();
	void ProcessLongLocoPacket();
	void ProcessAccBroadcastPacket();
	void ProcessAccPacket();

	// Function pointers for the library callbacks
	IdleResetHandler idleHandler = 0;
	IdleResetHandler resetHandler = 0;
	BasicAccHandler	basicAccHandler = 0;
	AccPomHandler basicAccPomHandler = 0;
	ExtendedAccHandler extendedAccHandler = 0;
	AccPomHandler extAccPomHandler = 0;
	AccPomHandler legacyAccPomHandler = 0;
	BasicControlHandler basicControlHandler = 0;

	BitstreamErrorHandler bitstreamErrorHandler = 0;
	BitstreamErrorHandler bitstreamMaxErrorHandler = 0;
	PacketErrorHandler packetErrorHandler = 0;
	PacketErrorHandler packetMaxErrorHandler = 0;
	DecodingErrorHandler decodingErrorHandler = 0;

	// error handling for bitstream and packet processing
	void BitStreamError(byte errorCode);
	void PacketError(byte errorCode);

	// pointer to allow us to access member objects from callbacks
	static DCCdecoder* currentInstance;

	// callbacks for bitstream and packet builder
	static void WrapperBitStream(unsigned long incomingBits);
	static void WrapperBitStreamError(byte errorCode);
	static void WrapperDCCPacket(byte *packetData, byte size);
	static void WrapperDCCPacketError(byte errorCode);
};

#endif
