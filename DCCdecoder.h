
/*

DCC Decoder

A class to decode DCC packets, as defined here:
https://www.nmra.org/sites/default/files/s-92-2004-07.pdf
https://www.nmra.org/sites/default/files/s-9.2.1_2012_07.pdf

Summary:

This class decodes a DCC packet as described in the NMRA specs above. It determines the packet type,
then processes it to extract the address and specific packet data. Callbacks are provided for each of
the main packet types. CV support is provided via the EEPROM library, allowing for setting a decoder
address and for configuration changes.

Example Usage:

	DCCdecoder dcc();                        // create an instance of the DCCdecoder
	dcc.SetupDecoder(0, 0, cv29, false);     // configure the dcc decoder
	dcc.ProcessPacket(packetData, size);     // send packet data to be decoded

Details:

Packet decoding begins when the ProcessPacket is called with packet data. The packet is inspected to 
determine its type, after which specific methods are called to decode it accordingly. Each method
gets the DCC address, packet data, and any other information from the packet, and then performs a
callback to pass the decoded data back to the calling library. Packets to addresses other than the
configured address are ignored by default. Broadcast packets are returned with a value of 0 in the
address field. Packet data is assumed to be a valid, checksummed packet, for example from the 
DCCpacket class.

Idle, locomotive (short and long), accessory, broadcast (loco and accessory) packet types are supported.
The basic packet type is determined by masking bits of the packet and conparing to the expected 
patterns as defined in the NMRA spec. Packet specs are ordered beginning with the most common, except
where the expected bit patterns require a certain sequence. Decoding of the packet data is handled 
specifically for each packet type. Accessory packet types are further categorized in a similar manner. 
Basic and extended packets are supported, as are basic program on main, extended program on main, and 
legacy program on main.

Reading and writing of CVs to non-volatile storage is provided via the EEPROM library. A decoder
address may be configured and stored so that only relevant packets are returned in the callbacks.

TODO: The library currently only implements the most basic locomotive functionality.

*/



#ifndef _DCCDECODER_h
#define _DCCDECODER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


// (from MynaBay DCC_Decoder.h)
// Multifunction Decoders
#define kDCC_STOP_SPEED            0xFE
#define kDCC_ESTOP_SPEED           0xFF
#define kCV_PrimaryAddress            1
#define kCV_Vstart                    2
#define kCV_AccelerationRate          3
#define kCV_Deceleration Rate         4
#define kCV_ManufacturerVersionNo     7 
#define kCV_ManufacturedID            8
#define kCV_ExtendedAddress1          17
#define kCV_ExtendedAddress2          18
#define kCV_ConfigurationData1        29

// Accessory Decoders
#define kCV_AddressLSB                1
#define kCV_AddressMSB                9

// DCC_Decoder results/errors
#define kDCC_ERR_UNKNOWN_PACKET       101

// Min and max valid packet lengths
#define kPACKET_LEN_MIN               3
#define kPACKET_LEN_MAX               6

// CV 1..256 are supported
#define kCV_MAX                       257



class DCCdecoder
{
public:
    // CV29 bits  (from NmraDcc.h)
    enum CV29bits {
        CV29_LOCO_DIR            = 0x01, // bit 0: Locomotive Direction: "0" = normal, "1" = reversed
        CV29_F0_LOCATION         = 0x02, // bit 1: F0 location: "0" = bit 4 in Speed and Direction instructions, 
                                         //        "1" = bit 4 in function group one instruction
        CV29_APS				 = 0x04, // bit 2: Alternate Power Source (APS) "0" = NMRA Digital only, 
                                         //        "1" = Alternate power source set by CV12
        CV29_ADV_ACK             = 0x08, // bit 3: ACK, Advanced Acknowledge mode enabled if 1, disabled if 0
        CV29_SPEED_TABLE_ENABLE  = 0x10, // bit 4: STE, Speed Table Enable, "0" = values in CVs 2, 4 and 6, 
                                         //        "1" = Custom table selected by CV 25
        CV29_EXT_ADDRESSING      = 0x20, // bit 5: "0" = one byte addressing, "1" = two byte addressing
        CV29_OUTPUT_ADDRESS_MODE = 0x40, // bit 6: "0" = Decoder Address Mode, "1" = Output Address Mode
        CV29_ACCESSORY_DECODER   = 0x80, // bit 7: "0" = Multi-Function Decoder Mode, "1" = Accessory Decoder Mode
    };

    // callback function typedefs
    typedef void (*IdleResetPacket)(byte byteCount, byte* packetBytes);
    typedef void (*BaselineControlPacket)(int address, int speed, int direction);
    typedef void (*BasicAccDecoderPacket)(int boardAddress, int outputAddress, byte activate, byte data);
    typedef void (*ExtendedAccDecoderPacket)(int boardAddress, int outputAddress, byte data);
    typedef void (*AccDecoderPomPacket)(int boardAddress,int outputAddress, byte instructionType, int cv, byte data);
    typedef void (*CVUpdateCallback)(int CV, byte oldValue, byte newValue);
    typedef void (*DecodingErrorHandler)(byte ErrorCode);

    // Basic decoder setup (manufacturer ID, CV29 config, all packets option)
    DCCdecoder();
    DCCdecoder(byte mfgID, byte mfgVers, byte cv29, boolean allPackets);
    void SetupDecoder(byte mfgID, byte mfgVers, byte cv29, boolean allPackets);

    // process an incoming packet
    void ProcessPacket(byte *packetData, byte packetSize);

    // S 9.2 defines two special packets. Idle and reset.
    void SetIdlePacketHandler(IdleResetPacket func);
    void SetResetPacketHandler(IdleResetPacket func);

    // Handler for S 9.2 baseline packets. Speed value will be 1-14, 1-28, kDCC_STOP_SPEED or kDCC_ESTOP_SPEED
    void SetBaselineControlPacketHandler(BaselineControlPacket func);

    // Handler for RP 9.2.1 Accessory Decoders.
    void SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket func);
    void SetBasicAccessoryPomPacketHandler(AccDecoderPomPacket func);
    void SetExtendedAccessoryDecoderPacketHandler(ExtendedAccDecoderPacket func);
    void SetExtendedAccessoryPomPacketHandler(AccDecoderPomPacket func);
    void SetLegacyAccessoryPomPacketHandler(AccDecoderPomPacket func);

    // Error handler
    void SetDecodingErrorHandler(DecodingErrorHandler func);

    // Read/Write CVs
    byte GetCV(int cv);
    boolean SetCV(int cv, byte newValue);
    void SetCVUpdateHandler(CVUpdateCallback func);
    boolean CVIsValidForWrite(int cv);

    // Helper function to read decoder address
    int Address();

private:

    // DCC packet types and identifying specs
    enum PacketType
    {
        UNKNOWNPKT,
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
        unsigned long specMask;
        unsigned long specAns;
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

    const byte numPacketTypes = sizeof(packetSpec)/sizeof(PacketSpec);

    // accessory packet types and specs
    enum AccPacketType
    {
        UNKNOWNACC,
        BASIC,
        BASICPOM,
        EXTENDED,
        EXTENDEDPOM,
        LEGACYPOM,
    };

    struct AccPacketSpec
    {
        AccPacketType accPacketType;
        unsigned long specMask;
        unsigned long specAns;
    };

    // Order these starting with the expected most common packet types, but note dependencies
    const AccPacketSpec accPacketSpec[5] =
    {
        { BASICPOM,    (0x80UL << 8) + 0xF0, (0x80UL << 8) + 0xE0 },
        { BASIC,       (0x80UL << 8) + 0x00, (0x80UL << 8) + 0x00 },   // must follow BASICPOM
        { EXTENDEDPOM, (0x89UL << 8) + 0xF0, (0x01UL << 8) + 0xE0 },
        { EXTENDED,    (0x89UL << 8) + 0xE0, (0x01UL << 8) + 0x00 },   // must follow EXTENDEDPOM
        { LEGACYPOM,   (0x8CUL << 8) + 0x00, (0x0CUL << 8) + 0x00 },
    };

    const byte numAccPacketTypes = sizeof(accPacketSpec)/sizeof(AccPacketSpec);

    // packet vars
    byte packet[kPACKET_LEN_MAX];          // the packet bytes
    byte packetSize = 0;                   // the current packet size
    PacketType packetType = UNKNOWNPKT;    // the packet type
    boolean returnAllPackets = false;      // return all packets, not just the ones for the decoder's address

    // Packet processors
    void ProcessIdlePacket();
    void ProcessBroadcastPacket();
    void ProcessShortLocoPacket();
    void ProcessLongLocoPacket();
    void ProcessAccBroadcastPacket();
    void ProcessAccPacket();

    // Function pointers for the library callbacks
    IdleResetPacket          func_IdlePacket = 0;
    IdleResetPacket          func_ResetPacket = 0;
    BasicAccDecoderPacket    func_BasicAccPacket = 0;
    AccDecoderPomPacket      func_BasicAccPomPacket = 0;
    ExtendedAccDecoderPacket func_ExtdAccPacket = 0;
    AccDecoderPomPacket      func_ExtdAccPomPacket = 0;
    AccDecoderPomPacket      func_LegacyAccPomPacket = 0;
    BaselineControlPacket    func_BaselineControlPacket = 0;

    DecodingErrorHandler     func_DecodingErrorHandler = 0;
    CVUpdateCallback         func_CVUpdateCallback = 0;
};

#endif

