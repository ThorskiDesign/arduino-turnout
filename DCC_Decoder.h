//
// DCC_Decoder.h - Arduino library for NMRA DCC Decoding.
// Written by Kevin Snow, MynaBay.com, November, 2011. 
// Questions: dcc@mynabay.com
// Released into the public domain.
//

#ifndef __DCC_DECODER_H__
#define __DCC_DECODER_H__

#include "Arduino.h"
#include <EEPROM.h>


///////////////////////////////////////////////////////////////////////////////////////

#define kDCC_STOP_SPEED     0xFE
#define kDCC_ESTOP_SPEED    0xFF

    // Multifunction Decoders
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
#define kDCC_OK                       0
#define kDCC_OK_UNHANDLED             1
#define kDCC_OK_BOOT                  2
#define kDCC_OK_IDLE                  3
#define kDCC_OK_RESET                 4
#define kDCC_OK_RAW                   5
#define kDCC_OK_BASELINE              6 
#define kDCC_OK_BASIC_ACCESSORY       7 
#define kDCC_OK_EXTENDED_ACCESSORY    8
#define kDCC_OK_LEGACY_ACCESSORY      9
#define kDCC_OK_LONGLOCO              10
#define kDCC_OK_BROADCAST             11
#define kDCC_OK_ACCBROADCAST          12
#define kDCC_OK_MAX                   99

#define kDCC_ERR_DETECTION_FAILED     100
#define kDCC_ERR_BASELINE_ADDR        101             
#define kDCC_ERR_BASELINE_INSTR       102         // Baseline packet instruction isn't 0x01DCSSSS
#define kDCC_ERR_MISSED_BITS          103
#define kDCC_ERR_NOT_0_OR_1           104
#define kDCC_ERR_INVALID_LENGTH       105
#define kDCC_ERR_MISSING_END_BIT      106
#define kDCC_ERR_UNKNOWN_PACKET       107

    // Min and max valid packet lengths
#define kPACKET_LEN_MIN               3
#define kPACKET_LEN_MAX               6

    // CV 1..256 are supported
#define kCV_MAX                       257

///////////////////////////////////////////////////////////////////////////////////////

typedef boolean (*RawPacket)(byte byteCount, byte* packetBytes);

typedef void (*IdleResetPacket)(byte byteCount, byte* packetBytes);

typedef void (*BaselineControlPacket)(int address, int speed, int direction);

typedef void (*BasicAccDecoderPacket)(int boardAddress, int outputAddress, byte activate, byte data);
typedef void (*ExtendedAccDecoderPacket)(int boardAddress, int outputAddress, byte data);
typedef void (*AccDecoderPomPacket)(int boardAddress,int outputAddress, byte instructionType, int cv, byte data);

typedef void (*DecodingEngineCompletion)(byte resultOfLastPacket);

typedef void (*CVUpdateCallback)(int CV, byte oldValue, byte newValue);


///////////////////////////////////////////////////////////////////////////////////////

typedef void(*StateFunc)();

///////////////////////////////////////////////////////////////////////////////////////
// Packet types and identifying specs

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
const PacketSpec packetSpec[] =
{
	{ IDLEPKT,       0xFF, 0xFF  },     // 11111111
	{ LOCO_LONG,     0xC0, 0xC0  },     // 11AAAAAA    must follow IDLEPKT
	{ ACCBROADCAST,  0xFF, 0xBF  },     // 10111111
	{ ACCESSORY,     0xC0, 0x80  },     // 10AAAAAA    must follow ACCBROADCAST
	{ BROADCAST,     0xFF, 0x00  },     // 00000000
	{ LOCO_SHORT,    0x80, 0x00  },     // 0AAAAAAA    must follow BROADCAST
};

const byte gNumPacketTypes = sizeof(packetSpec)/sizeof(PacketSpec);


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
const AccPacketSpec accPacketSpec[] =
{
	{ BASICPOM,    (0x80 << 8) + 0xF0, (0x80 << 8) + 0xE0 },
	{ BASIC,       (0x80 << 8) + 0x00, (0x80 << 8) + 0x00 },   // must follow BASICPOM
	{ EXTENDEDPOM, (0x89 << 8) + 0xF0, (0x01 << 8) + 0xE0 },
	{ EXTENDED,    (0x89 << 8) + 0xE0, (0x01 << 8) + 0x00 },   // must follow EXTENDEDPOM
	{ LEGACYPOM,   (0x8C << 8) + 0x00, (0x0C << 8) + 0x00 },
};

const byte gNumAccPacketTypes = sizeof(accPacketSpec)/sizeof(AccPacketSpec);




class DCC_Decoder
{
public:
    DCC_Decoder();
    
        // Called from setup in Arduino Sketch. Set mfgID, mfgVers and interrupt. Call one SetupXXX
    void SetupDecoder(byte mfgID, byte mfgVers, byte interrupt);    // Used for Decoder
    void SetupMonitor(byte interrupt);                              // Used when building a monitor
    
        // All packets are sent to RawPacketHandler. Return true to stop dispatching to other handlers.
    void SetRawPacketHandler(RawPacket func);
    
        // S 9.2 defines two special packets. Idle and reset.
    void SetIdlePacketHandler(IdleResetPacket func);
    void SetResetPacketHandler(IdleResetPacket func);
    
        // Handler for S 9.2 baseline packets. Speed value will be 1-14, 1-28, kDCC_STOP_SPEED or kDCC_ESTOP_SPEED
    void SetBaselineControlPacketHandler(BaselineControlPacket func, boolean allPackets);
    
        // Handler for RP 9.2.1 Accessory Decoders.
    void SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket func, boolean allPackets);
	void SetBasicAccessoryPomPacketHandler(AccDecoderPomPacket func);
    void SetExtendedAccessoryDecoderPacketHandler(ExtendedAccDecoderPacket func, boolean allPackets);
	void SetExtendedAccessoryPomPacketHandler(AccDecoderPomPacket func);
	void SetLegacyAccessoryPomPacketHandler(AccDecoderPomPacket func);
                
        // Read/Write CVs
	byte GetCV(int cv);
	boolean SetCV(int cv, byte newValue);
	void SetCVUpdateHandler(CVUpdateCallback func);
    
        // Helper function to read decoder address
    int Address();
    
        // Call at least once from mainloop. Not calling frequently enough and library will miss data bits!
    void loop();
    
        // Returns the packet data in string form.
    char* MakePacketString(char* buffer60Bytes, byte packetByteCount, byte* packet);
        
        // Returns the number of bits in last preamble 
    int LastPreambleBitCount();
    
        // Timing functions. These return MS since various packets
    unsigned long MillisecondsSinceLastValidPacket();
    unsigned long MillisecondsSinceLastPacketToThisDecoder();
    unsigned long MillisecondsSinceLastIdlePacket();
    unsigned long MillisecondsSinceLastResetPacket();
    
    
    //=======================   Debugging   =======================//    
        // Everytime the DCC Decoder engine starts looking for preamble bits this will be 
        // called with result of last packet. (Debugging)
    void SetDecodingEngineCompletionStatusHandler(DecodingEngineCompletion func);
        // Converts code passed into completionStatusHandler to human readable string.
    const char PROGMEM* ResultString(byte resultCode);
    
    //======================= Library Internals =======================//
private:
        // State machine functions
    static void State_Boot();
    static void State_ReadPreamble();
    static void State_ReadPacket();
    static void State_Execute();
    static void State_Reset();
    
	// Packet processors
	static void ProcessIdlePacket();
	static void ProcessBroadcastPacket();
	static void ProcessShortLocoPacket();
	static void ProcessLongLocoPacket();
	static void ProcessAccBroadcastPacket();
	static void ProcessAccPacket();

	// Read/write CVs
    static byte ReadCV(int cv);
    static boolean WriteCV(int cv, byte data);
	static boolean CVIsValidForWrite(int cv);

        // Function pointers for the library callbacks
    static RawPacket                func_RawPacket;
    static IdleResetPacket          func_IdlePacket;
    static IdleResetPacket          func_ResetPacket;
        
    static BasicAccDecoderPacket    func_BasicAccPacket;
	static AccDecoderPomPacket      func_BasicAccPomPacket;
    static boolean                  func_BasicAccPacket_All_Packets;
    static ExtendedAccDecoderPacket func_ExtdAccPacket;
	static AccDecoderPomPacket      func_ExtdAccPomPacket;
    static boolean                  func_ExtdAccPacket_All_Packets;
	static AccDecoderPomPacket      func_LegacyAccPomPacket;
    
    static BaselineControlPacket    func_BaselineControlPacket;
    static boolean                  func_BaselineControlPacket_All_Packets;
    
    static DecodingEngineCompletion func_DecodingEngineCompletion;
	static CVUpdateCallback         func_CVUpdateCallback;
    
        // Current state function pointer
    static StateFunc                gState;                      // Current state function pointer
    
        // Timing data from last interrupt
    static unsigned int             gLastChaos;                  // Interrupt chaos count we processed
    
        // Preamble bit count
    static int                      gPreambleCount;              // Bit count for reading preamble
    
        // Reset reason 
    static byte                     gResetReason;                // Result code of last reason decoder was reset
    static boolean                  gHandledAsRawPacket;
    
        // Packet data
	static PacketType				gPacketType;                 // the packet type
    static byte                     gPacket[kPACKET_LEN_MAX];    // The packet data.
    static byte                     gPacketIndex;                // Byte index to write to.
    static byte                     gPacketMask;                 // Bit index to write to. 0x80,0x40,0x20,...0x01
    static boolean                  gPacketEndedWith1;           // Set true if packet ended on 1. Spec requires that the 
                                                                 // packet end bit can count as a bit in next preamble. 
                                                                 // CV Storage
    //static byte                     gCV[kCV_MAX];                // CV Storage (TODO - Storage in PROGMEM)
    
        // Packet arrival timing
    static unsigned long            gThisPacketMS;               // Milliseconds of this packet being parsed
    static boolean                  gLastPacketToThisAddress;    // Was last pack processed to this decoder's address?
    
    static unsigned long            gLastValidPacketMS;          // Milliseconds of last valid packet
    static unsigned long            gLastValidPacketToAddressMS; // Milliseconds of last valid packet to this decoder
    static unsigned long            gLastValidIdlePacketMS;      // Milliseconds of last valid idle packet
    static unsigned long            gLastValidResetPacketMS;     // Milliseconds of last valid reset packet

        //////////////////////////////////////////////////////
        // Interrupt Support
    static void StartInterrupt(byte interrupt);    
    static void DCC_Interrupt();
    static void ShiftInterruptAlignment();
    
    static unsigned long          gInterruptMicros;
    static byte                   gInterruptTimeIndex;
    static volatile unsigned int  gInterruptTime[2];
    static volatile unsigned int  gInterruptChaos;
};

///////////////////////////////////////////////////////////////////////////////////////

extern DCC_Decoder DCC;

///////////////////////////////////////////////////////////////////////////////////////

#endif