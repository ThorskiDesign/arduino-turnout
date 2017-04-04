//
// DCC_Decoder.cpp - Arduino library for NMRA DCC Decoding.
// Written by Kevin Snow, MynaBay.com, November, 2011. 
// Questions: dcc@mynabay.com
// Released into the public domain.
//

#include "Arduino.h"
#include "DCC_Decoder.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// NMRA DCC Definitions
//

    // Microsecond 0 & 1 timings 
#define    kONE_Min         46     // 58 +/- 12, rather than +/- 6 as per DCC spec
#define    kONE_Max         70     // due to interrupt (+/-6) and micros (+/-4) timing considerations

#define    kZERO_Min        88     // 100 +/- 12, as above
#define    kZERO_Max        10000

    // Minimum preamble length
#define    kPREAMBLE_MIN    10


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Interrupt handling, these are static
//
unsigned long          DCC_Decoder::gInterruptMicros = 0;
byte                   DCC_Decoder::gInterruptTimeIndex = 0;
volatile unsigned int  DCC_Decoder::gInterruptTime[2];
volatile unsigned int  DCC_Decoder::gInterruptChaos;

///////////////////////////////////////////////////

void DCC_Decoder::DCC_Interrupt()    // static, this is our ISR
{
#ifdef _DEBUG
    // for testing interrupt timing/consistency
    //PORTB |= (1 << 2);     // pulse output pin 10
    //PORTB &= ~(1 << 2);
#endif

    unsigned long ms = micros();
    gInterruptTime[gInterruptTimeIndex] = ms - gInterruptMicros;
    gInterruptMicros = ms;
    gInterruptChaos += gInterruptTimeIndex;
    gInterruptTimeIndex ^= 0x01;    
}

///////////////////////////////////////////////////

void DCC_Decoder::ShiftInterruptAlignment()
{
    noInterrupts();
    gInterruptTime[0] = gInterruptTime[1];
    gInterruptTimeIndex = 1;
    interrupts();
}

///////////////////////////////////////////////////

void DCC_Decoder::StartInterrupt(byte interruptPin)
{
    gInterruptTimeIndex = 0;
    gInterruptTime[0] = gInterruptTime[1] = 0;
    gInterruptChaos = 0;
    gInterruptMicros = micros();
    
    attachInterrupt( digitalPinToInterrupt(interruptPin), DCC_Interrupt, CHANGE );
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Packet Timing Support
//
unsigned long DCC_Decoder::MillisecondsSinceLastValidPacket()
{
    return millis() - gLastValidPacketMS;
}

unsigned long DCC_Decoder::MillisecondsSinceLastPacketToThisDecoder()
{
    return millis() - gLastValidPacketToAddressMS;
}

unsigned long DCC_Decoder::MillisecondsSinceLastIdlePacket()
{
    return millis() - gLastValidIdlePacketMS;
}

unsigned long DCC_Decoder::MillisecondsSinceLastResetPacket()
{
    return millis() - gLastValidResetPacketMS;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// CV Support
//

void DCC_Decoder::SetCVUpdateHandler(CVUpdateCallback func)
{
	func_CVUpdateCallback = func;
}


// read a CV
byte DCC_Decoder::ReadCV(int cv)
{
    if(cv >= kCV_PrimaryAddress && cv < kCV_MAX) return EEPROM.read(cv);
    return 0;        
}


// check if a CV is valid for writing
boolean DCC_Decoder::CVIsValidForWrite(int cv)
{
	return (cv >= kCV_PrimaryAddress && cv < kCV_MAX && cv != kCV_ManufacturerVersionNo && cv != kCV_ManufacturedID);
}


// write a CV (return true and perform callback if updated)
boolean DCC_Decoder::WriteCV(int cv, byte newValue)
{
	// if cv is not valid, just return
    if (!CVIsValidForWrite(cv)) return false;

	// is the value we're writing different from what is stored?
	byte currentValue = EEPROM.read(cv);
	if (newValue != currentValue)
	{
		EEPROM.update(cv, newValue);
		if (func_CVUpdateCallback)    // callback if we are writing a new value
			func_CVUpdateCallback(cv, currentValue, newValue);
		return true;
	}

	// return false if we didn't update anything
	return false;
}


// instance methods for CVs
byte DCC_Decoder::GetCV(int cv) { return ReadCV(cv); }
boolean DCC_Decoder::SetCV(int cv, byte newValue) { return WriteCV(cv,newValue); }


// get the address of this decoder
int DCC_Decoder::Address()
{
	int address;

	byte cv29 = ReadCV(kCV_ConfigurationData1);

	if( cv29 & 0x80 )   // Is this an accessory decoder?
	{
		address = ReadCV(kCV_AddressMSB)<<6 | ReadCV(kCV_AddressLSB);        
	}
	else
	{
		if( cv29 & 0x20 )   // Multifunction using extended addresses?
		{
			address = ReadCV(kCV_ExtendedAddress1)<<8 | ReadCV(kCV_ExtendedAddress2);        
		}
		else
		{
			address = ReadCV(kCV_PrimaryAddress);
		}
	}

	return address;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Set callback handlers
//

void DCC_Decoder::SetBaselineControlPacketHandler(BaselineControlPacket func, boolean allPackets)
{
    func_BaselineControlPacket = func;
    func_BaselineControlPacket_All_Packets = allPackets;
}


void DCC_Decoder::SetRawPacketHandler(RawPacket func)
{
    func_RawPacket = func;
}


void DCC_Decoder::SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket func, boolean allPackets)
{
    func_BasicAccPacket = func;
    func_BasicAccPacket_All_Packets = allPackets;
}

void DCC_Decoder::SetBasicAccessoryPomPacketHandler(AccDecoderPomPacket func)
{
    func_BasicAccPomPacket = func;
}

void DCC_Decoder::SetLegacyAccessoryPomPacketHandler(AccDecoderPomPacket func)
{
	func_LegacyAccPomPacket = func;
}

void DCC_Decoder::SetExtendedAccessoryDecoderPacketHandler(ExtendedAccDecoderPacket func, boolean allPackets)
{
    func_ExtdAccPacket = func;
    func_ExtdAccPacket_All_Packets = allPackets;
}

void DCC_Decoder::SetExtendedAccessoryPomPacketHandler(AccDecoderPomPacket func)
{
    func_ExtdAccPomPacket = func;
}


void DCC_Decoder::SetIdlePacketHandler(IdleResetPacket func)
{
    func_IdlePacket = func;
}


void DCC_Decoder::SetResetPacketHandler(IdleResetPacket func)
{
    func_ResetPacket = func;
}


void DCC_Decoder::SetDecodingEngineCompletionStatusHandler(DecodingEngineCompletion func)
{
    func_DecodingEngineCompletion = func;
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// State Change Macros
//
#define GOTO_DecoderReset(reason) { gState = &DCC_Decoder::State_Reset; gResetReason = reason; return; }
#define GOTO_ExecutePacket()      { gState = &DCC_Decoder::State_Execute; return; }
#define GOTO_ReadPacketState()    { gState = &DCC_Decoder::State_ReadPacket; return; }
#define GOTO_PreambleState()      { gState = &DCC_Decoder::State_ReadPreamble; return; }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Execute packet
//
void DCC_Decoder::State_Execute()
{
    ///////////////////////////////////////////////////////////
    // Test error dectection
    byte errorDectection = gPacket[0] ^ gPacket[1];             // initial xor of address and 1st instruction byte
    for (int i = 2; i < gPacketIndex - 1; i++) errorDectection ^= gPacket[i];  // xor additional instruction bytes
    if( errorDectection != gPacket[gPacketIndex-1] )
    {
        GOTO_DecoderReset( kDCC_ERR_DETECTION_FAILED );
    }
    
	////////////////////////////////////////////////////////////
	// Determine the packet type
	// loop through packet specs and check against current packet
	int i = 0;
	bool packetIdentified = false;

	while (!packetIdentified && i < gNumPacketTypes )
	{
		if ((gPacket[0] & packetSpec[i].specMask) == packetSpec[i].specAns)
			packetIdentified = true;
		i++;
	}

	// assign the packet type we identified
	if (packetIdentified)
	{
		gPacketType = packetSpec[i - 1].packetType;
	}
	else    // exit with error if we can't identify the packet
	{
		gPacketType = UNKNOWNPKT;
        GOTO_DecoderReset( kDCC_ERR_UNKNOWN_PACKET );
	}
		

	//////////////////////////////////////////////////////////////
	// now we have a valid and identified packet

	// Save off milliseconds of this valid packet
    gThisPacketMS = millis();
    gLastPacketToThisAddress = false;
    

    // Dispatch to RawPacketHandler - All packets go to raw
    // 
    // gHandledAsRawPacket cleared in Reset. If packet is handled here this flag avoids
    // sending to another dispatch routine. We don't just return here because we need to 
    // figure out packet type and update time fields.
    if( func_RawPacket )
    {
        gHandledAsRawPacket = (func_RawPacket)(gPacketIndex,gPacket);
    }


	// Process the packet depending on its type
	switch (gPacketType)
	{
	case IDLEPKT:
   		ProcessIdlePacket();
		break;
	case BROADCAST:
		ProcessBroadcastPacket();
		break;
	case LOCO_SHORT:
		ProcessShortLocoPacket();
		break;
	case LOCO_LONG:
		ProcessLongLocoPacket();
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
void DCC_Decoder::ProcessIdlePacket()
{
    if( !gHandledAsRawPacket && func_IdlePacket )
    {
        (func_IdlePacket)(gPacketIndex,gPacket);
    }

	GOTO_DecoderReset( kDCC_OK_IDLE );
}


// Process a broadcast packet
void DCC_Decoder::ProcessBroadcastPacket()
{
	//  reset packet
	if (gPacket[1] == 0x00)
	{
		if( !gHandledAsRawPacket && func_ResetPacket )
		{
			(func_ResetPacket)(gPacketIndex,gPacket);
		}
	}

	//  broadcast stop packet
	if ((gPacket[1] & 0xCE) == 0x40)
	{
		// TODO: handle broadcast stop here
	}

	// general broadcast packet
	// TODO: handle general broadcast packet here

	GOTO_DecoderReset( kDCC_OK_RESET );
}


// Process a loco packet with a short address
void DCC_Decoder::ProcessShortLocoPacket()
{
	// bits as defined in 9.2
	byte addressByte =  gPacket[0] & 0x7F;
	byte directionBit = gPacket[1] & 0x20;
	byte cBit =         gPacket[1] & 0x10;
	byte speedBits =    gPacket[1] & 0x0F;

	// Stop or estop??
	if( speedBits==0 )
	{
		speedBits = kDCC_STOP_SPEED;    
	}
	else
	{
		if( speedBits== 1 )
		{
			speedBits = kDCC_ESTOP_SPEED;
		}
		else
		{            
			if( ReadCV(kCV_ConfigurationData1) & 0x02 )  // Bit 1 of CV29: 0=14speeds, 1=28Speeds
			{
				speedBits = ((speedBits << 1 ) & (cBit ? 1 : 0)) - 3;   // speedBits = 1..28
			}
			else
			{
				speedBits -= 1;                                         // speedBits = 1..14
			}
		}
	}

	// Make callback
	gLastPacketToThisAddress = (addressByte == ReadCV(kCV_PrimaryAddress));
	if( func_BaselineControlPacket_All_Packets || gLastPacketToThisAddress )
	{
		if( !gHandledAsRawPacket && func_BaselineControlPacket )
		{
			(*func_BaselineControlPacket)(addressByte,speedBits,directionBit);
		}
	}

	GOTO_DecoderReset( kDCC_OK_BASELINE );      
}


// Process a loco packet with a long address
void DCC_Decoder::ProcessLongLocoPacket()
{
	// TODO: Implement long loco packet processing
	GOTO_DecoderReset( kDCC_OK_LONGLOCO );
}


// Process an accessory broadcast packet
void DCC_Decoder::ProcessAccBroadcastPacket()
{
	// TODO: Implement accessory broadcast packet processing
	GOTO_DecoderReset( kDCC_OK_ACCBROADCAST );
}


// Process an accessory packet
void DCC_Decoder::ProcessAccPacket()
{
	// combine packet bytes for comparison against packet specs
	unsigned long comp = (gPacket[1] << 8) | gPacket[2];

	// loop through packet specs and check against current packet to identify it
	int i = 0;
	bool packetIdentified = false;
	while (!packetIdentified && i < gNumAccPacketTypes )
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
        GOTO_DecoderReset( kDCC_ERR_UNKNOWN_PACKET );
	}


	// Ged board and output addresses
	int hiAddr = (~gPacket[1] & 0x70) << 2;
	int lowAddr = gPacket[0] & 0x3F;
	int boardAddress = (hiAddr | lowAddr) - 1;

	int outAddr = (gPacket[1] & 0x06) >> 1;
	int outputAddress = ((boardAddress << 2) | outAddr) + 1;
	if (accType == LEGACYPOM) outputAddress = (boardAddress << 2) + 1;

	gLastPacketToThisAddress = (outputAddress == Address());

	// process the packet types

	if (accType == BASIC)
	{
		if (gLastPacketToThisAddress || func_BasicAccPacket_All_Packets)
		{
			if (!gHandledAsRawPacket && func_BasicAccPacket)
			{
				// Call BasicAccHandler                             Activate bit     last data bit of packet 2
				func_BasicAccPacket(boardAddress, outputAddress, (gPacket[1] & 0x08)>>3, (gPacket[1] & 0x01));
			}
		}
		GOTO_DecoderReset( kDCC_OK_BASIC_ACCESSORY );
	}

	if (accType == EXTENDED)
	{
		if (gLastPacketToThisAddress || func_ExtdAccPacket_All_Packets)
		{
			if (!gHandledAsRawPacket && func_ExtdAccPacket)
			{
				// Call ExtAccHandler                               data bits
				func_ExtdAccPacket(boardAddress, outputAddress, gPacket[2] & 0x1F);
			}
		}
		GOTO_DecoderReset( kDCC_OK_EXTENDED_ACCESSORY );
	}

	if (accType == BASICPOM)
	{
		if (gLastPacketToThisAddress || func_BasicAccPacket_All_Packets)
		{
			if (!gHandledAsRawPacket && func_BasicAccPomPacket)
			{
				byte instType = (gPacket[2] & 0x0C)>>2;   // instruction type
				int cv = ((gPacket[2] & 0x03) << 8) + gPacket[3];   // cv 10 bit address
				byte data = gPacket[4];
				
				// Call Basic Acc Pom Handler
				func_BasicAccPomPacket(boardAddress, outputAddress, instType, cv, data);
			}
		}
		GOTO_DecoderReset( kDCC_OK_BASIC_ACCESSORY );
	}

	if (accType == EXTENDEDPOM)
	{
		if (gLastPacketToThisAddress || func_ExtdAccPacket_All_Packets)
		{
			if (!gHandledAsRawPacket && func_ExtdAccPomPacket)
			{
				byte instType = (gPacket[2] & 0x0C)>>2;   // instruction type
				int cv = ((gPacket[2] & 0x03) << 8) + gPacket[3];   // cv 10 bit address
				byte data = gPacket[4];
				
				// Call Ext Acc Pom Handler
				func_ExtdAccPomPacket(boardAddress, outputAddress, instType, cv, data);
			}
		}
		GOTO_DecoderReset( kDCC_OK_EXTENDED_ACCESSORY );
	}

	if (accType == LEGACYPOM)
	{
		if (gLastPacketToThisAddress || func_BasicAccPacket_All_Packets)
		{
			if (!gHandledAsRawPacket && func_LegacyAccPomPacket)
			{
				byte instType = 0;   // no instruction type for legacy packets
				int cv = ((gPacket[1] & 0x03) << 8) + gPacket[2];   // cv 10 bit address
				byte data = gPacket[3];
				
				// Call Legacy Acc Pom Handler
				func_LegacyAccPomPacket(boardAddress, outputAddress, instType, cv, data);
			}
		}
		GOTO_DecoderReset( kDCC_OK_LEGACY_ACCESSORY );
	}
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Standard interrupt reader - If a complete bit has been read it places timing in periodA & periodB and flows out bottom.
//
#define StandardInterruptHeader(behalfOf)                                   \
            noInterrupts();                                                 \
            if( gInterruptChaos == gLastChaos )                             \
            {                                                               \
                interrupts();                                               \
                return;                                                     \
            }                                                               \
            if( gInterruptChaos-gLastChaos > 1 )                            \
            {                                                               \
                interrupts();                                               \
                GOTO_DecoderReset( kDCC_ERR_MISSED_BITS );                  \
            }                                                               \
            unsigned int periodA = gInterruptTime[0];                       \
            unsigned int periodB = gInterruptTime[1];                       \
            gLastChaos = gInterruptChaos;                                   \
            interrupts();                                                   \
            boolean aIs1 = ( periodA >= kONE_Min && periodA <= kONE_Max );  \
            if( !aIs1 && (periodA < kZERO_Min || periodA > kZERO_Max) )     \
            {                                                               \
                GOTO_DecoderReset( kDCC_ERR_NOT_0_OR_1 );                   \
            }                                                               \
            boolean bIs1 = ( periodB >= kONE_Min && periodB <= kONE_Max );  \
            if( !bIs1 && (periodB < kZERO_Min || periodB > kZERO_Max) )     \
            {                                                               \
                GOTO_DecoderReset( kDCC_ERR_NOT_0_OR_1 );                   \
            }                                                               \

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Read packet bytes
//
void DCC_Decoder::State_ReadPacket()
{
    // Interrupt header
    StandardInterruptHeader();
    
    // Normally the two halves match. If not, reset
    if( aIs1 == bIs1 )
    {
        // 8 out of 9 times through we'll have a mask and be writing bits
        if( gPacketMask )
        {
            // Write the bit.
            if( aIs1 )
            {
                gPacket[gPacketIndex] |= gPacketMask;
            }
            // advance the bit mask
            gPacketMask = gPacketMask >> 1;
            
        }else{        
            // Getting here is the 9th time and the it's the data start bit between bytes. 
            // Zero indicates more data, 1 indicates end of packet
            
            // Advance index and reset mask
            gPacketIndex++;
            gPacketMask = 0x80;
            
            // Data start bit is a 1, that's the end of packet! Execute.
            if( aIs1 )
            {
                gPacketEndedWith1 = true;
                if( gPacketIndex>=kPACKET_LEN_MIN && gPacketIndex<=kPACKET_LEN_MAX )
                {
                    GOTO_ExecutePacket();
                }
                GOTO_DecoderReset( kDCC_ERR_INVALID_LENGTH );
            }else{
                // Data start bit is a 0. Do we have room for more data?
                if( gPacketIndex >= kPACKET_LEN_MAX )
                {
                    GOTO_DecoderReset( kDCC_ERR_MISSING_END_BIT );
                }
            }
        }
    }else{
        GOTO_DecoderReset( kDCC_ERR_NOT_0_OR_1 );
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Watch for Preamble
//
void DCC_Decoder::State_ReadPreamble()
{     
   // Interrupt header
    StandardInterruptHeader();
    
    // If we get here, booleans aIs1 and bIs1 are set to the two halves of the next bit.
    
    // If both are 1, it's a 1 bit.
    if( aIs1 && bIs1 )
    {
        // Increment preamble bit count
        ++gPreambleCount;
    }else{
        // If they equal it's a 0.
        if( aIs1 == bIs1 )
        {    
            if( gPreambleCount >= kPREAMBLE_MIN )
            { 
                // BANG! Read preamble plus trailing 0. Go read the packet.
                GOTO_ReadPacketState();
            }
        }else{
            // One is 0 the other 1. Shift alignment.
            ShiftInterruptAlignment();  
        }
        // Not enough bits in preamble or shifted alignment. Start over at zero preamble.
        gPreambleCount = 0;
    }  
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Reset handling (Part 2)
//
void DCC_Decoder::State_Reset()
{    
#ifdef _DEBUG
    // test for timing error issues - pulses pin 10 if we didn't get a 1 or 0
    if (gResetReason == kDCC_ERR_NOT_0_OR_1)
    {
       PORTB |= (1 << 2);     // pulse output pin 10
       PORTB &= ~(1 << 2);
    }
#endif
     
     // EngineReset Handler  (Debugging)
    if( func_DecodingEngineCompletion )
    {
        (func_DecodingEngineCompletion)(gHandledAsRawPacket ? kDCC_OK_MAX : gResetReason);
    }
    gHandledAsRawPacket = false;
    
        // If reset with an OK code, this was a valid packet. Save off times
    if( gResetReason < kDCC_OK_MAX )
    {
        // Save MS of last valid packet
        gLastValidPacketMS = gThisPacketMS;
        
        // Save off other times
        switch( gResetReason )
        {
            case kDCC_OK_IDLE:
                gLastValidIdlePacketMS = gThisPacketMS;
                break;
            case kDCC_OK_RESET:
                gLastValidResetPacketMS = gThisPacketMS;
                break;
            case kDCC_OK_BASELINE:
            case kDCC_OK_BASIC_ACCESSORY:
            case kDCC_OK_EXTENDED_ACCESSORY:
			case kDCC_OK_LEGACY_ACCESSORY:
			case kDCC_OK_LONGLOCO:
			case kDCC_OK_BROADCAST:
			case kDCC_OK_ACCBROADCAST:
              if(gLastPacketToThisAddress)
                {
                    gLastValidPacketToAddressMS = gThisPacketMS;
                }
                break;
            default:
                break;
        }
    }
    
        // Reset packet data
    gPacket[0] = gPacket[1] = gPacket[2] = gPacket[3] = gPacket[4] = gPacket[5] = 0;
    gPacketIndex = 0;
    gPacketMask = 0x80;
    
        // Copy last time and reset chaos
    noInterrupts();
    gPreambleCount = (gPacketEndedWith1 && gLastChaos==gInterruptChaos) ? 1 : 0;
    gLastChaos = gInterruptChaos = 0;
    interrupts();
    
        // Clear packet ended 1 flag
    gPacketEndedWith1 = false;
    
        // Go find preamble 
    GOTO_PreambleState();
}

void DCC_Decoder::State_Boot()
{   
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// SetupDecoder
//
void DCC_Decoder::SetupDecoder(byte interruptPin, byte mfgID, byte mfgVers, byte cv29)  
{
    if( gInterruptMicros == 0 )
    {        
        // Save mfg info
        WriteCV(kCV_ManufacturerVersionNo, mfgID);
        WriteCV(kCV_ManufacturedID, mfgVers);

		// store CV29 settings
        WriteCV(kCV_ConfigurationData1, cv29);

        // Attach the DCC interrupt
        StartInterrupt(interruptPin);
    
        // Start decoder in reset state
        GOTO_DecoderReset( kDCC_OK_BOOT );
    }
}

void DCC_Decoder::SetupMonitor(byte interruptPin)
{
    if( gInterruptMicros == 0 )
    {        
            // Attach the DCC interrupt
        StartInterrupt(interruptPin);
        
            // Start decoder in reset state
        GOTO_DecoderReset( kDCC_OK_BOOT );    
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Hearbeat function. Dispatch the dcc_decoder library state machine.
//
void DCC_Decoder::loop()
{
    (this->*gState)();   // call state function pointed to by gstate
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Constructor (Not really).
//
DCC_Decoder::DCC_Decoder() 
{
    gState = &DCC_Decoder::State_Boot;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Human readable error strings
//

const char PROGMEM*
DCC_Decoder::ResultString(byte resultCode)
{
    static const char PROGMEM* const gResults[] =
    {
        "OK",
        "OK - Unhandled",
        "OK - Boot",
        "OK - Idle packet",
        "OK - Reset packet",
        "OK - Handled raw",
        "OK - Handled baseline",
        "OK - Handled basic accessory",
        "OK - Handled extended accessory",
    };

    static const char PROGMEM* const gErrors[] =
    {
        "ERROR - Detection failed",
        "ERROR - Baseline address",
        "ERROR - Baseline instruction",
        "ERROR - Missed bits",
        "ERROR - Not 0 or 1",
        "ERROR - Invalid packet length",
        "ERROR - Missing packet end bits",
    };

    static const char PROGMEM* const gErrorsBadCode = "ERROR - Bad result code";

    if( resultCode>=0 && resultCode<(sizeof(gResults)/sizeof(gResults[0])) )
    {
        return gResults[resultCode];
    }
    if( resultCode>=100 && (resultCode-100)<(byte)(sizeof(gErrors)/sizeof(gErrors[0])) )
    {
        return gErrors[resultCode-100];
    }
    return gErrorsBadCode;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Helper to make packet strings
//
char* DCC_Decoder::MakePacketString(char* buffer60Bytes, byte byteCount, byte* packet)
{
    buffer60Bytes[0] = 0;
    if( byteCount>=kPACKET_LEN_MIN && byteCount<=kPACKET_LEN_MAX )
    {
        int i = 0;
        for(byte byt=0; byt<byteCount; ++byt)
        {
            byte bit=0x80;
            while(bit)
            {
                buffer60Bytes[i++] = (packet[byt] & bit) ? '1' : '0';
                bit=bit>>1;
            }
            buffer60Bytes[i++] = ' ';
        }
        buffer60Bytes[--i] = 0;
    }
    return buffer60Bytes;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Helper to return preamble length
//
int DCC_Decoder::LastPreambleBitCount()
{
    return gPreambleCount;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
