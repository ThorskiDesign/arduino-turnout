
#include "DCCdecoder.h"
#include "EEPROM.h"

// Constructors

DCCdecoder::DCCdecoder()
{
    // packet vars
    for (int i=0; i<kPACKET_LEN_MAX; i++)
        packet[i] = 0;
}


DCCdecoder::DCCdecoder(byte mfgID, byte mfgVers, byte cv29, boolean allPackets) : DCCdecoder()
{
    SetupDecoder(mfgID, mfgVers, cv29, allPackets);
}


// Set up or recondifure the decoder
void DCCdecoder::SetupDecoder(byte mfgID, byte mfgVers, byte cv29, boolean allPackets)  
{
    // Save mfg info
    SetCV(kCV_ManufacturerVersionNo, mfgID);
    SetCV(kCV_ManufacturedID, mfgVers);

    // store CV29 settings
    SetCV(kCV_ConfigurationData1, cv29);

    // return all packets, not just those for the decoder's address?
    returnAllPackets = allPackets;
}


// CV Support  =======================================================================

void DCCdecoder::SetCVUpdateHandler(CVUpdateCallback func)
{
    func_CVUpdateCallback = func;
}


// read a CV
byte DCCdecoder::GetCV(int cv)
{
    if(cv >= kCV_PrimaryAddress && cv < kCV_MAX) return EEPROM.read(cv);
    return 0;        
}


// check if a CV is valid for writing
boolean DCCdecoder::CVIsValidForWrite(int cv)
{
    return (cv >= kCV_PrimaryAddress && cv < kCV_MAX && cv != kCV_ManufacturerVersionNo && cv != kCV_ManufacturedID);
}


// write a CV (return true and perform callback if updated)
boolean DCCdecoder::SetCV(int cv, byte newValue)
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


// get the address of this decoder
int DCCdecoder::Address()
{
    int address;

    byte cv29 = GetCV(kCV_ConfigurationData1);

    if( cv29 & 0x80 )   // Is this an accessory decoder?
    {
        address = GetCV(kCV_AddressMSB)<<6 | GetCV(kCV_AddressLSB);        
    }
    else
    {
        if( cv29 & 0x20 )   // Multifunction using extended addresses?
        {
            address = GetCV(kCV_ExtendedAddress1)<<8 | GetCV(kCV_ExtendedAddress2);        
        }
        else
        {
            address = GetCV(kCV_PrimaryAddress);
        }
    }

    return address;
}


// Set callback handlers  ==========================================================

void DCCdecoder::SetBaselineControlPacketHandler(BaselineControlPacket func)
{
    func_BaselineControlPacket = func;
}


void DCCdecoder::SetBasicAccessoryDecoderPacketHandler(BasicAccDecoderPacket func)
{
    func_BasicAccPacket = func;
}


void DCCdecoder::SetBasicAccessoryPomPacketHandler(AccDecoderPomPacket func)
{
    func_BasicAccPomPacket = func;
}


void DCCdecoder::SetLegacyAccessoryPomPacketHandler(AccDecoderPomPacket func)
{
    func_LegacyAccPomPacket = func;
}


void DCCdecoder::SetExtendedAccessoryDecoderPacketHandler(ExtendedAccDecoderPacket func)
{
    func_ExtdAccPacket = func;
}


void DCCdecoder::SetExtendedAccessoryPomPacketHandler(AccDecoderPomPacket func)
{
    func_ExtdAccPomPacket = func;
}


void DCCdecoder::SetIdlePacketHandler(IdleResetPacket func)
{
    func_IdlePacket = func;
}


void DCCdecoder::SetResetPacketHandler(IdleResetPacket func)
{
    func_ResetPacket = func;
}


void DCCdecoder::SetDecodingErrorHandler(DecodingErrorHandler func)
{
    func_DecodingErrorHandler = func;
}



// Packet processing   =========================================================================

// process an incoming packet
// we assume this is a valid, checksummed packet, for example from DCCpacket class
void DCCdecoder::ProcessPacket(byte *packetData, byte size)
{
    // assign params to class vars
    packetSize = size;
    for (int i=0; i<packetSize; i++)
        packet[i] = packetData[i];

    // Determine the basic packet type - loop through packet specs and check against current packet
    int i = 0;
    bool packetIdentified = false;

    while (!packetIdentified && i < numPacketTypes )
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
        packetType = UNKNOWNPKT;
        if (func_DecodingErrorHandler)
            func_DecodingErrorHandler(kDCC_ERR_UNKNOWN_PACKET);
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
void DCCdecoder::ProcessIdlePacket()
{
    if(func_IdlePacket)
        (func_IdlePacket)(packetSize,packet);
}


// Process a broadcast packet
void DCCdecoder::ProcessBroadcastPacket()
{
    //  reset packet
    if (packet[1] == 0x00)
    {
        if(func_ResetPacket)
            (func_ResetPacket)(packetSize,packet);
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
void DCCdecoder::ProcessShortLocoPacket()
{
    // bits as defined in 9.2
    byte addressByte =  packet[0] & 0x7F;
    byte directionBit = packet[1] & 0x20;
    byte cBit =         packet[1] & 0x10;
    byte speedBits =    packet[1] & 0x0F;

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
            if( GetCV(kCV_ConfigurationData1) & 0x02 )  // Bit 1 of CV29: 0=14speeds, 1=28Speeds
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
    boolean isPacketForThisAddress = (addressByte == GetCV(kCV_PrimaryAddress));
    if (isPacketForThisAddress || returnAllPackets)
    {
        if(func_BaselineControlPacket)
            func_BaselineControlPacket(addressByte,speedBits,directionBit);
    }
}


// Process a loco packet with a long address
void DCCdecoder::ProcessLongLocoPacket()
{
    // TODO: Implement long loco packet processing
}


// Process an accessory broadcast packet
void DCCdecoder::ProcessAccBroadcastPacket()
{
    // TODO: Do we want separate callbacks for these, as opposed to returning 0 address?

    // basic acc packet
    if ((packet[1] & 0xF0) == 0x80)
    {
        if (func_BasicAccPacket)
            func_BasicAccPacket(0, 0, (packet[1] & 0x08)>>3, (packet[1] & 0x01));
        return;
    }

    // extended acc packet
    if ((packet[1] & 0xFF) == 0x07)
    {
        if (func_ExtdAccPacket)
            func_ExtdAccPacket(0, 0, packet[2] & 0x1F);
        return;
    }

    // we got here somehow with an incorrectly identified packet
    if (func_DecodingErrorHandler)
        func_DecodingErrorHandler(kDCC_ERR_UNKNOWN_PACKET);
}


// Process an accessory packet
void DCCdecoder::ProcessAccPacket()
{
    // combine packet bytes for comparison against packet specs
    unsigned long comp = (packet[1] << 8) | packet[2];

    // loop through packet specs and check against current packet to identify it
    int i = 0;
    bool packetIdentified = false;
    while (!packetIdentified && i < numAccPacketTypes )
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
        if (func_DecodingErrorHandler)
            func_DecodingErrorHandler(kDCC_ERR_UNKNOWN_PACKET);
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
    boolean isPacketForThisAddress = (outputAddress == Address());
    if (isPacketForThisAddress || returnAllPackets)
    {
        switch (accType)
        {
        case BASIC:
            if (func_BasicAccPacket)
                // Call BasicAccHandler                             Activate bit     last data bit of packet 2
                func_BasicAccPacket(boardAddress, outputAddress, (packet[1] & 0x08)>>3, (packet[1] & 0x01));
            break;
        case EXTENDED:
            if (func_ExtdAccPacket)
                // Call ExtAccHandler                               data bits
                func_ExtdAccPacket(boardAddress, outputAddress, packet[2] & 0x1F);
            break;
        case BASICPOM:
            if (func_BasicAccPomPacket)
            {
                byte instType = (packet[2] & 0x0C)>>2;   // instruction type
                int cv = ((packet[2] & 0x03) << 8) + packet[3] + 1;   // cv 10 bit address, add one for zero index
                byte data = packet[4];

                // Call Basic Acc Pom Handler
                func_BasicAccPomPacket(boardAddress, outputAddress, instType, cv, data);
            }
            break;
        case EXTENDEDPOM:
            if (func_ExtdAccPomPacket)
            {
                byte instType = (packet[2] & 0x0C)>>2;   // instruction type
                int cv = ((packet[2] & 0x03) << 8) + packet[3] + 1;   // cv 10 bit address, add one for zero index
                byte data = packet[4];

                // Call Ext Acc Pom Handler
                func_ExtdAccPomPacket(boardAddress, outputAddress, instType, cv, data);
            }
            break;
        case LEGACYPOM:
            if (func_LegacyAccPomPacket)
            {
                byte instType = 0;   // no instruction type for legacy packets
                int cv = ((packet[1] & 0x03) << 8) + packet[2] + 1;   // cv 10 bit address, add one for zero index
                byte data = packet[3];

                // Call Legacy Acc Pom Handler
                func_LegacyAccPomPacket(boardAddress, outputAddress, instType, cv, data);
            }
            break;

        }    // end switch
    }     // end if
}
