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


// Bitstream setup ==========================================================================

DCCdecoder dcc;


void BitErrorHandler(byte errorCode)
{
    //Serial.print("Bit error, code: ");
    //Serial.println(errorCode,DEC);

	digitalWrite(6, HIGH); delay(100); digitalWrite(6, LOW);
}


void PacketErrorHandler(byte errorCode)
{
    Serial.print("Packet error, code: ");
    Serial.println(errorCode,DEC);
}


void DCC_AccessoryDecoderHandler(int boardAddress, int outputAddress, byte activate, byte data)
{
    Serial.print("Basic Acc Packet, Board Address: ");
    Serial.print(boardAddress, DEC);
    Serial.print("  Output Address: ");
    Serial.print(outputAddress, DEC);
    Serial.print("  Activate Bit: ");
    Serial.print(activate,DEC);
    Serial.print("  Data: ");
    Serial.println(data,DEC);
}


void DCC_ExtendedAccDecoderHandler(int boardAddress, int outputAddress, byte data)
{
    Serial.print("Ext Acc Packet, Board Address: ");
    Serial.print(boardAddress, DEC);
    Serial.print("  Output Address: ");
    Serial.print(outputAddress, DEC);
    Serial.print("  Data: ");
    Serial.println(data,DEC);
}


void DCC_BaselineControlHandler(int address, int speed, int direction)
{
    Serial.print("Baseline Packet, Loco Address: ");
    Serial.print(address, DEC);
    Serial.print("  Speed: ");
    Serial.print(speed, DEC);
    Serial.print("  Direction: ");
    Serial.println(direction,DEC);
}


void DCC_AccPomHandler(int boardAddress,int outputAddress, byte instructionType, int cv, byte data)
{
    Serial.print("Basic Acc POM Packet, Board Address: ");
    Serial.print(boardAddress, DEC);
    Serial.print("  Output Address: ");
    Serial.print(outputAddress, DEC);
    Serial.print("  Instruction Type: ");
    Serial.print(instructionType,DEC);
    Serial.print("  CV: ");
    Serial.print(cv,DEC);
    Serial.print("  Data: ");
    Serial.println(data,DEC);
}



// Setup  =================================================================
//
void setup()
{
	//pinMode(0, OUTPUT);
	//pinMode(1, OUTPUT);

	pinMode(6, OUTPUT);
	pinMode(18, OUTPUT);
	pinMode(19, OUTPUT);

	// verify led output correct
	digitalWrite(6, HIGH);
	delay(1000);
	digitalWrite(6, LOW);
	
	Serial.begin(115200);

    dcc.SetupDecoder(0,0,0,true);
    dcc.SetBasicAccessoryDecoderPacketHandler(&DCC_AccessoryDecoderHandler);
    dcc.SetExtendedAccessoryDecoderPacketHandler(&DCC_ExtendedAccDecoderHandler);
    dcc.SetBaselineControlPacketHandler(&DCC_BaselineControlHandler);
    dcc.SetBasicAccessoryPomPacketHandler(&DCC_AccPomHandler);

	//dcc.SetBitstreamErrorHandler(BitErrorHandler);
	//dcc.SetPacketErrorHandler(PacketErrorHandler);
	dcc.SetBitstreamMaxErrorHandler(BitErrorHandler);
	//dcc.SetPacketMaxErrorHandler(PacketErrorHandler);
	
	Serial.println("dcc decoder setup complete.");

	dcc.ResumeBitstream();
	Serial.println("bitstream capture started.");
}



// Main loop   =======================================================================

void loop()
{
	dcc.ProcessTimeStamps();
}
