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

TurnoutBase

A class providing common funtionality for the TurnoutMgr and XoverMgr classes.

Summary:

The TurnoutBase class provides common funtionality for the TurnoutMgr and XoverMgr classes. It handles 
the processing of received DCC commands, controls the LED indications, controls the aux outputs, and 
monitors the pushbutton. It contains the hardware pin assignments for all the I/O, as well as the
definition and defaults for the CVs. A reset to default may be performed by holding the pushbutton
while the hardware is powered up. Options to swap the interpretation of the DCC command, the occupancy 
sensors, and the relays are provided. Two auxiliary outputs are controllable using extended accessory
(signal aspect) commands.

Details:

DCC command processing takes place as follows. The raw bitstream is captured by the BitStream
object. When 32 bits have been captured, a callback to the packet decoder begins the assembly
of the DCC packet. After the packet decoder has assembled and checksummed a complete packet, a
callback to the DCCdecoder initiates processing of the packet. Callbacks from the DCCdecoder
trigger actions for normal accessory decoder packets, extended accessory decoder packets, and 
programming on main packets.

The InitMain method performs the setup for the class, including setting up the DCC packet 
processor, reading the stored configuration from EEPROM (via the DCCdecoder lib), and getting 
the stored position of the turnout.

The Update method processes timestamps received by the BitStream object, which then sends them 
to the DCCpacket object to be assembled into a full DCC packet. It also handles millis-related 
updates for the LED, sensors, and timers. Packet error count per second is also checked. If it 
exceeds a configurable max value, a reset of the bitstream object takes place.

The DCCExtCommandHandler processes an extended accessory command, using signal aspects for turning 
the two auxilliary outputs on and off. It also provides the capability to toggle error indication on
and off. The DCCPomHandler method processes a program on main packet. It checks for a valid CV, 
stores the data via the DCCdecoder object, and then re-reads the basic configuration for the turnout. 
It also provides complete and partial reset via POM commands.

*/

#ifndef _TURNOUTBASE_h
#define _TURNOUTBASE_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Bitstream.h"
#include "DCCpacket.h"
#include "DCCdecoder.h"
#include "RGB_LED.h"
#include "Button.h"
#include "OutputPin.h"
#include "EventTimer.h"


class TurnoutBase
{
protected:
	TurnoutBase();

	// Hardware assignments
	const byte Aux1Pin = 0;
	const byte Aux2Pin = 1;
	//const byte HWirqPin = 2;       set in bitstream.h
	const byte ButtonPin = 3;
	const byte ServoPowerPin = 4;
	const byte Servo1Pin = 5;
	const byte LedBPin = 6;
	const byte LedRPin = 7;
	//const byte ICRPin = 8;         set in bitstream.h
	const byte Servo2Pin = 9;
	const byte Servo3Pin = 10;
	const byte Servo4Pin = 11;
	const byte LedGPin = 12;
	const byte Relay1Pin = 14;
	const byte Relay2Pin = 15;
	const byte Sensor1Pin = 16;
	const byte Sensor2Pin = 17;
	const byte Relay3Pin = 18;
	const byte Relay4Pin = 19;

	// main functions
	void InitMain();
	void Update();
	void FactoryReset(bool HardReset);

	// Sensors and outputs
	Button button{ ButtonPin, true };
	RgbLed led{ LedRPin, LedGPin, LedBPin };
	OutputPin servoPower{ ServoPowerPin };
	OutputPin auxOutput1{ Aux1Pin };
	OutputPin auxOutput2{ Aux2Pin };
	EventTimer resetTimer;
	EventTimer errorTimer;
	EventTimer servoTimer;

	// DCC bitstream and packet processors
	BitStream bitStream;
	DCCpacket dccPacket{ true, true, 250 };
	DCCdecoder dcc;

	// bitstream and packet builder related
	unsigned long bitErrorCount = 0;
	unsigned long packetErrorCount = 0;
	const byte maxBitErrors = 10;         // number of bit errors before indication
	const byte maxPacketErrors = 10;      // number of packet errors before bitstream reset
	unsigned long lastMillis = 0;         // for tracking refresh interval for error counts

	// other instance variables
	enum State { STRAIGHT, CURVED };
	byte dccAddress = 1;                       // the dcc address of the decoder
	State position = STRAIGHT;                 // the current or commanded position of the switch
	bool occupancySensorSwap = false;          // optionally swap the straight/curved occupancy sensors
	bool dccCommandSwap = false;               // optionally swap the meaning of received dcc commands
	bool relaySwap = false;					   // optionally swap the straight/curved relays
	bool factoryReset = false;                 // is a reset in progress
	bool showErrorIndication = false;           // enable or disable LED error indications
	bool servosActive = false;                 // flag to indicate if servos are active or not
	byte currentServo = 0;                     // the servo that is currently in motion
	bool servoRate = LOW;                      // rate at which the servos will be set

	// define our available cv's  (allowable range 33-81 per 9.2.2)
	const byte CV_AddressLSB = 1;
	const byte CV_AddressMSB = 9;
	const byte CV_servo1MinTravel = 33;
	const byte CV_servo1MaxTravel = 34;
	const byte CV_servoLowSpeed = 35;
	const byte CV_servoHighSpeed = 36;
	const byte CV_occupancySensorSwap = 38;
	const byte CV_dccCommandSwap = 39;
	const byte CV_relaySwap = 40;
	const byte CV_Aux1Off = 41;
	const byte CV_Aux1On = 42;
	const byte CV_Aux2Off = 43;
	const byte CV_Aux2On = 44;
	const byte CV_positionIndicationToggle = 45;
	const byte CV_errorIndicationToggle = 46;
	const byte CV_turnoutPosition = 50;
	const byte CV_servo2MinTravel = 62;
	const byte CV_servo2MaxTravel = 63;
	const byte CV_servo3MinTravel = 64;
	const byte CV_servo3MaxTravel = 65;
	const byte CV_servo4MinTravel = 66;
	const byte CV_servo4MaxTravel = 67;

	// set up default cv's
	struct CVPair
	{
		uint16_t  CV;
		uint8_t   Value;
		bool      SoftReset;
	};

	// factory default settings
	const byte CV_reset = 55;
	const byte CV_softResetValue = 11;
	const byte CV_hardResetValue = 55;

	CVPair FactoryDefaultCVs[23] =
	{
		{ CV_AddressLSB, 1, false },
		{ CV_AddressMSB, 0, false },
		{ CV_servo1MinTravel, 90, false },
		{ CV_servo1MaxTravel, 90, false },
		{ CV_servoLowSpeed, 25, true },
		{ CV_servoHighSpeed, 0, true },
		{ CV_occupancySensorSwap, 0, true },
		{ CV_dccCommandSwap, 0, true },
		{ CV_relaySwap, 0, true },
		{ CV_Aux1Off, 10, true },
		{ CV_Aux1On, 11, true },
		{ CV_Aux2Off, 20, true },
		{ CV_Aux2On, 21, true },
		{ CV_positionIndicationToggle, 1, true },
		{ CV_errorIndicationToggle, 2, true },
		{ CV_turnoutPosition, 0, false },
		{ CV_servo2MinTravel, 90, false },
		{ CV_servo2MaxTravel, 90, false },
		{ CV_servo3MinTravel, 90, false },
		{ CV_servo3MaxTravel, 90, false },
		{ CV_servo4MinTravel, 90, false },
		{ CV_servo4MaxTravel, 90, false }
	};


	// event handlers
	void DCCExtCommandHandler(unsigned int Addr, unsigned int Data);
	void DCCPomHandler(unsigned int Addr, byte instType, unsigned int CV, byte Value);
	void ErrorTimerHandler();
};

#endif
