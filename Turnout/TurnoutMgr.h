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

Arduino Turnout Manager

A class for managing the top level operation of a DCC controlled, servo actuated model railroad
turnout.

Summary:

The TurnoutMgr class provides the top level functionality for the turnout. It handles the processing
of received DCC commands, controls the servo that drives the turnout, controls the LED indications,
controls optional relays for powering points or leads, and monitors the occupancy sensors. A reset 
to default may be performed by holding the pushbutton while the hardware is powered up. Options to 
swap the interpretation of the DCC command, the servo endpoints, the occupancy sensors, and the 
relays are provided.

Example Usage:

	TurnoutMgr TurnoutManager;          // create an instance of the turnout manager
	TurnoutManager.Initialize();        // initialize the turnout manager. call this in setup().
	TurnoutManager.Update();            // check for DCC commands, update sensors and actuators.
										   call this in loop().

Details:

The InitMain method performs the major setup for the class, including setting up the DCC packet
processor, reading the stored configuration from EEPROM (via the DCCdecoder lib), getting the stored
position of the turnout, and configuring the servo. It calls the EndServoMove method to set the LED 
and relays and start the bitstream capture. Event handlers are configured in the constructor.
If a factory reset is triggered in the Initialize method, the CVs are restored to their default 
settings, and a timer is set which then runs the InitMain method. The Initialize method should be 
called once from the main arduino setup() function.

The Update method should be called in the main arduino loop() method. This method processes timestamps
received by the BitStream object, which then sends them to the DCCpacket object to be assembled
into a full DCC packet. It also handles millis-related updates for the LED, sensors, timers and
servo.

The BeginServoMove method configures the turnout prior to beginning a servo motion. It stores the
new position to EEPROM, starts the LED flashing, disables relays, and stops the bitstream capture.
It then starts PWM for the servo and enables the servo power pin. It then calls the ServoMoveDoneHandler
to perform the actual motion. After the final servo motion is complete, the EndServoMove method is
called via the servoTimer event handler. The EndServoMove method sets the LED for the new position,
stops the servo PWM and disables the servo power, resumes the bitstream capture, and sets the relays.

The ButtonEventHandler, OSStraightHandler, and OSCurvedHandler respond to events from
the button and occupancy sensors, and trigger a change in the turnout position.

The DCCAccCommandHandler processes a basic accessory command, used to set the position of the
turnout. Occupancy sensors are checked prior to setting the turnout, with an error indication given
if they are occupied. The DCCPomHandler method processes a program on main packet. It checks for a 
valid CV, stores the data via the DCCdecoder object, and then re-reads the basic configuration for 
the turnout.

Event handler wrappers for the sensors, button, servo, timer, and DCC classes are static, so that
they are accessible as callbacks from those classes. An instance variable provides access to the
instance of the turnout manager, where the actual callback handling takes place.

*/

#ifndef _TURNOUTMGR_h
#define _TURNOUTMGR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "TurnoutBase.h"
#include "TurnoutServo.h"


class TurnoutMgr : protected TurnoutBase
{
public:
	TurnoutMgr();
	void Initialize();
	void Update();

private:
	// main functions
	void InitMain();
	void BeginServoMove();
	void EndServoMove();

	// Sensors and outputs
	const byte numServos = 1;
	TurnoutServo servo[1] = { {Servo1Pin} };
	Button osStraight{ Sensor1Pin, true };
	Button osCurved{ Sensor2Pin, true };
	OutputPin relayStraight{ Relay1Pin };
	OutputPin relayCurved{ Relay2Pin };

	// servo and relay state tables
	const byte servoState[1][2] = {
		{ 0, 1 }
	};

	// event handlers
	void ResetTimerHandler();
	void ServoMoveDoneHandler();
	void ButtonEventHandler(bool ButtonState);
	void OSStraightHandler(bool ButtonState);
	void OSCurvedHandler(bool ButtonState);
	void DCCAccCommandHandler(unsigned int Addr, unsigned int Direction);
	void DCCPomHandler(unsigned int Addr, byte instType, unsigned int CV, byte Value);
	void DCCDecodingError(byte errorCode);

	// pointer to allow us to access member objects from callbacks
	static TurnoutMgr *currentInstance;

	// Turnout manager event handler wrappers
	static void WrapperButtonPress(bool ButtonState);
	static void WrapperOSStraight(bool ButtonState);
	static void WrapperOSCurved(bool ButtonState);
	static void WrapperServoMoveDone();

	// DCC event handler wrappers
	static void WrapperDCCAccPacket(int boardAddress, int outputAddress, byte activate, byte data);
	static void WrapperDCCExtPacket(int boardAddress, int outputAddress, byte data);
	static void WrapperDCCAccPomPacket(int boardAddress, int outputAddress, byte instructionType, int cv, byte data);
	static void WrapperDCCDecodingError(byte errorCode);

	// Turnout manager event handler wrappers
	static void WrapperResetTimer();
	static void WrapperErrorTimer();
	static void WrapperServoTimer();

	// Wrappers for events in Turnoutbase
	static void WrapperMaxBitErrors(byte errorCode);
	static void WrapperMaxPacketErrors(byte errorCode);
};


#endif
