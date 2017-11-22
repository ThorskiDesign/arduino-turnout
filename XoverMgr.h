
/*

Arduino Crossover Manager

A class for managing the top level operation of a DCC controlled, servo actuated model railroad
crossover.

Summary:

The XoverMgr class provides the top level functionality for the crossover. It handles the processing
of received DCC commands, controls the servos that drive the turnouts, controls the LED indications,
controls the relays for powering points or leads, and monitors the occupancy sensors. A reset
to default may be performed by holding the pushbutton while the hardware is powered up.

Example Usage:

XoverMgr CrossoverManager;      // create an instance of the crossover manager
XoverMgr.Initialize();          // initialize the crossover manager. call this in setup().
XoverMgr.Update();              // check for DCC commands, update sensors and actuators.
								   call this in loop().

Details:

The InitMain method performs the major setup for the class, including setting up the DCC packet
processor, reading the stored configuration from EEPROM (via the DCCdecoder lib), getting the stored
position of the crossover, and configuring the servos. It calls the EndServoMove method to set the LED
and relays and start the bitstream capture. Event handlers are configured in the constructor.
If a factory reset is triggered in the Initialize method, the CVs are restored to their default
settings, and a timer is set which then runs the InitMain method. The Initialize method should be
called once from the main arduino setup() function.

The Update method should be called in the main arduino loop() method. This method processes timestamps
received by the BitStream object, which then sends them to the DCCpacket object to be assembled
into a full DCC packet. It also handles millis-related updates for the LED, sensors, timers and
servo.

The BeginServoMove method configures the crossover prior to beginning the servo motions. It stores the
new position to EEPROM, starts the LED flashing, disables the relays, and stops the bitstream capture.
It then starts PWM for the servos and enables the servo power pin. Each motion is performed in turn,
with the ServoMoveDoneHandler called after each servo motion is complete. After the final servo motion 
is complete, the EndServoMove method is called via the servoTimer event handler. The EndServoMove method 
sets the LED for the new position, stops the servo PWM and disables the servo power, resumes the 
bitstream capture, and sets the relays.

The ButtonEventHandler responds to events from the button and triggers a change in the crossover 
position.

The DCCAccCommandHandler processes a basic accessory command, used to set the position of the
crossover. Occupancy sensors are checked prior to setting the turnouts, with an error indication given
if they are occupied. The DCCPomHandler method processes a program on main packet. It checks for a
valid CV, stores the data via the DCCdecoder object, and then re-reads the basic configuration for
the crossover.

Event handler wrappers for the sensors, button, servos, timer, and DCC classes are static, so that
they are accessible as callbacks from those classes. An instance variable provides access to the
instance of the crossover manager, where the actual callback handling takes place.

*/

#ifndef _XOVERMGR_h
#define _XOVERMGR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "TurnoutBase.h"
#include "TurnoutServo.h"


class XoverMgr : protected TurnoutBase
{
public:
	XoverMgr();
	void Initialize();
	void Update();

private:
	// main functions
	void InitMain();
	void BeginServoMove();
	void EndServoMove();

	// Sensors and outputs
	const byte numServos = 4;
	TurnoutServo servo[4] = { { Servo1Pin },{ Servo2Pin },{ Servo3Pin },{ Servo4Pin } };
	OutputPin relay[4] = { { Relay1Pin }, {Relay2Pin}, {Relay3Pin},{Relay4Pin} };
	Button osAB{ Sensor1Pin, true };            // occupancy sensor between switches A and B (servos 0 and 2)
	Button osCD{ Sensor2Pin, true };            // occupancy sensor between switches C and D (servos 1 and 3)

	// servo and relay state tables
	const byte servoState[4][2] = {
		{ 0, 1},
		{ 0, 1 },
		{ 0, 1 },
		{ 0, 1 }
	};
	const byte relayState[4][2] = {
		{ 1, 0 },
		{ 0, 1 },
		{ 1, 0 },
		{ 0, 1 }
	};

	// event handlers
	void ResetTimerHandler();
	void ServoMoveDoneHandler();
	void ButtonEventHandler(bool ButtonState);
	void OSABHandler(bool ButtonState);
	void OSCDHandler(bool ButtonState);
	void DCCAccCommandHandler(unsigned int Addr, unsigned int Direction);
	void DCCPomHandler(unsigned int Addr, byte instType, unsigned int CV, byte Value);

	// pointer to allow us to access member objects from callbacks
	static XoverMgr *currentInstance;

	// Turnout manager event handler wrappers
	static void WrapperServoMoveDone();
	static void WrapperButtonPress(bool ButtonState);
	static void WrapperOSAB(bool ButtonState);
	static void WrapperOSCD(bool ButtonState);

	// DCC event handler wrappers
	static void WrapperDCCAccPacket(int boardAddress, int outputAddress, byte activate, byte data);
	static void WrapperDCCExtPacket(int boardAddress, int outputAddress, byte data);
	static void WrapperDCCAccPomPacket(int boardAddress, int outputAddress, byte instructionType, int cv, byte data);
	static void WrapperDCCDecodingError(byte errorCode);

	// wrappers for callbacks in TurnoutBase ================================================================

	// callbacks for bitstream and packet builder
	static void WrapperBitStream(unsigned long incomingBits);
	static void WrapperBitStreamError(byte errorCode);
	static void WrapperDCCPacket(byte *packetData, byte size);
	static void WrapperDCCPacketError(byte errorCode);

	// Turnout manager event handler wrappers
	static void WrapperResetTimer();
	static void WrapperErrorTimer();
	static void WrapperServoTimer();
};

#endif
