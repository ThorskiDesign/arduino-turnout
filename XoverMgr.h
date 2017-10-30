// XoverMgr.h

#ifndef _XOVERMGR_h
#define _XOVERMGR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "TurnoutBase.h"


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
