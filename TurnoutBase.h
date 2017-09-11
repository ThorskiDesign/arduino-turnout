// TurnoutBase.h

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
#include "TurnoutServo.h"
#include "RGB_LED.h"
#include "Button.h"
#include "OutputPin.h"
#include "EventTimer.h"


class TurnoutBase
{
public:
	TurnoutBase();
	void Initialize();
	void Update();

protected:
	// Hardware assignments
	const byte Aux1Pin = 0;
	const byte Aux2Pin = 1;
	const byte ButtonPin = 3;
	const byte ServoPowerPin = 4;
	const byte ServoPWMPin = 5;
	const byte LedPinB = 6;
	const byte LedPinR = 7;
	const byte DCCPin = 8;
	const byte Servo2PWMPin = 9;
	const byte Servo3PWMPin = 10;
	const byte Servo4PWMPin = 11;
	const byte LedPinG = 12;
	const byte RelayStraightPin = 14;
	const byte RelayCurvedPin = 15;
	const byte OSstraightPin = 16;
	const byte OScurvedPin = 17;

	// main functions
	void InitMain();
	void FactoryReset(bool HardReset);

	// Sensors and outputs
	Button button;
	Button osStraight;
	Button osCurved;
	RgbLed led;
	OutputPin relayStraight;
	OutputPin relayCurved;
	OutputPin auxOutput1;
	OutputPin auxOutput2;
	EventTimer resetTimer;
	EventTimer errorTimer;

	// DCC bitstream and packet processors
	BitStream bitStream;
	DCCpacket dccPacket;
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
	bool servoEndPointSwap = false;            // optionally swap the low/high servo endpoints
	bool occupancySensorSwap = false;          // optionally swap the straight/curved occupancy sensors
	bool dccCommandSwap = false;               // optionally swap the meaning of received dcc commands
	bool relaySwap = false;					   // optionally swap the straight/curved relays
	bool factoryReset = false;                 // is a reset in progress
	bool showErrorIndication = true;           // enable or disable LED error indications

	// define our available cv's  (allowable range 33-81 per 9.2.2)
	const byte CV_AddressLSB = 1;
	const byte CV_AddressMSB = 9;
	const byte CV_servoMinTravel = 33;
	const byte CV_servoMaxTravel = 34;
	const byte CV_servoLowSpeed = 35;
	const byte CV_servoHighSpeed = 36;
	const byte CV_servoEndPointSwap = 37;
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
		{ CV_servoMinTravel, 90, false },
		{ CV_servoMaxTravel, 90, false },
		{ CV_servoLowSpeed, 25, true },
		{ CV_servoHighSpeed, 0, true },
		{ CV_servoEndPointSwap, 0, true },
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
	void ResetTimerHandler();
	void ErrorTimerHandler();

	// pointer to allow us to access member objects from callbacks
	static TurnoutBase* currentInstance;

	// callbacks for bitstream and packet builder
	static void WrapperBitStream(unsigned long incomingBits);
	static void WrapperBitStreamError(byte errorCode);
	static void WrapperDCCPacket(byte *packetData, byte size);
	static void WrapperDCCPacketError(byte errorCode);

	// Turnout manager event handler wrappers
	static void WrapperResetTimer();
	static void WrapperErrorTimer();
};

#endif
