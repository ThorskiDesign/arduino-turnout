// TurnoutMgr.h

#ifndef _TURNOUTMGR_h
#define _TURNOUTMGR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Bitstream.h"
#include "DCCpacket.h"
#include "DCCdecoder.h"
#include "TurnoutServo.h"
#include "RGB_LED.h"
#include "Button.h"
#include "Relay.h"
#include "EventTimer.h"


class TurnoutMgr
{
public:
	TurnoutMgr();
	void Initialize();
	void Update();

private:
    // Hardware assignments
	const byte DCCPin = 2;
	const byte ButtonPin = 3;
	const byte ServoPowerPin = 4;
	const byte ServoPWMPin = 5;
	const byte LedPinR = 7;
	const byte LedPinG = 8;
	const byte LedPinB = 6;
	const byte RelayStraightPin = 14;
	const byte RelayCurvedPin = 15;
	const byte OSstraightPin = 16;
	const byte OScurvedPin = 17;

    // main functions
	void InitMain();
	void FactoryReset();
	void SetServo(bool ServoRate);
	void SetRelays();

    // DCC bitstream and packet processors
    BitStream bitStream;
    DCCpacket dccPacket;
    DCCdecoder dcc;

    // Sensors and outputs
	Button button;
	Button osStraight;
	Button osCurved;
	RgbLed led;
	TurnoutServo servo;
	RelayClass relayStraight;
	RelayClass relayCurved;
	EventTimer resetTimer;
	EventTimer errorTimer;

    // bitstream and packet builder related
    boolean haveNewBits = false;          // flag from bitstream object for next set of bits
    volatile unsigned long bits = 0;      // 32 bits from the bitstream
    unsigned long bitErrorCount = 0;
    unsigned long packetErrorCount = 0;
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

	// define our available cv's
	const byte CV_servoMinTravel = 32;
	const byte CV_servoMaxTravel = 33;
	const byte CV_servoLowSpeed = 34;
	const byte CV_servoHighSpeed = 35;
	const byte CV_servoEndPointSwap = 36;
	const byte CV_occupancySensorSwap = 37;
	const byte CV_dccCommandSwap = 38;
	const byte CV_relaySwap = 39;
	const byte CV_turnoutPosition = 50;

	// set up default cv's
	struct CVPair
	{
		uint16_t  CV;
		uint8_t   Value;
	};

	// factory default settings
	CVPair FactoryDefaultCVs [11] =
	{
		{kCV_AddressLSB, 1},    // defined in DCCdecoder.h, default to address = 1
		{kCV_AddressMSB, 0},    // defined in DCCdecoder.h
		{CV_servoMinTravel, 90},
		{CV_servoMaxTravel, 110},
		{CV_servoLowSpeed, 25},
		{CV_servoHighSpeed, 0},
		{CV_servoEndPointSwap, 1},
		{CV_occupancySensorSwap, 0},
		{CV_dccCommandSwap, 0},
		{CV_relaySwap, 0},
		{CV_turnoutPosition, 0}
	};

	// event handlers
	void DCCcommandHandler(unsigned int Addr, unsigned int Direction);
	void DCCPomHandler(unsigned int Addr, byte instType, unsigned int CV, byte Value);
	void ButtonEventHandler(bool ButtonState);
	void ServoMoveDoneHandler();
	void ServoPowerOffHandler();
	void ResetTimerHandler();
	void ErrorTimerHandler();
	void OSStraightHandler(bool ButtonState);
	void OSCurvedHandler(bool ButtonState);

    // pointer to allow us to access member objects from callbacks
    static TurnoutMgr* currentInstance;           

    // callbacks for bitstream and packet builder
    static void WrapperBitStream(unsigned long incomingBits);
    static void WrapperBitStreamError(byte errorCode);
    static void WrapperDCCPacket(byte *packetData, byte size);
    static void WrapperDCCPacketError(byte errorCode);
    
    // Turnout manager event handler wrappers
    static void WrapperButtonPress(bool ButtonState);
    static void WrapperServoMoveDon();
    static void WrapperServoPowerOff();
    static void WrapperResetTimer();
    static void WrapperErrorTimer();
    static void WrapperOSStraight(bool ButtonState);
    static void WrapperOSCurved(bool ButtonState);

    // DCC event handler wrappers in main
    static void WrapperDCCAccPacket(int boardAddress, int outputAddress, byte activate, byte data);
    static void WrapperDCCAccPomPacket(int boardAddress,int outputAddress, byte instructionType, int cv, byte data);
    static void WrapperDCCExtAccPacket(int boardAddress, int outputAddress, byte data);
    static void WrapperDCCDecodingError(byte errorCode);
};


#endif
