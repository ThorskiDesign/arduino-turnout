
#include "XoverMgr.h"


// ========================================================================================================
// Public Methods


// TurnoutMgr constructor
XoverMgr::XoverMgr() :
	servoA(Servo1Pin, ServoPowerPin),
	servoB(Servo2Pin, ServoPowerPin),
	servoC(Servo3Pin, ServoPowerPin),
	servoD(Servo4Pin, ServoPowerPin),
	osAB(Sensor1Pin, true),
	osCD(Sensor2Pin, true),
	relayACstraight(Relay1Pin),
	relayACcurved(Relay2Pin),
	relayBDstraight(Relay3Pin),
	relayBDcurved(Relay4Pin)
{
	// set pointer to this instance of the turnout manager, so that we can reference it in callbacks
	currentInstance = this;

	// configure sensor/servo event handlers
	button.SetButtonPressHandler(WrapperButtonPress);
	osAB.SetButtonPressHandler(WrapperOSAB);
	osCD.SetButtonPressHandler(WrapperOSCD);

	// TODO: figure out how to handle servo poweron/off events
	servoA.SetServoStartupHandler(WrapperServoStartup);
	servoA.SetServoMoveDoneHandler(WrapperServoMoveDone);
	servoA.SetServoPowerOffHandler(WrapperServoPowerOff);

	// configure dcc event handlers
	dcc.SetBasicAccessoryDecoderPacketHandler(WrapperDCCAccPacket);
	dcc.SetExtendedAccessoryDecoderPacketHandler(WrapperDCCExtPacket);
	dcc.SetBasicAccessoryPomPacketHandler(WrapperDCCAccPomPacket);
	dcc.SetDecodingErrorHandler(WrapperDCCDecodingError);

	// set callbacks for the bitstream capture
	bitStream.SetDataFullHandler(WrapperBitStream);
	bitStream.SetErrorHandler(WrapperBitStreamError);

	// set callbacks for the packet builder
	dccPacket.SetPacketCompleteHandler(WrapperDCCPacket);
	dccPacket.SetPacketErrorHandler(WrapperDCCPacketError);

	// configure timer event handlers
	errorTimer.SetTimerHandler(WrapperErrorTimer);
	resetTimer.SetTimerHandler(WrapperResetTimer);
}


// Check for factory reset, then proceed with main initialization
void XoverMgr::Initialize()
{
	// check for button hold on startup (for reset to defaults)
	if (button.RawState() == LOW)
	{
		FactoryReset(true);    // perform a complete reset
	}
	else
	{
		InitMain();
	}
}


// update sensors and outputs
void XoverMgr::Update()
{
	// do all the updates that TurnoutBase handles
	TurnoutBase::Update();

	// then update our sensors and servo
	unsigned long currentMillis = millis();
	osAB.Update(currentMillis);
	osCD.Update(currentMillis);
	servoA.Update(currentMillis);
	servoB.Update(currentMillis);
	servoC.Update(currentMillis);
	servoD.Update(currentMillis);
}


// Initialize the turnout manager by setting up the dcc config, reading stored values from CVs, and setting up the servo
void XoverMgr::InitMain()
{
	// do the init stuff in TurnoutBase
	TurnoutBase::InitMain();

	// servo setup - get extents, rates, and last position from cv's
	byte lowSpeed = dcc.GetCV(CV_servoLowSpeed) * 100;
	byte highSpeed = dcc.GetCV(CV_servoHighSpeed) * 100;
	servoA.Initialize(dcc.GetCV(CV_servo1MinTravel), dcc.GetCV(CV_servo1MaxTravel), lowSpeed, highSpeed, StateAC());
	servoB.Initialize(dcc.GetCV(CV_servo2MinTravel), dcc.GetCV(CV_servo2MaxTravel), lowSpeed, highSpeed, StateBD());
	servoC.Initialize(dcc.GetCV(CV_servo3MinTravel), dcc.GetCV(CV_servo3MaxTravel), lowSpeed, highSpeed, StateAC());
	servoD.Initialize(dcc.GetCV(CV_servo4MinTravel), dcc.GetCV(CV_servo4MaxTravel), lowSpeed, highSpeed, StateBD());

	// initlize led and relays
	SetRelays();

	// finally, kick off the bitstream capture
	bitStream.Resume();

#ifdef _DEBUG
	Serial.println("TurnoutMgr init done.");
#endif
}


// set the led and relays for the current position
void XoverMgr::SetRelays()
{
	State stateAC = StateAC();
	State stateBD = StateBD();

	if (stateAC == STRAIGHT && stateBD == STRAIGHT)            // all turnouts straight
	{
		led.SetLED(RgbLed::GREEN, RgbLed::ON);
		relayACstraight.SetPin(HIGH);
		relayBDstraight.SetPin(HIGH);
	}
	if (stateAC == STRAIGHT && stateBD == CURVED)              // A and C straight, B and D curved
	{
		led.SetLED(RgbLed::CYAN, RgbLed::ON);
		relayACstraight.SetPin(HIGH);
		relayBDcurved.SetPin(HIGH);
	}
	if (stateAC == CURVED && stateBD == STRAIGHT)              // A and C curved, B and D straight
	{
		led.SetLED(RgbLed::MAGENTA, RgbLed::ON);
		relayACcurved.SetPin(HIGH);
		relayBDstraight.SetPin(HIGH);
	}
	if (stateAC == CURVED && stateBD == CURVED)                // all turnouts curved
	{
		led.SetLED(RgbLed::RED, RgbLed::ON);
		relayACcurved.SetPin(HIGH);
		relayBDcurved.SetPin(HIGH);
	}
}

TurnoutBase::State XoverMgr::StateAC()
{
	return State();
}

TurnoutBase::State XoverMgr::StateBD()
{
	return State();
}


// set the turnout to a new position (also disables relays prior to starting servo motion)
void XoverMgr::SetServos(bool ServoRate)
{
	// turn off the relays
	relayACcurved.SetPin(LOW);
	relayACstraight.SetPin(LOW);
	relayBDcurved.SetPin(LOW);
	relayBDstraight.SetPin(LOW);

	// set the servos
	State stateAC = StateAC();
	State stateBD = StateBD();

	servoA.Set(stateAC, ServoRate);
	servoB.Set(stateBD, ServoRate);
	servoC.Set(stateAC, ServoRate);
	servoD.Set(stateBD, ServoRate);

#ifdef _DEBUG
	Serial.print("Setting servos A/C to "); Serial.print(stateAC, DEC);
	Serial.print(",  Setting servos B/D to "); Serial.print(stateBD, DEC); 
	Serial.print(",  at rate "); Serial.println(ServoRate, DEC);
#endif

	// TODO: how to store xover position?
	// store new position to cv
	dcc.SetCV(CV_turnoutPosition, position);
}




// ========================================================================================================
// Event Handlers


// do things when the servo starts up
void XoverMgr::ServoStartupHandler()
{
	bitStream.Suspend();  // suspend the bitstream to free up Timer1 for servo usage
}

// do things after the servo is powered off
void XoverMgr::ServoPowerOffHandler()
{
	bitStream.Resume();   // resume the bitstream capture stopped prior to beginning servo motion
}


// do things after the servo finishes moving to its new position
void XoverMgr::ServoMoveDoneHandler()
{
	SetRelays();          // set the relays for the new position
}

// handle a button press
void TurnoutMgr::ButtonEventHandler(bool ButtonState)
{
	// check button state (HIGH so we respond after button release)
	if (ButtonState == HIGH)
	{
		// proceed only if both occupancy sensors are inactive (i.e., sensors override button press)
		if (osStraight.SwitchState() == HIGH && osCurved.SwitchState() == HIGH)
		{
			// toggle from current position and set new position
			position = (State)!position;
			led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::FLASH);
			SetServo(LOW);
		}
		else
		{
			// button error indication, normal led will resume after this timer expires
			errorTimer.StartTimer(1000);
			led.SetLED(RgbLed::YELLOW, RgbLed::ON);
		}
	}
}









// ========================================================================================================

XoverMgr *XoverMgr::currentInstance = 0;    // pointer to allow us to access member objects from callbacks

												// servo/sensor callback wrappers
void XoverMgr::WrapperButtonPress(bool ButtonState) { currentInstance->ButtonEventHandler(ButtonState); }
void XoverMgr::WrapperOSAB(bool ButtonState) { currentInstance->OSABHandler(ButtonState); }
void XoverMgr::WrapperOSCD(bool ButtonState) { currentInstance->OSCDHandler(ButtonState); }
void XoverMgr::WrapperServoStartup() { currentInstance->ServoStartupHandler(); }
void XoverMgr::WrapperServoMoveDone() { currentInstance->ServoMoveDoneHandler(); }
void XoverMgr::WrapperServoPowerOff() { currentInstance->ServoPowerOffHandler(); }


// ========================================================================================================
// dcc processor callback wrappers

void XoverMgr::WrapperDCCAccPacket(int boardAddress, int outputAddress, byte activate, byte data)
{
	currentInstance->DCCAccCommandHandler(outputAddress, data);
}

void XoverMgr::WrapperDCCExtPacket(int boardAddress, int outputAddress, byte data)
{
	currentInstance->DCCExtCommandHandler(outputAddress, data);
}

void XoverMgr::WrapperDCCAccPomPacket(int boardAddress, int outputAddress, byte instructionType, int cv, byte data)
{
	currentInstance->DCCPomHandler(outputAddress, instructionType, cv, data);
}

void XoverMgr::WrapperDCCDecodingError(byte errorCode)
{
	// TODO: add optional LED indication

#ifdef _DEBUG
	Serial.print("Packet error, code: ");
	Serial.println(errorCode, DEC);
#endif
}


// wrappers for callbacks in TurnoutBase ================================================================

// this is called from the bitstream capture when there are 32 bits to process.
void XoverMgr::WrapperBitStream(unsigned long incomingBits)
{
	currentInstance->dccPacket.ProcessIncomingBits(incomingBits);
}

void XoverMgr::WrapperBitStreamError(byte errorCode)
{
	currentInstance->bitErrorCount++;
}


// this is called by the packet builder when a complete packet is ready, to kick off the actual decoding
void XoverMgr::WrapperDCCPacket(byte *packetData, byte size)
{
	// kick off the packet processor
	currentInstance->dcc.ProcessPacket(packetData, size);
}

void XoverMgr::WrapperDCCPacketError(byte errorCode)
{
	currentInstance->packetErrorCount++;
}


// timer callback wrappers
void XoverMgr::WrapperResetTimer() { currentInstance->ResetTimerHandler(); }
void XoverMgr::WrapperErrorTimer() { currentInstance->ErrorTimerHandler(); }
