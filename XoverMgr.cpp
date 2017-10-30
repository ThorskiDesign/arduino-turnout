
#include "XoverMgr.h"


// ========================================================================================================
// Public Methods


// TurnoutMgr constructor
XoverMgr::XoverMgr() :
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

	// assign handler for servo move done events
	for (byte i = 0; i < numServos; i++)
		servo[i].SetServoMoveDoneHandler(WrapperServoMoveDone);

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
	servoTimer.SetTimerHandler(WrapperServoTimer);
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

	if (servosActive)
		for (byte i = 0; i < numServos; i++)
			servo[i].Update(currentMillis);
}


// Initialize the turnout manager by setting up the dcc config, reading stored values from CVs, and setting up the servo
void XoverMgr::InitMain()
{
	// do the init stuff in TurnoutBase
	TurnoutBase::InitMain();

	// servo setup - get extents, rates, and last position from cv's
	byte lowSpeed = dcc.GetCV(CV_servoLowSpeed) * 100;
	byte highSpeed = dcc.GetCV(CV_servoHighSpeed) * 100;
	servo[0].Initialize(dcc.GetCV(CV_servo1MinTravel), dcc.GetCV(CV_servo1MaxTravel), lowSpeed, highSpeed, StateAC());
	servo[1].Initialize(dcc.GetCV(CV_servo2MinTravel), dcc.GetCV(CV_servo2MaxTravel), lowSpeed, highSpeed, StateBD());
	servo[2].Initialize(dcc.GetCV(CV_servo3MinTravel), dcc.GetCV(CV_servo3MaxTravel), lowSpeed, highSpeed, StateAC());
	servo[3].Initialize(dcc.GetCV(CV_servo4MinTravel), dcc.GetCV(CV_servo4MaxTravel), lowSpeed, highSpeed, StateBD());

	// set led and relays, and begin bitstream capture
	EndServoMove();

#ifdef _DEBUG
	Serial.println("TurnoutMgr init done.");
#endif
}


// set the led and relays for the current position
//void XoverMgr::SetRelays()
//{
//	State stateAC = StateAC();
//	State stateBD = StateBD();
//
//	if (stateAC == STRAIGHT && stateBD == STRAIGHT)            // all turnouts straight
//	{
//		led.SetLED(RgbLed::GREEN, RgbLed::ON);
//		relayACstraight.SetPin(HIGH);
//		relayBDstraight.SetPin(HIGH);
//	}
//	if (stateAC == STRAIGHT && stateBD == CURVED)              // A and C straight, B and D curved
//	{
//		led.SetLED(RgbLed::CYAN, RgbLed::ON);
//		relayACstraight.SetPin(HIGH);
//		relayBDcurved.SetPin(HIGH);
//	}
//	if (stateAC == CURVED && stateBD == STRAIGHT)              // A and C curved, B and D straight
//	{
//		led.SetLED(RgbLed::MAGENTA, RgbLed::ON);
//		relayACcurved.SetPin(HIGH);
//		relayBDstraight.SetPin(HIGH);
//	}
//	if (stateAC == CURVED && stateBD == CURVED)                // all turnouts curved
//	{
//		led.SetLED(RgbLed::RED, RgbLed::ON);
//		relayACcurved.SetPin(HIGH);
//		relayBDcurved.SetPin(HIGH);
//	}
//}


TurnoutBase::State XoverMgr::StateAC()
{
	return State();
}

TurnoutBase::State XoverMgr::StateBD()
{
	return State();
}


// set the turnout to a new position (also disables relays prior to starting servo motion)
//void XoverMgr::SetServos(bool ServoRate)
//{
//	// turn off the relays
//	relayACcurved.SetPin(LOW);
//	relayACstraight.SetPin(LOW);
//	relayBDcurved.SetPin(LOW);
//	relayBDstraight.SetPin(LOW);
//
//	// set the servos
//	State stateAC = StateAC();
//	State stateBD = StateBD();
//
//	servoA.Set(stateAC, ServoRate);
//	servoB.Set(stateBD, ServoRate);
//	servoC.Set(stateAC, ServoRate);
//	servoD.Set(stateBD, ServoRate);
//
//#ifdef _DEBUG
//	Serial.print("Setting servos A/C to "); Serial.print(stateAC, DEC);
//	Serial.print(",  Setting servos B/D to "); Serial.print(stateBD, DEC); 
//	Serial.print(",  at rate "); Serial.println(ServoRate, DEC);
//#endif
//
//	// TODO: how to store xover position?
//	// store new position to cv
//	dcc.SetCV(CV_turnoutPosition, position);
//}


// set the turnout to a new position
void XoverMgr::BeginServoMove()
{
	// store new position to cv
	dcc.SetCV(CV_turnoutPosition, position);

	// configure the servo settings
	for (byte i = 0; i < numServos; i++)
		servoState[i] = position;

	// set the led to indicate servo is in motion
	led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::FLASH);

	// stop the bitstream capture
	bitStream.Suspend();

	// turn off the relays
	relayStraight.SetPin(LOW);
	relayCurved.SetPin(LOW);

	// start pwm for current positions of all servos
	for (byte i = 0; i < numServos; i++)
		servo[i].StartPWM();

	// turn on servo power
	servoPower.SetPin(HIGH);

	// set the servo index to the first servo and start moving the servos in sequence
	servosActive = true;
	currentServo = 0;
	ServoMoveDoneHandler();
}


// resume normal operation after servo motion is complete
void XoverMgr::EndServoMove()
{
	// set the led solid for the current position
	led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::ON);

	// turn off servo power
	servoPower.SetPin(LOW);

	// stop pwm all servos
	for (byte i = 0; i < numServos; i++)
		servo[i].StopPWM();

	// enable the appropriate relay, swapping if needed
	if ((position == STRAIGHT && !relaySwap) || (position == CURVED && relaySwap))
	{
		relayStraight.SetPin(HIGH);
	}

	if ((position == CURVED && !relaySwap) || ((position == STRAIGHT) && relaySwap))
	{
		relayCurved.SetPin(HIGH);
	}

	// resume the bitstream capture
	servosActive = false;
	bitStream.Resume();
}





// ========================================================================================================
// Event Handlers


// do things after the servo finishes moving to its new position
void XoverMgr::ServoMoveDoneHandler()
{
	if (currentServo < numServos)
	{
#ifdef _DEBUG
		Serial.print("Setting servo ");
		Serial.print(currentServo, DEC);
		Serial.print(" to ");
		Serial.print(servoState[currentServo], DEC);
		Serial.print(" at rate ");
		Serial.println(servoRate, DEC);
#endif

		servo[currentServo].Set(servoState[currentServo], servoRate);
		currentServo++;
	}
	else
	{
		int servoPowerOffDelay = 500;    // ms
		servoTimer.StartTimer(servoPowerOffDelay);
	}
}


// handle a button press
void XoverMgr::ButtonEventHandler(bool ButtonState)
{
	// check button state (HIGH so we respond after button release)
	if (ButtonState == HIGH)
	{
		// proceed only if both occupancy sensors are inactive (i.e., sensors override button press)
		if (osStraight.SwitchState() == HIGH && osCurved.SwitchState() == HIGH)
		{
			// toggle from current position and set new position
			position = (State)!position;
			servoRate = LOW;
			BeginServoMove();
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
void XoverMgr::WrapperServoMoveDone() { currentInstance->ServoMoveDoneHandler(); }


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
void XoverMgr::WrapperServoTimer() { currentInstance->EndServoMove(); }
