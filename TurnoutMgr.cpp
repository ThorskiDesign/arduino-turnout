
#include "TurnoutMgr.h"


// ========================================================================================================
// Public Methods


// TurnoutMgr constructor
TurnoutMgr::TurnoutMgr():
	osStraight(Sensor1Pin, true),
	osCurved(Sensor2Pin, true),
	relayStraight(Relay1Pin),
	relayCurved(Relay2Pin) 
{
	// set pointer to this instance of the turnout manager, so that we can reference it in callbacks
	currentInstance = this;

	// configure sensor/servo event handlers
	button.SetButtonPressHandler(WrapperButtonPress);
	osStraight.SetButtonPressHandler(WrapperOSStraight);
	osCurved.SetButtonPressHandler(WrapperOSCurved);
	servo[0].SetServoMoveDoneHandler(WrapperServoMoveDone);

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

	// configure servo event handlers
	for (byte i = 0; i < numServos; i++)
		servo[i].SetServoMoveDoneHandler(WrapperServoMoveDone);
	servoTimer.SetTimerHandler(WrapperServoTimer);
}


// Check for factory reset, then proceed with main initialization
void TurnoutMgr::Initialize()
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
void TurnoutMgr::Update()
{
	// do all the updates that TurnoutBase handles
	TurnoutBase::Update();

	// then update our sensors and servo
	unsigned long currentMillis = millis();
	osStraight.Update(currentMillis);
	osCurved.Update(currentMillis);

	if (servosActive)
		for (byte i = 0; i < numServos; i++)
			servo[i].Update(currentMillis);

}



// ========================================================================================================
// Private Methods


// Initialize the turnout manager by setting up the dcc config, reading stored values from CVs, and setting up the servo
void TurnoutMgr::InitMain()
{
	// do the init stuff in TurnoutBase
	TurnoutBase::InitMain();    
	
	servo[0].Initialize(
        dcc.GetCV(CV_servo1MinTravel),
        dcc.GetCV(CV_servo1MaxTravel),
        dcc.GetCV(CV_servoLowSpeed) * 100,
        dcc.GetCV(CV_servoHighSpeed) * 100,
        servoState);

    // set led and relays, and begin bitstream capture
    EndServoMove();

#ifdef _DEBUG
	Serial.println("TurnoutMgr init done.");
#endif
}


// set the turnout to a new position
void TurnoutMgr::BeginServoMove()
{
    // store new position to cv
    dcc.SetCV(CV_turnoutPosition,position);

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
void TurnoutMgr::EndServoMove()
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
void TurnoutMgr::ServoMoveDoneHandler() 
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
void TurnoutMgr::ButtonEventHandler(bool ButtonState)
{
    // check button state (HIGH so we respond after button release)
    if (ButtonState == HIGH)
    {
        // proceed only if both occupancy sensors are inactive (i.e., sensors override button press)
        if (osStraight.SwitchState() == HIGH && osCurved.SwitchState() == HIGH)
        {
            // toggle from current position and set new position
            position = (State) !position;
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


// handle straight occupancy sensor signal
void TurnoutMgr::OSStraightHandler(bool ButtonState)
{
    State newPos = (occupancySensorSwap) ? CURVED : STRAIGHT;

    // check occupancy sensor state (LOW so we respond when train detected)
    if (ButtonState == LOW && newPos != position)
    {
        position = newPos;
		servoRate = HIGH;
        BeginServoMove();
    }
}


// handle curved occupancy sensor signal
void TurnoutMgr::OSCurvedHandler(bool ButtonState)
{
    State newPos = (occupancySensorSwap) ? STRAIGHT : CURVED;

    // check occupancy sensor state (LOW so we respond when train detected)
    if (ButtonState == LOW && newPos != position)
    {
        position = newPos;
		servoRate = HIGH;
        BeginServoMove();
    }
}


// handle a DCC basic accessory command, used for changing the state of the turnout
void TurnoutMgr::DCCAccCommandHandler(unsigned int Addr, unsigned int Direction)
{
    // assume we are filtering repeated packets in the packet builder, so we don't check for that here
	// assume DCCdecoder is set to return only packets for this decoder's address.

    State dccState;
    dccState = (Direction == 0) ? CURVED : STRAIGHT;
    if (dccCommandSwap) dccState = (State) !dccState; // swap the interpretation of dcc command if needed

    // if we are already in the desired position, just exit
    if (dccState == position) return;

#ifdef _DEBUG
    Serial.print("Received dcc command to position ");
    Serial.println(dccState, DEC);
#endif

    // proceed only if both occupancy sensors are inactive (i.e., sensors override dcc command)
    if (osStraight.SwitchState() == HIGH && osCurved.SwitchState() == HIGH)
    {
        // set switch state based on dcc command
        position = dccState;
		servoRate = LOW;
        BeginServoMove();
    }
    else
    {
        // command error indication, normal led will resume after this timer expires
        errorTimer.StartTimer(1000);
        led.SetLED(RgbLed::YELLOW, RgbLed::FLASH);
    }
}



// handle a DCC program on main command
void TurnoutMgr::DCCPomHandler(unsigned int Addr, byte instType, unsigned int CV, byte Value)
{
	// do most of the program on main stuff in TurnoutBase
	TurnoutBase::DCCPomHandler(Addr, instType, CV, Value);

	// update servo vars from eeprom
	if (CV == CV_servo1MinTravel) servo[0].SetExtent(LOW, dcc.GetCV(CV_servo1MinTravel));
	if (CV == CV_servo1MaxTravel) servo[0].SetExtent(HIGH, dcc.GetCV(CV_servo1MaxTravel));
	if (CV == CV_servoLowSpeed) servo[0].SetDuration(LOW, dcc.GetCV(CV_servoLowSpeed) * 100);
	if (CV == CV_servoHighSpeed) servo[0].SetDuration(HIGH, dcc.GetCV(CV_servoHighSpeed) * 100);
}


// ========================================================================================================

TurnoutMgr *TurnoutMgr::currentInstance = 0;    // pointer to allow us to access member objects from callbacks

												// servo/sensor callback wrappers
void TurnoutMgr::WrapperButtonPress(bool ButtonState) { currentInstance->ButtonEventHandler(ButtonState); }
void TurnoutMgr::WrapperOSStraight(bool ButtonState) { currentInstance->OSStraightHandler(ButtonState); }
void TurnoutMgr::WrapperOSCurved(bool ButtonState) { currentInstance->OSCurvedHandler(ButtonState); }
void TurnoutMgr::WrapperServoMoveDone() { currentInstance->ServoMoveDoneHandler(); }


// ========================================================================================================
// dcc processor callback wrappers

void TurnoutMgr::WrapperDCCAccPacket(int boardAddress, int outputAddress, byte activate, byte data)
{
	currentInstance->DCCAccCommandHandler(outputAddress, data);
}

void TurnoutMgr::WrapperDCCExtPacket(int boardAddress, int outputAddress, byte data)
{
	currentInstance->DCCExtCommandHandler(outputAddress, data);
}

void TurnoutMgr::WrapperDCCAccPomPacket(int boardAddress, int outputAddress, byte instructionType, int cv, byte data)
{
	currentInstance->DCCPomHandler(outputAddress, instructionType, cv, data);
}

void TurnoutMgr::WrapperDCCDecodingError(byte errorCode)
{
	// TODO: add optional LED indication

#ifdef _DEBUG
	Serial.print("Packet error, code: ");
	Serial.println(errorCode, DEC);
#endif
}


// wrappers for callbacks in TurnoutBase ================================================================

// this is called from the bitstream capture when there are 32 bits to process.
void TurnoutMgr::WrapperBitStream(unsigned long incomingBits)
{
	currentInstance->dccPacket.ProcessIncomingBits(incomingBits);
}

void TurnoutMgr::WrapperBitStreamError(byte errorCode)
{
	currentInstance->bitErrorCount++;
}


// this is called by the packet builder when a complete packet is ready, to kick off the actual decoding
void TurnoutMgr::WrapperDCCPacket(byte *packetData, byte size)
{
	// kick off the packet processor
	currentInstance->dcc.ProcessPacket(packetData, size);
}

void TurnoutMgr::WrapperDCCPacketError(byte errorCode)
{
	currentInstance->packetErrorCount++;
}


// timer callback wrappers
void TurnoutMgr::WrapperResetTimer() { currentInstance->ResetTimerHandler(); }
void TurnoutMgr::WrapperErrorTimer() { currentInstance->ErrorTimerHandler(); }
void TurnoutMgr::WrapperServoTimer() { currentInstance->EndServoMove(); }