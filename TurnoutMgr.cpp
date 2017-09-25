
#include "TurnoutMgr.h"


// ========================================================================================================
// Public Methods


// TurnoutMgr constructor
TurnoutMgr::TurnoutMgr() :
	servo(Servo1Pin, ServoPowerPin),
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
	servo.SetServoStartupHandler(WrapperServoStartup);
	servo.SetServoMoveDoneHandler(WrapperServoMoveDone);
	servo.SetServoPowerOffHandler(WrapperServoPowerOff);

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
    servo.Update(currentMillis);
}


// Initialize the turnout manager by setting up the dcc config, reading stored values from CVs, and setting up the servo
void TurnoutMgr::InitMain()
{
	// do the init stuff in TurnoutBase
	TurnoutBase::InitMain();

	// servo setup - get extents, rates, and last position from cv's
	servo.Initialize(
		dcc.GetCV(CV_servo1MinTravel),
		dcc.GetCV(CV_servo1MaxTravel),
		dcc.GetCV(CV_servoLowSpeed) * 100,
		dcc.GetCV(CV_servoHighSpeed) * 100,
		position);

	// initlize led and relays
	SetRelays();

	// finally, kick off the bitstream capture
	bitStream.Resume();

#ifdef _DEBUG
	Serial.println("TurnoutMgr init done.");
#endif
}


// set the led and relays for the current position
void TurnoutMgr::SetRelays()
{
    // set the led solid for the current position
    led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::ON);

    // enable the appropriate relay, swapping if needed
    if ((position == STRAIGHT && !relaySwap) || (position == CURVED && relaySwap)) 
    {
        relayStraight.SetPin(HIGH);
    }

    if ((position == CURVED && !relaySwap) || ((position == STRAIGHT) && relaySwap))
    {
        relayCurved.SetPin(HIGH);
    }
}


// set the turnout to a new position (also disables relays prior to starting servo motion)
void TurnoutMgr::SetServo(bool ServoRate)
{
    // turn off the relays
    relayStraight.SetPin(LOW);
    relayCurved.SetPin(LOW);

    // set the servo
    servo.Set(position, ServoRate);

#ifdef _DEBUG
    Serial.print("Setting servo to ");
    Serial.print(position, DEC);
    Serial.print(" at rate ");
    Serial.println(ServoRate, DEC);
#endif

    // store new position to cv
    dcc.SetCV(CV_turnoutPosition,position);
}



// ========================================================================================================
// Event Handlers


// do things when the servo starts up
void TurnoutMgr::ServoStartupHandler()
{
	bitStream.Suspend();  // suspend the bitstream to free up Timer1 for servo usage
}

// do things after the servo is powered off
void TurnoutMgr::ServoPowerOffHandler()
{
    bitStream.Resume();   // resume the bitstream capture stopped prior to beginning servo motion
}


// do things after the servo finishes moving to its new position
void TurnoutMgr::ServoMoveDoneHandler() 
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
            position = (State) !position;
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


// handle straight occupancy sensor signal
void TurnoutMgr::OSStraightHandler(bool ButtonState)
{
    State newPos = (occupancySensorSwap) ? CURVED : STRAIGHT;

    // check occupancy sensor state (LOW so we respond when train detected)
    if (ButtonState == LOW && newPos != position)
    {
        position = newPos;
        led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::FLASH);
        SetServo(HIGH);
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
        led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::FLASH);
        SetServo(HIGH);
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
        led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::FLASH);
        SetServo(LOW);
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
	if (CV == CV_servo1MinTravel) servo.SetExtent(LOW, dcc.GetCV(CV_servo1MinTravel));
	if (CV == CV_servo1MaxTravel) servo.SetExtent(HIGH, dcc.GetCV(CV_servo1MaxTravel));
	if (CV == CV_servoLowSpeed) servo.SetDuration(LOW, dcc.GetCV(CV_servoLowSpeed) * 100);
	if (CV == CV_servoHighSpeed) servo.SetDuration(HIGH, dcc.GetCV(CV_servoHighSpeed) * 100);
}



// ========================================================================================================

TurnoutMgr *TurnoutMgr::currentInstance = 0;    // pointer to allow us to access member objects from callbacks

// servo/sensor callback wrappers
void TurnoutMgr::WrapperButtonPress(bool ButtonState) { currentInstance->ButtonEventHandler(ButtonState); }
void TurnoutMgr::WrapperOSStraight(bool ButtonState) { currentInstance->OSStraightHandler(ButtonState); }
void TurnoutMgr::WrapperOSCurved(bool ButtonState) { currentInstance->OSCurvedHandler(ButtonState); }
void TurnoutMgr::WrapperServoStartup() { currentInstance->ServoStartupHandler(); }
void TurnoutMgr::WrapperServoMoveDone() { currentInstance->ServoMoveDoneHandler(); }
void TurnoutMgr::WrapperServoPowerOff() { currentInstance->ServoPowerOffHandler(); }


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

void TurnoutMgr::WrapperDCCAccPomPacket(int boardAddress,int outputAddress, byte instructionType, int cv, byte data)
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
