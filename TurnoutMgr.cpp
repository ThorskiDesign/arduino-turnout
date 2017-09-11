
#include "TurnoutMgr.h"


// ========================================================================================================
// Public Methods


// TurnoutMgr constructor
TurnoutMgr::TurnoutMgr() :
	servo(ServoPWMPin, ServoPowerPin)
{
	// set pointer to this instance of the turnout manager, so that we can reference it in callbacks
	TurnoutBase::currentInstance = this;
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
}


// check for new bitstream data, update sensors and outputs, check for
// max number of packet errors and reset bitstream capture if necessary
void TurnoutMgr::Update()
{
	// do all the updates that TurnoutBase handles
	TurnoutBase::Update();

	// then do our servo update here
    unsigned long currentMillis = millis();
    servo.Update(currentMillis);
}


// Initialize the turnout manager by setting up the dcc config, reading stored values from CVs, and setting up the servo
void TurnoutMgr::InitMain()
{
	// do the init stuff in TurnoutBase
	TurnoutBase::InitMain();

	// servo setup - get extents, rates, and last position from cv's
	State servoState = position;
	if (servoEndPointSwap)
		servoState = (servoState == STRAIGHT) ? CURVED : STRAIGHT;  // swap the servo endpoints if needed
	servo.Initialize(
		dcc.GetCV(CV_servoMinTravel),
		dcc.GetCV(CV_servoMaxTravel),
		dcc.GetCV(CV_servoLowSpeed) * 100,
		dcc.GetCV(CV_servoHighSpeed) * 100,
		servoState);

	// initlize led and relays
	SetRelays();

	// finally, kick off the bitstream capture
	bitStream.Resume();
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
    State servoState = position;
    if (servoEndPointSwap) 
        servoState = (servoState == STRAIGHT) ? CURVED : STRAIGHT;  // swap the servo endpoints if needed
    servo.Set(servoState, ServoRate);

#ifdef _DEBUG
    Serial.print("Setting servo to ");
    Serial.print(servoState, DEC);
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
	if (CV == CV_servoMinTravel) servo.SetExtent(LOW, dcc.GetCV(CV_servoMinTravel));
	if (CV == CV_servoMaxTravel) servo.SetExtent(HIGH, dcc.GetCV(CV_servoMaxTravel));
	if (CV == CV_servoLowSpeed) servo.SetDuration(LOW, dcc.GetCV(CV_servoLowSpeed) * 100);
	if (CV == CV_servoHighSpeed) servo.SetDuration(HIGH, dcc.GetCV(CV_servoHighSpeed) * 100);
}



// ========================================================================================================

TurnoutMgr* TurnoutMgr::currentInstance = 0;    // pointer to allow us to access member objects from callbacks

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

