
#include "TurnoutMgr.h"


// ========================================================================================================
// Public Methods


// TurnoutMgr constructor
TurnoutMgr::TurnoutMgr():
	button(ButtonPin, true),
	osStraight(OSstraightPin, true),
	osCurved(OScurvedPin, true),
	led(LedPinR, LedPinG, LedPinB),
	servo(ServoPWMPin, ServoPowerPin),
	relayStraight(RelayStraightPin),
	relayCurved(RelayCurvedPin)
{
	// TODO: why can we initialize member classes here e.g. servo.initialize ?
	// e.g., why didn't this work in constructor - servo.Initialize(45, 110, LOW);
}


// Check for factory reset, then proceed with main initialization
void TurnoutMgr::Initialize()
{
	// check for button hold on startup (for reset to defaults)
	if (button.RawState() == LOW) 
	{
		FactoryReset();
	}
	else
	{
		InitMain();
	}
}


// Run the dcc process update. this needs to be run from loop().
void TurnoutMgr::UpdateDccProcess()
{ 
	dcc.process();
}


// update the sensors and outputs.
void TurnoutMgr::UpdateSensors()
{
	unsigned long currentMillis = millis();

	// do the updates to maintain flashing led and slow servo motion
	servo.Update(currentMillis);
	led.Update(currentMillis);

	// timer updates
	errorTimer.Update(currentMillis);
	resetTimer.Update(currentMillis);

	// update sensors
	button.Update(currentMillis);
	osStraight.Update(currentMillis);
	osCurved.Update(currentMillis);
}



// ========================================================================================================
// Private Methods


// Initialize the turnout manager by setting up the dcc config, reading stored values from CVs, and setting up the servo
void TurnoutMgr::InitMain()
{
	// Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up
	dcc.pin(0, DCCPin, 0);

	// Call the main DCC Init function to enable the DCC Receiver
	dcc.init( MAN_ID_DIY, 10, CV29_ACCESSORY_DECODER, 0 );
	Serial.println("DCC init done.");

	// get variables from cv's
	dccAddress = dcc.getAddr();
	servoEndPointSwap = dcc.getCV(CV_servoEndPointSwap);
	occupancySensorSwap = dcc.getCV(CV_occupancySensorSwap);
	dccCommandSwap = dcc.getCV(CV_dccCommandSwap);
	relaySwap = dcc.getCV(CV_relaySwap);

	Serial.print("Parameters read from CVs, using dcc address ");
	Serial.println(dccAddress, DEC);

	// set the current position based on the stored position
	position = (dcc.getCV(CV_turnoutPosition) == 0) ? STRAIGHT : CURVED;

	// servo setup - get extents, rates, and last position from cv's
	State servoState = position;
	if (servoEndPointSwap) servoState = (State) !servoState;  // swap the servo endpoints if needed
	servo.Initialize(
		dcc.getCV(CV_servoMinTravel),
		dcc.getCV(CV_servoMaxTravel),
		dcc.getCV(CV_servoLowSpeed) * 100,
		dcc.getCV(CV_servoHighSpeed) * 100,
		servoState);
	Serial.println("Servo init done.");
	Serial.print("Servo position read from CVs is ");
	Serial.println(position, DEC);

	// configure the event handlers
	button.SetButtonPressHandler(&HandleButtonPressWrapper);
	servo.SetServoMoveDoneHandler(&HandleServoMoveDoneWrapper);
	servo.SetServoPowerOffHandler(&HandleServoPowerOffWrapper);
	osStraight.SetButtonPressHandler(&HandleOSStraightWrapper);
	osCurved.SetButtonPressHandler(&HandleOSCurvedWrapper);
	errorTimer.SetTimerHandler(&HandleErrorTimerWrapper);

	// initlize led and relays
	SetRelays();
}


// perform a reset to factory defaults
void TurnoutMgr::FactoryReset()
{
	Serial.println("Reset to defaults initiated.");

	factoryReset = true;    // set flag indicating we are in reset
	unsigned long resetDelay = 2500;  // time to flash led so we have indication of reset occuring
	
	// normal initilization will resume after this timer expires
	resetTimer.StartTimer(resetDelay);
	resetTimer.SetTimerHandler(&HandleResetTimerWrapper);

	led.SetLED(RgbLed::MAGENTA, RgbLed::FLASH);

	// do the reset
	if(dcc.isSetCVReady())
	{
		unsigned int numCVs = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
		for (unsigned int cv = 0; cv < numCVs; cv++)
		{
			dcc.setCV( FactoryDefaultCVs[cv].CV, FactoryDefaultCVs[cv].Value);
		}
	}

	Serial.println("Reset to defaults completed.");
}


// set the led and relays for the current position
void TurnoutMgr::SetRelays()
{
	// set the led solid for the current position
	led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::ON);

	// enable the appropriate relay, swapping if needed
	if ((position == STRAIGHT && !relaySwap) || (position == CURVED && relaySwap)) 
	{
		relayStraight.SetRelay(HIGH);
	}

	if ((position == CURVED && !relaySwap) || ((position == STRAIGHT) && relaySwap))
	{
		relayCurved.SetRelay(HIGH);
	}
}


// set the turnout to a new position (also disables relays prior to starting servo motion)
void TurnoutMgr::SetServo(bool ServoRate)
{
	// turn off the relays
	relayStraight.SetRelay(LOW);
	relayCurved.SetRelay(LOW);

	// set the servo
	State servoState = position;
	if (servoEndPointSwap) servoState = (State) !servoState;  // swap the servo endpoints if needed
	servo.Set(servoState, ServoRate);

#ifdef _DEBUG
	Serial.print("Setting servo to ");
	Serial.print(servoState, DEC);
	Serial.print(" at rate ");
	Serial.println(ServoRate, DEC);
#endif

	// store new position to cv
	dcc.setCV(CV_turnoutPosition,position);
}



// ========================================================================================================
// Event Handlers


// handle the reset timer callback
void TurnoutMgr::ResetTimerHandler()
{
	// run the main init after the reset timer expires
	factoryReset = false;
	InitMain();
}


// handle the error timer callback
void TurnoutMgr::ErrorTimerHandler()
{
	// all we need to do here is turn the led back on normally
	led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::ON);
}


// do things after the servo is powered off
void TurnoutMgr::ServoPowerOffHandler() { }


// do things after the servo finishes moving to its new position
void TurnoutMgr::ServoMoveDoneHandler() { SetRelays(); }


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
			// button error indication
			led.SetLED(RgbLed::BLUE, RgbLed::FLASH);

			// normal led will resume after this timer expires
			errorTimer.StartTimer(2500);
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


// handle the dcc interpreter callback
void TurnoutMgr::DCCcommandHandler(unsigned int Addr, unsigned int Direction)
{
	if (Addr != dccAddress) return;   // exit if this command is not for our address

	State dccState;
	dccState = (Direction == 0) ? CURVED : STRAIGHT;
	if (dccCommandSwap) dccState = (State) !dccState; // swap the interpretation of dcc command if needed

	// exit if receiving the same dcc command sequentially (cuts down on dcc spam)
	if (dccState == position) return;

#ifdef _DEBUG
	Serial.print("In class callback for dcc command direction ");
	Serial.println(Direction, DEC);
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
		// command error indication
		led.SetLED(RgbLed::BLUE, RgbLed::FLASH);

		// normal led will resume after this timer expires
		errorTimer.StartTimer(2500);
	}
}


// handle setting CVs
void TurnoutMgr::CVchangeHandler(unsigned int CV, unsigned int Value)
{
	// we don't read back the stored position, since this updates every time the switch changes
	if (factoryReset || CV == CV_turnoutPosition) return;

	// provide feedback that we are programming
	led.SetLED(RgbLed::YELLOW, RgbLed::ON);

	// normal led will resume after this timer expires
	errorTimer.StartTimer(1000);
	errorTimer.SetTimerHandler(&HandleErrorTimerWrapper);

	// set the cv
	if(dcc.isSetCVReady()) dcc.setCV(CV, Value);

	// read back values from eeprom
	dccAddress = dcc.getAddr();
	servoEndPointSwap = dcc.getCV(CV_servoEndPointSwap);
	occupancySensorSwap = dcc.getCV(CV_occupancySensorSwap);
	dccCommandSwap = dcc.getCV(CV_dccCommandSwap);
	relaySwap = dcc.getCV(CV_relaySwap);

	// read back and update servo vars from eeprom
	servo.SetExtent(LOW,dcc.getCV(CV_servoMinTravel));
	servo.SetExtent(HIGH,dcc.getCV(CV_servoMaxTravel));
	servo.SetDuration(LOW,dcc.getCV(CV_servoLowSpeed) * 100);
	servo.SetDuration(HIGH,dcc.getCV(CV_servoHighSpeed) * 100);
}