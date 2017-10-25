
#include "TurnoutMgr.h"


// ========================================================================================================
// Public Methods


// TurnoutMgr constructor
TurnoutMgr::TurnoutMgr():
	button(ButtonPin, true),
	osStraight(OSstraightPin, true),
	osCurved(OScurvedPin, true),
	led(LedPinR, LedPinG, LedPinB),
	servoPower(ServoPowerPin),
	relayStraight(RelayStraightPin),
	relayCurved(RelayCurvedPin),
	auxOutput1(Aux1Pin),
	auxOutput2(Aux2Pin),
	//bitStream(DCCPin, false, true, 48, 68, 88, 10000, 10),    // bitstream capture object, non-standard timings
	bitStream(DCCPin, false),                           // bitstream capture object
	dccPacket(true, true, 250),                         // DCC packet builder
	dcc()                                               // DCC packet processor
{
	// set pointer to this instance of the turnout manager, so that we can reference it in callbacks
	currentInstance = this;

	// set callbacks for the bitstream capture
	bitStream.SetDataFullHandler(WrapperBitStream);
	bitStream.SetErrorHandler(WrapperBitStreamError);

	// set callbacks for the packet builder
	dccPacket.SetPacketCompleteHandler(WrapperDCCPacket);
	dccPacket.SetPacketErrorHandler(WrapperDCCPacketError);

	// configure dcc event handlers
	dcc.SetBasicAccessoryDecoderPacketHandler(WrapperDCCAccPacket);
	dcc.SetExtendedAccessoryDecoderPacketHandler(WrapperDCCExtPacket);
	dcc.SetBasicAccessoryPomPacketHandler(WrapperDCCAccPomPacket);
	dcc.SetDecodingErrorHandler(WrapperDCCDecodingError);

	// configure other event handlers
	for (byte i = 0; i < numServos; i++)
		servo[i].SetServoMoveDoneHandler(WrapperServoMoveDone);
	button.SetButtonPressHandler(WrapperButtonPress);
	osStraight.SetButtonPressHandler(WrapperOSStraight);
	osCurved.SetButtonPressHandler(WrapperOSCurved);
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


// check for new bitstream data, update sensors and outputs, check for
// max number of packet errors and reset bitstream capture if necessary
void TurnoutMgr::Update()
{
	// process any DCC interrupts that have been timestamped
	bitStream.ProcessTimestamps();

    // do the updates to maintain flashing led and slow servo motion
    unsigned long currentMillis = millis();
    led.Update(currentMillis);

	if (servosActive)
		for (byte i = 0; i < numServos; i++)
			servo[i].Update(currentMillis);

    // timer updates
    errorTimer.Update(currentMillis);
    resetTimer.Update(currentMillis);

    // update sensors
    button.Update(currentMillis);
    osStraight.Update(currentMillis);
    osCurved.Update(currentMillis);

    // check/reset error counts
    if (currentMillis - lastMillis > 1000)
    {
#ifdef _DEBUG
        Serial.print("Bit Error Count: ");
        Serial.print(bitErrorCount, DEC);
        Serial.print("     Packet Error Count: ");
        Serial.println(packetErrorCount, DEC);
#endif
		// indicate bit errors
		if ((bitErrorCount > maxBitErrors) && showErrorIndication)
		{
			// set up timer for LED indication, normal led will resume after this timer expires
			errorTimer.StartTimer(250);
			led.SetLED(RgbLed::YELLOW, RgbLed::ON);
		}

        // if we see repeated packet errors, reset bitstream capture
        if (packetErrorCount > maxPacketErrors)
        {
			// assume we lost sync on the bitstream, reset the bitstream capture
            bitStream.Suspend();
            bitStream.Resume();

			if (showErrorIndication)
			{
				// set up timer for LED indication, normal led will resume after this timer expires
				errorTimer.StartTimer(2000);
				led.SetLED(RgbLed::YELLOW, RgbLed::FLASH);
			}
		}

        lastMillis = currentMillis;
        bitErrorCount = 0;
        packetErrorCount = 0;
    }
}



// ========================================================================================================
// Private Methods


// Initialize the turnout manager by setting up the dcc config, reading stored values from CVs, and setting up the servo
void TurnoutMgr::InitMain()
{
    // Configure and initialize the DCC packet processor (accessory decoder in output address mode)
    byte cv29 = DCCdecoder::CV29_ACCESSORY_DECODER | DCCdecoder::CV29_OUTPUT_ADDRESS_MODE;
    dcc.SetupDecoder(0, 0, cv29, false);

    // get variables from cv's
    dccAddress = dcc.Address();
    servoEndPointSwap = dcc.GetCV(CV_servoEndPointSwap);
    occupancySensorSwap = dcc.GetCV(CV_occupancySensorSwap);
    dccCommandSwap = dcc.GetCV(CV_dccCommandSwap);
    relaySwap = dcc.GetCV(CV_relaySwap);

    // set the current position based on the stored position
    position = (dcc.GetCV(CV_turnoutPosition) == 0) ? STRAIGHT : CURVED;

    // servo setup - get extents, rates, and last position from cv's
    State servoState = position;
    if (servoEndPointSwap) 
        servoState = (servoState == STRAIGHT) ? CURVED : STRAIGHT;  // swap the servo endpoints if needed
    servo[0].Initialize(
        dcc.GetCV(CV_servoMinTravel),
        dcc.GetCV(CV_servoMaxTravel),
        dcc.GetCV(CV_servoLowSpeed) * 100,
        dcc.GetCV(CV_servoHighSpeed) * 100,
        servoState);

    // set led and relays, and begin bitstream capture
    EndServoMove();

#ifdef _DEBUG
    Serial.print("DCC init done, using dcc address ");
    Serial.println(dccAddress, DEC);
    Serial.println("Servo init done.");
    Serial.print("Servo position read from CVs is ");
    Serial.println(position, DEC);
#endif
}


// perform a reset to factory defaults
void TurnoutMgr::FactoryReset(bool HardReset)
{
#ifdef _DEBUG
    Serial.println("Reset to defaults initiated.");
#endif

    factoryReset = true;    // set flag indicating we are in reset
    unsigned long resetDelay = 2500;  // time to flash led so we have indication of reset occuring

    // normal initilization will resume after this timer expires
    resetTimer.StartTimer(resetDelay);
    led.SetLED(RgbLed::MAGENTA, RgbLed::FLASH);

	// suspend bitstream in case of soft reset
	bitStream.Suspend();

	// reset vars for soft reset
	bitErrorCount = 0;
	packetErrorCount = 0;
	lastMillis = 0;
	showErrorIndication = true;

	// do the cv reset
    unsigned int numCVs = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
    for (unsigned int cv = 0; cv < numCVs; cv++)
    {
		if (HardReset || FactoryDefaultCVs[cv].SoftReset)
	       dcc.SetCV(FactoryDefaultCVs[cv].CV, FactoryDefaultCVs[cv].Value);
    }

#ifdef _DEBUG
    Serial.println("Reset to defaults completed.");
#endif
}


// set the turnout to a new position
void TurnoutMgr::BeginServoMove()
{
    // store new position to cv
    dcc.SetCV(CV_turnoutPosition,position);

	// configure the servo settings
	for (byte i = 0; i < numServos; i++)
	{
		servoState[i] = position;
		if (servoEndPointSwap)
			servoState[i] = (servoState[i] == STRAIGHT) ? CURVED : STRAIGHT;  // swap the servo endpoints if needed
	}

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
		EndServoMove();
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


// handle a DCC extended accessory command, used for controlling the aux outputs
void TurnoutMgr::DCCExtCommandHandler(unsigned int Addr, unsigned int Data)
{
	// assume we are filtering repeated packets in the packet builder, so we don't check for that here
	// assume DCCdecoder is set to return only packets for this decoder's address.

#ifdef _DEBUG
	Serial.print("Received dcc signal apsect command, value ");
	Serial.println(Data, DEC);
#endif

	// process a matching signal aspect to turn aux outputs on or off
	if (Data == dcc.GetCV(CV_Aux1Off))
	{
		auxOutput1.SetPin(LOW);
		return;
	}

	if (Data == dcc.GetCV(CV_Aux1On))
	{
		auxOutput1.SetPin(HIGH);
		return;
	}

	if (Data == dcc.GetCV(CV_Aux2Off))
	{
		auxOutput2.SetPin(LOW);
		return;
	}

	if (Data == dcc.GetCV(CV_Aux2On))
	{
		auxOutput2.SetPin(HIGH);
		return;
	}


	// process a matching signal aspect to toggle error indication
	if (Data == dcc.GetCV(CV_errorIndicationToggle))
	{
		showErrorIndication = !showErrorIndication;

		// set up timer for LED indication, normal led will resume after this timer expires
		errorTimer.StartTimer(1000);
		led.SetLED(RgbLed::BLUE, RgbLed::ON);
		return;
	}


	// an invalid signal aspect was received, provide an error indication
	errorTimer.StartTimer(1000);
	led.SetLED(RgbLed::YELLOW, RgbLed::ON);
}


// handle setting CVs
void TurnoutMgr::DCCPomHandler(unsigned int Addr, byte instType, unsigned int CV, byte Value)
{
    // assume we are filtering repeated packets in the packet builder, so we don't check for that here
	// assume DCCdecoder is set to return only packets for this decoder's address.

#ifdef _DEBUG
    Serial.print("In class callback for dcc program on main, CV: ");
    Serial.print(CV, DEC);
    Serial.print(", Value: ");
    Serial.println(Value, DEC);
#endif

	// check for and perform cv commanded reset
	if (CV == CV_reset)
	{
		if (Value == CV_softResetValue)
		{
			FactoryReset(false);
			return;
		}
		if (Value == CV_hardResetValue)
		{
			FactoryReset(true);
			return;
		}
	}

    // check against our defined CVs to verify that the CV is valid
    boolean isValidCV = false;
    unsigned int numCVs = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
    for (unsigned int i = 0; i < numCVs; i++)
    {
        if (CV == FactoryDefaultCVs[i].CV) isValidCV = true;
    }

    // set up timer for LED indication, normal led will resume after this timer expires
    errorTimer.StartTimer(1000);

    // provide indication and exit if CV is invalid
    if (!isValidCV)
    {
        led.SetLED(RgbLed::YELLOW, RgbLed::ON);
        return;
    }

    // provide feedback that we are programming a valid CV
    led.SetLED(RgbLed::BLUE, RgbLed::ON);

    // set the cv
    dcc.SetCV(CV, Value);

    // read back values from eeprom
    dccAddress = dcc.Address();
    servoEndPointSwap = dcc.GetCV(CV_servoEndPointSwap);
    occupancySensorSwap = dcc.GetCV(CV_occupancySensorSwap);
    dccCommandSwap = dcc.GetCV(CV_dccCommandSwap);
    relaySwap = dcc.GetCV(CV_relaySwap);

    // update servo vars from eeprom
    if (CV == CV_servoMinTravel) servo[0].SetExtent(LOW,dcc.GetCV(CV_servoMinTravel));
	if (CV == CV_servoMaxTravel) servo[0].SetExtent(HIGH,dcc.GetCV(CV_servoMaxTravel));
	if (CV == CV_servoLowSpeed) servo[0].SetDuration(LOW,dcc.GetCV(CV_servoLowSpeed) * 100);
	if (CV == CV_servoHighSpeed) servo[0].SetDuration(HIGH,dcc.GetCV(CV_servoHighSpeed) * 100);
}



// callbacks for bitstream and packet builder =============================================================

TurnoutMgr* TurnoutMgr::currentInstance=0;    // pointer to allow us to access member objects from callbacks

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


// ========================================================================================================
// sensor/timer callback wrappers

void TurnoutMgr::WrapperButtonPress(bool ButtonState) { currentInstance->ButtonEventHandler(ButtonState); }
void TurnoutMgr::WrapperServoMoveDone() { currentInstance->ServoMoveDoneHandler(); }
void TurnoutMgr::WrapperResetTimer() { currentInstance->ResetTimerHandler(); }
void TurnoutMgr::WrapperErrorTimer() { currentInstance->ErrorTimerHandler(); }
void TurnoutMgr::WrapperOSStraight(bool ButtonState) { currentInstance->OSStraightHandler(ButtonState); }
void TurnoutMgr::WrapperOSCurved(bool ButtonState) { currentInstance->OSCurvedHandler(ButtonState); }


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
    Serial.println(errorCode,DEC);
#endif
}

