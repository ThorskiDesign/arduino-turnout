
#include "TurnoutBase.h"




// ========================================================================================================
// Public Methods


// TurnoutMgr constructor
TurnoutBase::TurnoutBase() :
	button(ButtonPin, true),
	led(LedRPin, LedGPin, LedBPin),
	servoPower(ServoPowerPin),
	auxOutput1(Aux1Pin),
	auxOutput2(Aux2Pin),
	bitStream(),
	dccPacket(true, true, 250),                         // DCC packet builder
	dcc()                                               // DCC packet processor
{
}


// check for new bitstream data, update sensors and outputs, check for
// max number of packet errors and reset bitstream capture if necessary
void TurnoutBase::Update()
{
	// process any DCC interrupts that have been timestamped
	bitStream.ProcessTimestamps();

	// do the updates to maintain flashing led and slow servo motion
	const unsigned long currentMillis = millis();
	led.Update(currentMillis);

	// timer updates
	errorTimer.Update(currentMillis);
	resetTimer.Update(currentMillis);
	servoTimer.Update(currentMillis);

	// update sensors
	button.Update(currentMillis);

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
void TurnoutBase::InitMain()
{
	// Configure and initialize the DCC packet processor (accessory decoder in output address mode)
	const byte cv29 = DCCdecoder::CV29_ACCESSORY_DECODER | DCCdecoder::CV29_OUTPUT_ADDRESS_MODE;
	dcc.SetupDecoder(0, 0, cv29, false);

	// get variables from cv's
	dccAddress = dcc.Address();
	occupancySensorSwap = dcc.GetCV(CV_occupancySensorSwap);
	dccCommandSwap = dcc.GetCV(CV_dccCommandSwap);
	relaySwap = dcc.GetCV(CV_relaySwap);

	// set the current position based on the stored position
	position = (dcc.GetCV(CV_turnoutPosition) == 0) ? STRAIGHT : CURVED;

#ifdef _DEBUG
	Serial.print("Base init done, using dcc address ");
	Serial.println(dccAddress, DEC);
	Serial.print("Servo position read from CVs is ");
	Serial.println(position, DEC);
#endif
}


// perform a reset to factory defaults
void TurnoutBase::FactoryReset(bool HardReset)
{
#ifdef _DEBUG
	Serial.println("Reset to defaults initiated.");
#endif

	factoryReset = true;    // set flag indicating we are in reset
	const unsigned long resetDelay = 2500;  // time to flash led so we have indication of reset occuring

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
	const unsigned int numCVs = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
	for (unsigned int cv = 0; cv < numCVs; cv++)
	{
		if (HardReset || FactoryDefaultCVs[cv].SoftReset)
			dcc.SetCV(FactoryDefaultCVs[cv].CV, FactoryDefaultCVs[cv].Value);
	}

#ifdef _DEBUG
	Serial.println("Reset to defaults completed.");
#endif
}



// ========================================================================================================
// Event Handlers


// handle the reset timer callback
void TurnoutBase::ResetTimerHandler()
{
	// run the main init after the reset timer expires
	factoryReset = false;
	InitMain();
}


// handle the error timer callback
void TurnoutBase::ErrorTimerHandler()
{
	// all we need to do here is turn the led back on normally
	led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::ON);
}


// handle a DCC extended accessory command, used for controlling the aux outputs
void TurnoutBase::DCCExtCommandHandler(unsigned int Addr, unsigned int Data)
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


// handle a DCC program on main command
void TurnoutBase::DCCPomHandler(unsigned int Addr, byte instType, unsigned int CV, byte Value)
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
	const unsigned int numCVs = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
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
	occupancySensorSwap = dcc.GetCV(CV_occupancySensorSwap);
	dccCommandSwap = dcc.GetCV(CV_dccCommandSwap);
	relaySwap = dcc.GetCV(CV_relaySwap);
}
