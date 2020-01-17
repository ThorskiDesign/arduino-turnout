/*

This file is part of Arduino Turnout
Copyright (C) 2017-2018 Eric Thorstenson

Arduino Turnout is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Arduino Turnout is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.

*/

#include "TurnoutBase.h"


// ========================================================================================================
// Public Methods


// TurnoutMgr constructor
TurnoutBase::TurnoutBase()
{
}


// check for new bitstream data, update sensors and outputs, check for
// max number of packet errors and reset bitstream capture if necessary
void TurnoutBase::Update()
{
	// process any DCC interrupts that have been timestamped
	dcc.ProcessTimeStamps();

	// do the updates to maintain flashing led and slow servo motion
	const unsigned long currentMillis = millis();
	led.Update(currentMillis);

	// timer updates
	errorTimer.Update(currentMillis);
	resetTimer.Update(currentMillis);
	servoTimer.Update(currentMillis);

	// update sensors
	button.Update(currentMillis);
}


// ========================================================================================================
// Private Methods


// Initialize the turnout manager by setting up the dcc config, reading stored values from CVs, and setting up the servo
void TurnoutBase::InitMain()
{

	// configure factory default CVs
	byte index = 0;
	index = cv.initCV(index, CV_AddressLSB, 1, 0, 255, false);
	index = cv.initCV(index, CV_AddressMSB, 0, 0, 255, false);
	index = cv.initCV(index, CV_servo1MinTravel, 90, 45, 135, false);
	index = cv.initCV(index, CV_servo1MaxTravel, 90, 45, 135, false);
	index = cv.initCV(index, CV_servoLowSpeed, 25);
	index = cv.initCV(index, CV_servoHighSpeed, 0);
	index = cv.initCV(index, CV_occupancySensorSwap, 0);
	index = cv.initCV(index, CV_dccCommandSwap, 0);
	index = cv.initCV(index, CV_relaySwap, 0);
	index = cv.initCV(index, CV_Aux1Off, 10);
	index = cv.initCV(index, CV_Aux1On, 11);
	index = cv.initCV(index, CV_Aux2Off, 20);
	index = cv.initCV(index, CV_Aux2On, 21);
	index = cv.initCV(index, CV_positionIndicationToggle, 1);
	index = cv.initCV(index, CV_errorIndicationToggle, 2);
	index = cv.initCV(index, CV_turnoutPosition, 0, 0, 1, false);
	index = cv.initCV(index, CV_servo2MinTravel, 90, 45, 135, false);
	index = cv.initCV(index, CV_servo2MaxTravel, 90, 45, 135, false);
	index = cv.initCV(index, CV_servo3MinTravel, 90, 45, 135, false);
	index = cv.initCV(index, CV_servo3MaxTravel, 90, 45, 135, false);
	index = cv.initCV(index, CV_servo4MinTravel, 90, 45, 135, false);
	cv.initCV(index, CV_servo4MaxTravel, 90, 45, 135, false);

	// load config
	LoadConfig();

	// Initialize the DCC decoder
	byte addr = (cv.getCV(CV_AddressMSB) << 8) + cv.getCV(CV_AddressLSB);
	dcc.SetAddress(addr);

	// get variables from cv's
	occupancySensorSwap = cv.getCV(CV_occupancySensorSwap);
	dccCommandSwap = cv.getCV(CV_dccCommandSwap);
	relaySwap = cv.getCV(CV_relaySwap);

	// set the current position based on the stored position
	position = (cv.getCV(CV_turnoutPosition) == 0) ? STRAIGHT : CURVED;
}


// perform a reset to factory defaults
void TurnoutBase::FactoryReset(bool HardReset)
{
	// normal initilization will resume after this timer expires
	const unsigned long resetDelay = 2500;  // time to flash led so we have indication of reset occuring
	resetTimer.StartTimer(resetDelay);
	led.SetLED(RgbLed::MAGENTA, RgbLed::FLASH);

	// suspend bitstream in case of soft reset
	dcc.SuspendBitstream();

	// do the cv reset
	cv.resetCVs();
	SaveConfig();
}


// ========================================================================================================
// Event Handlers


// handle the error timer callback
void TurnoutBase::ErrorTimerHandler()
{
	// all we need to do here is turn the led back on normally
	led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::ON);
}

void TurnoutBase::MaxBitErrorHandler()
{
	if (!showErrorIndication) return;

	// set up timer for LED indication, normal led will resume after this timer expires
	errorTimer.StartTimer(250);
	led.SetLED(RgbLed::YELLOW, RgbLed::ON);
}

void TurnoutBase::MaxPacketErrorHandler()
{
	if (!showErrorIndication) return;

	// set up timer for LED indication, normal led will resume after this timer expires
	errorTimer.StartTimer(500);
	led.SetLED(RgbLed::YELLOW, RgbLed::ON);
}

void TurnoutBase::DCCDecodingError()
{
	if (!showErrorIndication) return;

	// set up timer for LED indication, normal led will resume after this timer expires
	errorTimer.StartTimer(1000);
	led.SetLED(RgbLed::CYAN, RgbLed::FLASH);
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
	if (Data == cv.getCV(CV_Aux1Off))
	{
		auxOutput1.SetPin(LOW);
		return;
	}

	if (Data == cv.getCV(CV_Aux1On))
	{
		auxOutput1.SetPin(HIGH);
		return;
	}

	if (Data == cv.getCV(CV_Aux2Off))
	{
		auxOutput2.SetPin(LOW);
		return;
	}

	if (Data == cv.getCV(CV_Aux2On))
	{
		auxOutput2.SetPin(HIGH);
		return;
	}


	// process a matching signal aspect to toggle error indication
	if (Data == cv.getCV(CV_errorIndicationToggle))
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

	// set the cv
	if (cv.setCV(CV, Value))
	{
		// provide feedback that we are programming a valid CV
		errorTimer.StartTimer(1000);
		led.SetLED(RgbLed::BLUE, RgbLed::ON);
	}
	else
	{
		// or provide indication if CV is invalid
		errorTimer.StartTimer(1000);
		led.SetLED(RgbLed::YELLOW, RgbLed::ON);
	}

	SaveConfig();

	// read back values from cv manager
	occupancySensorSwap = cv.getCV(CV_occupancySensorSwap);
	dccCommandSwap = cv.getCV(CV_dccCommandSwap);
	relaySwap = cv.getCV(CV_relaySwap);
}


void TurnoutBase::LoadConfig()
{
	const bool firstBoot = (EEPROM.read(0) == 255);    // default value for unwritten eeprom

	if (firstBoot)
	{ 

		// reset cvs to defaults and save
		cv.resetCVs();
		SaveConfig();
	}
	else
	{

		// load stored config struct
		EEPROM.get(0, configVars);

		// copy stored config to working CVs
		for (byte i = 0; i < numCVindexes; i++)
		{

			cv.cv[i].cvValue = configVars.CVs[i];
		}
	}
}

void TurnoutBase::SaveConfig()
{
	// copy working CVs to our storage object
	for (byte i = 0; i < numCVindexes; i++)
	{
		configVars.CVs[i] = cv.cv[i].cvValue;

	}

	// store the config
	EEPROM.put(0, configVars);
}
