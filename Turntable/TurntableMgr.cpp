
#include "TurntableMgr.h"

// static pointer for callbacks
TurntableMgr* TurntableMgr::currentInstance = 0;

TurntableMgr::TurntableMgr()
{
	// pointer for callback functions
	currentInstance = this;
}

void TurntableMgr::Initialize()
{
	// load config and last saved state
	LoadConfig();
	LoadState();
	
	// setup the stepper
	configureStepper();

	// configure callbacks
	idleTimer.SetTimerHandler(WrapperIdleTimerHandler);
	warmupTimer.SetTimerHandler(WrapperWarmupTimerHandler);

	#if defined(WITH_TOUCHSCREEN)
	touchpad.Init();
	touchpad.SetGraphicButtonHandler(WrapperGraphicButtonHandler);
	#endif // defined(WITH_TOUCHSCREEN)

	// start the state machine
	if (currentState == MOVING || currentState == SEEK)
		// we shutdown without reaching our destination, so stepper position is unknown
	{
		currentState = SEEK;
		currentStateFunction = &TurntableMgr::stateSeek;
	}
	else
		// normal startup
	{
		currentState = IDLE;
		currentStateFunction = &TurntableMgr::stateIdle;
	}

}


void TurntableMgr::Update()
{
	// perform the current state function
	if (ttStateFunctions[currentState])
		(*this.*ttStateFunctions[currentState])();
}


//  state transition functions  ===================================================================

void TurntableMgr::stateIdle()
{
	if (subState == 0)     // transition to state
	{
		// TODO: go to full step before powering off?

		afStepper->release();                // power off the stepper
		flasher.SetLED(RgbLed::OFF);         // turn off the warning light

		#if defined(WITH_DCC)
		dcc.ResumeBitstream();               // resume listening for dcc commands
		#endif // defined(WITH_DCC)

		subState = 1;
	}

	// do the update functions for this state

	#if defined(WITH_DCC)
	dcc.ProcessTimeStamps();
	#endif // defined(WITH_DCC)

	#if defined(WITH_TOUCHSCREEN)
	touchpad.Update();
	#endif // defined(WITH_TOUCHSCREEN)
}


void TurntableMgr::stateMoving()
{
	if (subState == 0)     // transition to state
	{

		#if defined(WITH_DCC)
		dcc.SuspendBitstream();
		#endif   // WITH_DCC

		// move to the specified siding at normal speed
		accelStepper.setMaxSpeed(stepperMaxSpeed);
		accelStepper.setAcceleration(stepperAcceleration);
		moveToSiding();

		subState = 1;
	}

	// do the update functions for this state
	uint32_t currentMillis = millis();
	accelStepper.run();
	flasher.Update(currentMillis);

	#if defined(WITH_TOUCHSCREEN)
	touchpad.Update();
	#endif // defined(WITH_TOUCHSCREEN)

	if (accelStepper.distanceToGo() == 0) raiseEvent(MOVE_DONE);
}

void TurntableMgr::stateSeek()
{
	switch (subState)
	{
	case 0:                 // transition to seek state

		#if defined(WITH_DCC)
		dcc.SuspendBitstream();
		#endif

		flasher.SetLED(RgbLed::RED, RgbLed::FLASH);

		// start moving in a complete clockwise circle at normal speed
		accelStepper.setMaxSpeed(stepperMaxSpeed);
		accelStepper.setAcceleration(stepperAcceleration);
		accelStepper.move(2 * halfCircleSteps);

		subState = 1;
		break;
	case 1:    // CW rotation, waiting for hall sensor to go low, indicating magnet has entered its detection area
		if (!hallSensor.SwitchState()) subState = 2;   // found it, go to next state
		break;
	case 2:    // CW rotation, waiting for sensor to go high, indicating we've swung past in the CW direction
		if (hallSensor.SwitchState())
		{
			accelStepper.stop();     // set the stepper to stop with its current speed/accel settings
			subState = 3;
		}
		break;
	case 3:    // waiting for CW motion to stop
		if (accelStepper.distanceToGo() == 0)
		{
			// start moving in a counter clockwise halfcircle at low speed
			accelStepper.setMaxSpeed(stepperLowSpeed);
			accelStepper.setAcceleration(stepperAcceleration);
			accelStepper.move(-1L * halfCircleSteps);

			subState = 4;
		}
		break;
	case 4:    //  CCW rotation, waiting for sensor to go low
		if (!hallSensor.SwitchState()) subState = 5;
		break;
	case 5:    //  CCW rotation, waiting for sensor to go high
		if (hallSensor.SwitchState())
		{
			accelStepper.setCurrentPosition(0);    // we found our zero position, so set it
												   // TODO: use the nearest full step value
			//accelStepper.stop();                   // set the stepper to stop with its current speed/accel settings

			subState = 6;
		}
		break;
		//case 6:   // CCW rotation, waiting for motion to stop
		//	if (accelStepper.distanceToGo() == 0)
		//		raiseEvent(MOVE_DONE);      // motion has stopped, raise event to exit seek state
		//	break;
	default:
		break;
	}

	// TODO: add timeout/error check in case we somehow miss one of the substate steps
	// When we stop after substate 5, or if we miss a hall sensor event and go full circle, end the move
	if (accelStepper.distanceToGo() == 0)
		raiseEvent(MOVE_DONE);      // motion has stopped, raise event to exit seek state

	// do the update functions for this state
	uint32_t currentMillis = millis();
	accelStepper.run();
	flasher.Update(currentMillis);
	hallSensor.Update(currentMillis);
}

void TurntableMgr::statePowered()
{
	if (subState == 0)     // transition to state
	{

		#if defined(WITH_DCC)
		dcc.ResumeBitstream();               // resume listening for dcc commands
		#endif // deinfed(WITH_DCC)

		// start the timer for the transition to idle state
		idleTimer.StartTimer(idleTimeout);
		subState = 1;
	}

	// do the update functions for this state

	#if defined(WITH_DCC)
	dcc.ProcessTimeStamps();
	#endif // deinfed(WITH_DCC)

	#if defined(WITH_TOUCHSCREEN)
	touchpad.Update();
	#endif // defined(WITH_TOUCHSCREEN)

	uint32_t currentMillis = millis();
	flasher.Update(currentMillis);
	idleTimer.Update(currentMillis);
}

void TurntableMgr::stateWarmup()
{
	if (subState == 0)     // transition to state
	{

		#if defined(WITH_DCC)
		dcc.SuspendBitstream();
		#endif   // WITH_DCC

		flasher.SetLED(RgbLed::RED, RgbLed::FLASH);

		// start the timer for the transition to moving state
		warmupTimer.StartTimer(warmupTimeout);
		flasher.SetLED(RgbLed::RED, RgbLed::FLASH);
		subState = 1;
	}

	// do the update functions for this state
	uint32_t currentMillis = millis();
	flasher.Update(currentMillis);
	warmupTimer.Update(currentMillis);
}


void TurntableMgr::raiseEvent(const ttEvent event)
{
	byte i = 0;
	const byte n = sizeof(stateTransMatrix) / sizeof(stateTransMatrixRow);

	// loop through state transition table until we find matching state and event
	while ((i < n) && !(currentState == stateTransMatrix[i].currState && event == stateTransMatrix[i].event)) i++;

	// if there was no match just return
	if (i == n)	return;

	// otherwise transition to next state specified in the matching row
	currentState = stateTransMatrix[i].nextState;
	currentStateFunction = ttStateFunctions[currentState];
	subState = 0;
}



//  local functions   ===========================================================================

void TurntableMgr::configureStepper()
{
	// motorshield and stepper objects
	motorShield = Adafruit_MotorShield();
	afStepper = motorShield.getStepper(stepperStepsPerRev, motorShieldPort);
	accelStepper = AccelStepper(StepperClockwiseStep, StepperCounterclockwiseStep);

	// adafruit motorshield setup
	motorShield.begin();                            // create with the default PWM frequency 1.6KHz
	//TWBR = ((F_CPU / 400000l) - 16) / 2;        // change I2C clock to 400kHz
	Wire.setClock(400000l);                       // TODO: check that this really works on AVR and ARM

	// accelstepper setup
	accelStepper.setMaxSpeed(stepperMaxSpeed);
	accelStepper.setAcceleration(stepperAcceleration);
}


void TurntableMgr::moveToSiding()
{
	// get stepper position for desired siding
	const int32_t targetPos = configVars.sidingSteps[currentSiding].cvValue;

	// get current stepper position in positive half circle equivalent
	const uint16_t currentPos = findBasicPosition(accelStepper.currentPosition());

	// now figure out the shortest rotation
	const int32_t deltaCW = targetPos - currentPos;
	const int32_t deltaCCW = (targetPos - halfCircleSteps) - currentPos;

	// nothing to do, just return
	if (deltaCW == 0 || deltaCCW == 0) return;

	// start the move
	if (abs(deltaCW) < abs(deltaCCW))
	{
		accelStepper.move(deltaCW);
	}
	else
	{
		accelStepper.move(deltaCCW);
	}
}


void TurntableMgr::reverseSiding()
{
	// get current stepper position in positive half circle equivalent
	const uint16_t currentPos = findBasicPosition(accelStepper.currentPosition());

	// add a half circle of steps to that
	const uint16_t targetPos = currentPos + halfCircleSteps;

	// start the move
	accelStepper.moveTo(targetPos);
}


// convert stepper motor position to a positive position in the first half circle of rotation
uint16_t TurntableMgr::findBasicPosition(int32_t pos)
{
	int32_t p = pos;

	if (p < 0)     // convert negative motor position
	{
		byte n = -p / halfCircleSteps;
		p += (1 + n) * halfCircleSteps;
	}

	return p % halfCircleSteps;
}

// return the nearest full step to the current position, in microsteps
int32_t TurntableMgr::findFullStep(int32_t microsteps)
{
	byte remainder = microsteps % stepperMicroSteps;

	if (remainder < 8)
	{
		return microsteps - remainder;
	}
	else
	{
		return microsteps + (stepperMicroSteps - remainder);
	}
}



// event handlers and static wrappers

void TurntableMgr::WrapperIdleTimerHandler() { currentInstance->raiseEvent(IDLETIMER); }
void TurntableMgr::WrapperWarmupTimerHandler() { currentInstance->raiseEvent(WARMUPTIMER); }

void TurntableMgr::WrapperGraphicButtonHandler(byte buttonID, bool state)
{
	currentInstance->GraphicButtonHandler(buttonID, state);
}


uint16_t TurntableMgr::getCV(byte cv)
{
	byte i = 0;
	while (configVars.CVs[i].cvNum != i) i++;
	return configVars.CVs[i].cvValue;
}

void TurntableMgr::SaveConfig()
{
	#if !defined(ADAFRUIT_METRO_M0_EXPRESS)
	EEPROM.put(sizeof(stateVars), configVars);   // store the config vars struct starting after the state vars in EEPROM
	#endif
}

void TurntableMgr::SaveState()
{
	#if !defined(ADAFRUIT_METRO_M0_EXPRESS)
	EEPROM.put(0, stateVars);
	#endif
}

void TurntableMgr::LoadConfig()
{
	LoadConfig(false);
}

void TurntableMgr::LoadConfig(bool reset)
{
	#if !defined(ADAFRUIT_METRO_M0_EXPRESS)
	const bool firstBoot = (EEPROM.read(0) == 255);
	if (firstBoot || reset)   // if this is the first boot on fresh eeprom, or reset requested
	{
		//load default values for config vars and sidings
		for (byte i = 0; i < numCVs; i++)
			configVars.CVs[i].cvValue = configVars.CVs[i].cvDefault;
		for (byte i = 0; i < numSidings; i++)
			configVars.sidingSteps[i].cvValue = configVars.sidingSteps[i].cvDefault;
	}
	else
	{
		EEPROM.get(sizeof(stateVars), configVars);   // load the config vars struct starting after the state vars in EEPROM
	}
	#endif
}

void TurntableMgr::LoadState()
{
	#if !defined(ADAFRUIT_METRO_M0_EXPRESS)
	const bool firstBoot = (EEPROM.read(0) == 255);
	if (!firstBoot)
	{
		EEPROM.get(0, stateVars);    // get the last state from eeprom
	}
	#endif
}


void TurntableMgr::GraphicButtonHandler(byte buttonID, bool state)
{

	if (state)        // button was pressed
	{
		switch (buttonID)
		{
		case Touchpad::numpad0:
		case Touchpad::numpad1:
		case Touchpad::numpad2:
		case Touchpad::numpad3:
		case Touchpad::numpad4:
		case Touchpad::numpad5:
		case Touchpad::numpad6:
		case Touchpad::numpad7:
		case Touchpad::numpad8:
		case Touchpad::numpad9:
			if (currentPage == pageRun)
			{
				currentSiding = buttonID;
				raiseEvent(BUTTON_SIDING);
			}

			// TODO: how do we want to handle pressing siding button while in setup mode?

			break;
		case Touchpad::modeRun:
			currentPage = pageRun;
			break;
		case Touchpad::modeSetup:
			currentPage = pageSetup;
			break;
		case Touchpad::runReverse:
			break;
		case Touchpad::setupCW:
			break;
		case Touchpad::setupCCW:
			break;
		case Touchpad::setupSet:
			break;
		case Touchpad::setupHome:
			raiseEvent(BUTTON_SEEK);
			break;
		default:
			break;
		}


	}
	else             // button was released
	{

	}

}


void TurntableMgr::StepperClockwiseStep()
{
	currentInstance->afStepper->onestep(FORWARD, MICROSTEP);
}

void TurntableMgr::StepperCounterclockwiseStep()
{
	currentInstance->afStepper->onestep(BACKWARD, MICROSTEP);
}
