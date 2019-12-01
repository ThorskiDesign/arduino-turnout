
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
		#endif // deinfed(WITH_DCC)

		subState = 1;
	}

	// do the update functions for this state

	#if defined(WITH_DCC)
	dcc.ProcessTimeStamps();
	#endif // deinfed(WITH_DCC)

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

		flasher.SetLED(RgbLed::RED, RgbLed::FLASH);

		// move to the specified siding at normal speed
		accelStepper.setMaxSpeed(stepperMaxSpeed);
		accelStepper.setAcceleration(stepperAcceleration);
		moveToSiding(currentSiding);

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
			accelStepper.stop();                   // set the stepper to stop with its current speed/accel settings

			subState = 6;
		}
		break;
	case 6:   // CCW rotation, waiting for motion to stop
		if (accelStepper.distanceToGo() == 0)
			raiseEvent(MOVE_DONE);      // motion has stopped, raise event to exit seek state
		break;
	default:
		break;
	}

	// TODO: add timeout/error check in case we somehow miss one of the substate steps

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

	//String eventLabel[8] = {
	//	"NONE",
	//	"ANY",
	//	"BUTTON_SIDING",
	//	"MOVE_DONE",
	//	"HALLSENSOR_HIGH",
	//	"HALLSENSOR_LOW",
	//	"IDLETIMER",
	//	"WARMUPTIMER",
	//};

	//String stateLabel[6] = {
	//	"IDLE",           // stationary, with motor powered off, listening for dcc and touchscreen
	//	"POWERED",        // stationary, with motor powered on, listening for dcc and touchscreen, flasher on
	//	"WARMUP",         // stationary but with pending move, dcc and touchscreen suspended, flasher on
	//	"MOVING",         // rotating, dcc and touchscreen suspended, flasher on
	//	"SEEK",       // seeking the hall sensor at high speed in the clockwise direction
	//	"SEEKSLOW",       // hall sensor has gone active, tt moving in ccw direction until it deactivates again
	//};

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
	TWBR = ((F_CPU / 400000l) - 16) / 2;     // change I2C clock to 400kHz

	// accelstepper setup
	accelStepper.setMaxSpeed(stepperMaxSpeed);
	accelStepper.setAcceleration(stepperAcceleration);
}


void TurntableMgr::moveToSiding(const byte siding)
{
	// get stepper position for desired siding
	int32_t targetPos = FactoryDefaultSettings[siding - 1].Value;

	// get current stepper position in positive half circle equivalent
	uint16_t currentPos = findBasicPosition(accelStepper.currentPosition());

	// now figure out the shortest rotation
	int32_t deltaCW = targetPos - currentPos;
	int32_t deltaCCW = (targetPos - halfCircleSteps) - currentPos;

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
