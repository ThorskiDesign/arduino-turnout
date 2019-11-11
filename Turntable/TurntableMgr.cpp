
#include "TurntableMgr.h"

// static pointer for callbacks
TurntableMgr* TurntableMgr::currentInstance = 0;

TurntableMgr::TurntableMgr()
{
	// pointer for callback functions
	currentInstance = this;

	// configure event handlers
}

void TurntableMgr::Initialize()
{
	Serial.println("In initialize");

	// setup the stepper
	ConfigureStepper();

	// setup display
	pinMode(backlightPin, OUTPUT);
	digitalWrite(backlightPin, HIGH);

	tft.begin();
	tft.setRotation(TFT_rotation);
	tft.fillScreen(white);

#ifdef _DEBUG
	// read diagnostics (optional but can help debug problems)
	uint8_t x = tft.readcommand8(ILI9341_RDMODE);
	Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDMADCTL);
	Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDPIXFMT);
	Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDIMGFMT);
	Serial.print("Image Format: 0x"); Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDSELFDIAG);
	Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);
#endif  // DEBUG

	// setup touchscreen
	ctp.begin();

	// configure buttons
	ConfigureButtons();

	// configure callbacks
	hallSensor.SetButtonPressHandler(WrapperHallSensorHandler);
	idleTimer.SetTimerHandler(WrapperIdleTimerHandler);
	warmupTimer.SetTimerHandler(WrapperWarmupTimerHandler);

	// start the state machine
	if (currentState == MOVING || currentState == SEEKFAST || currentState == SEEKSLOW)
		// we shutdown without reaching our destination, so stepper position is unknown
	{
		currentState = SEEKFAST;
		stateSeekFast();
	}
	else
		// normal startup
	{
		currentState = IDLE;
		stateIdle();
	}
}


void TurntableMgr::Update()
{
	// TODO: figure out a tidy way to disable things that don't need to be updated
	//       depending on the state we're in

	//dcc.ProcessTimeStamps();
	PollTouchscreen();
	accelStepper.run();
	if (currentState == MOVING && accelStepper.distanceToGo() == 0) raiseEvent(MOVE_DONE);

	// update sensors, etc. as needed
	const unsigned long currentMillis = millis();
	hallSensor.Update(currentMillis);     // TODO: disable this if not in seek mode (maybe just use pin/irq directly rather than button class??)
	idleTimer.Update(currentMillis);      // TODO: skip this if not in powered state?
	warmupTimer.Update(currentMillis);
	flasher.Update(currentMillis);        // TODO: skip if not powered or moving?
}


void TurntableMgr::PollTouchscreen()
{
	const bool touched = ctp.touched();
	if (touched && lastbtn == -1)
	{
		const TS_Point p = ctp.getPoint();

		// flip touchscreen coords around to match the tft coords
#if TFT_rotation == 0
		const unsigned int x = map(p.x, 0, 240, 240, 0);
		const unsigned int y = map(p.y, 0, 320, 320, 0);
#endif
#if TFT_rotation == 1
		const unsigned int x = map(p.y, 0, 320, 320, 0);
		const unsigned int y = p.x;
#endif

		// Serial.print("ts coords:  "); Serial.print(x); Serial.print("  "); Serial.println(y);

		// loop through buttons to check
		byte i = 0;
		while (lastbtn == -1 && i < numButtons)
		{
			if ((*button)[i].Press(x, y))
				lastbtn = i;
			i++;
		}
	}

	if (!touched && lastbtn != -1)
	{
		(*button)[lastbtn].Release();
		lastbtn = -1;
	}
}

void TurntableMgr::ConfigureStepper()
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


//  state transition functions  ===================================================================

void TurntableMgr::stateIdle()
{
	// power off the stepper
	afStepper->release();

	// turn off the warning light
	flasher.SetLED(RgbLed::OFF);

	//	dcc.ResumeBitstream();
}

void TurntableMgr::stateMoving()
{
	//	dcc.SuspendBitstream();

	flasher.SetLED(RgbLed::RED, RgbLed::FLASH);

	// move to the specified siding at normal speed
	accelStepper.setMaxSpeed(stepperMaxSpeed);
	accelStepper.setAcceleration(stepperAcceleration);
	moveToSiding(currentSiding);
}

void TurntableMgr::stateSeekFast()
{
	//	dcc.SuspendBitstream();

	// start moving in a complete clockwise circle at normal speed
	accelStepper.setMaxSpeed(stepperMaxSpeed);
	accelStepper.setAcceleration(stepperAcceleration);
	accelStepper.move(2 * halfCircleSteps);
}

void TurntableMgr::stateSeekSlow()
{
	// TODO: slow or stop first here??

	// start moving in a counter clockwise halfcircle at low speed
	accelStepper.setMaxSpeed(stepperLowSpeed);
	accelStepper.setAcceleration(stepperAcceleration);
	accelStepper.move(-halfCircleSteps);
}

void TurntableMgr::statePowered()
{
	// TODO: stop stepper if running?  e.g., for in between seek states?
	// TODO: go to full step before stopping
	accelStepper.stop();

	// start the timer for the transition to idle state
	idleTimer.StartTimer(idleTimeout);

	//	dcc.ResumeBitstream();
}

void TurntableMgr::stateWarmup()
{
	warmupTimer.StartTimer(warmupTimeout);
	flasher.SetLED(RgbLed::RED, RgbLed::FLASH);
}


void TurntableMgr::raiseEvent(const ttEvent event)
{
	byte i = 0;
	const byte n = sizeof(stateTransMatrix) / sizeof(stateTransMatrixRow);

	// loop through state transition table until we find matching state and event
	while ((i < n) && !(currentState == stateTransMatrix[i].currState && event == stateTransMatrix[i].event)) i++;

	// if there was no match just return
	if (i == n)
	{
		Serial.println("event ignored, no match");
		return;
	}

	Serial.print("executing event: "); Serial.println(stateTransMatrix[i].event);
	Serial.print("current state  : "); Serial.println(stateTransMatrix[i].nextState);

	// otherwise transition to next state specified in the matching row
	currentState = stateTransMatrix[i].nextState;

	// call the state transition function
	(*this.*ttStateFunctions[currentState])();
}



//  local functions   ===========================================================================

void TurntableMgr::ConfigureButtons()
{
	button[0] = new GraphicButton(&tft, GraphicButton::TOGGLE, GraphicButton::ROUNDRECT, 10, 60, 60, 40, "1", 1);
	button[1] = new GraphicButton(&tft, GraphicButton::TOGGLE, GraphicButton::ROUNDRECT, 80, 60, 60, 40, "2", 2);
	button[2] = new GraphicButton(&tft, GraphicButton::TOGGLE, GraphicButton::ROUNDRECT, 150, 60, 60, 40, "3", 3);

	for (byte i = 0; i < numButtons; i++)
	{
		(*button)[i].SetButtonHandler(this, WrapperButtonHandler);
		(*button)[i].SetActive(true);
	}

	Serial.println("buttons configured");
}


void TurntableMgr::moveToSiding(byte siding)
{
	Serial.print("move to siding:  "); Serial.println(siding);

	int32_t targetPos = FactoryDefaultSettings[siding - 1].Value;
	Serial.print("targetPos:  "); Serial.println(targetPos);

	// get current stepper position in positive half circle equivalent
	uint16_t currentPos = BasicPosition(accelStepper.currentPosition());
	Serial.print("basicPos:  "); Serial.println(currentPos);

	// now figure out the shortest rotation
	int32_t deltaCW = targetPos - currentPos;
	int32_t deltaCCW = (targetPos - halfCircleSteps) - currentPos;

	if (deltaCW == 0 || deltaCCW == 0) return;

	Serial.print("delta CW:  "); Serial.println(deltaCW);
	Serial.print("delta CCW:  "); Serial.println(deltaCCW);

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
uint16_t TurntableMgr::BasicPosition(int32_t pos)
{
	int32_t p = pos;

	if (p < 0)     // convert negative motor position
	{
		byte n = -p / halfCircleSteps;
		p += (1 + n) * halfCircleSteps;
	}

	return p % halfCircleSteps;
}


void TurntableMgr::WrapperIdleTimerHandler() { currentInstance->raiseEvent(IDLETIMER); }
void TurntableMgr::WrapperWarmupTimerHandler() { currentInstance->raiseEvent(WARMUPTIMER); }

void TurntableMgr::WrapperHallSensorHandler(bool ButtonState)
{
	if (ButtonState)
	{
		currentInstance->raiseEvent(HALLSENSOR_HIGH);
	}
	else
	{
		currentInstance->raiseEvent(HALLSENSOR_LOW);
	}
}

void TurntableMgr::WrapperButtonHandler(void* p, bool state, unsigned int data)
{
	static_cast<TurntableMgr*>(p)->ButtonEventHandler(state, data);
}

void TurntableMgr::ButtonEventHandler(const bool state, const unsigned int data)
{
	if (state)
	{
		currentSiding = data;
		raiseEvent(BUTTON_SIDING);
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
