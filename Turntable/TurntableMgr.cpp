
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
}

void TurntableMgr::Update()
{
	if (dccIsActive[turntableMode]) dcc.ProcessTimeStamps();
	//	if (tsIsActive[turntableMode]) 
	PollTouchscreen();
	if (stepperIsRunning[turntableMode]) accelStepper.run();

	// need more here for end of move, etc.
	if (accelStepper.distanceToGo() == 0)
	{
		turntableMode == IDLE;
	}

	//// do the updates to maintain flashing led and slow servo motion
	//const unsigned long currentMillis = millis();
	//led.Update(currentMillis);

	//// timer updates
	//errorTimer.Update(currentMillis);
	//resetTimer.Update(currentMillis);
	//servoTimer.Update(currentMillis);

	//// update sensors
	//button.Update(currentMillis);
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
			if (button[i].Press(x, y))
				lastbtn = i;
			i++;
		}
	}

	if (!touched && lastbtn != -1)
	{
		button[lastbtn].Release();
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

void TurntableMgr::ConfigureButtons()
{
	for (byte i = 0; i < numButtons; i++)
	{
		button[i].SetButtonHandler(this, WrapperButtonHandler);
		button[i].SetActive(true);
	}
}


void TurntableMgr::moveToSiding(byte siding)
{
	Serial.print("siding:  "); Serial.println(siding);

	int32_t targetPos = FactoryDefaultSettings[siding - 1].Value;
	Serial.print("targetPos:  "); Serial.println(targetPos);

	// get current stepper position in positive half circle equivalent
	uint16_t currentPos = BasicPosition(accelStepper.currentPosition());
	Serial.print("basicPos:  "); Serial.println(currentPos);

	// now figure out the shortest rotation
	int32_t deltaCW = targetPos - currentPos;
	int32_t deltaCCW = (targetPos - halfCircleSteps) - currentPos;

	Serial.print("delta CW:  "); Serial.println(deltaCW);
	Serial.print("delta CCW:  "); Serial.println(deltaCCW);

	if (abs(deltaCW) < abs(deltaCCW))
	{
		turntableMode = RUNNING_CW;
		accelStepper.move(deltaCW);
	}
	else
	{
		turntableMode = RUNNING_CCW;
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


void TurntableMgr::WrapperButtonHandler(void* p, bool state, unsigned int data)
{
	static_cast<TurntableMgr*>(p)->ButtonEventHandler(state, data);
}

void TurntableMgr::ButtonEventHandler(bool state, unsigned int data)
{
	if (state)
		moveToSiding(data);
}





void TurntableMgr::StepperClockwiseStep()
{
	currentInstance->afStepper->onestep(FORWARD, MICROSTEP);
}

void TurntableMgr::StepperCounterclockwiseStep()
{
	currentInstance->afStepper->onestep(BACKWARD, MICROSTEP);
}
