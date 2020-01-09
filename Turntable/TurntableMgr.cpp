
#include "TurntableMgr.h"

// static pointer for callbacks
TurntableMgr* TurntableMgr::currentInstance = 0;

#if defined(ADAFRUIT_METRO_M0_EXPRESS)
FlashStorage(flashConfig, TurntableMgr::ConfigVars);
FlashStorage(flashState, TurntableMgr::StateVars);
#endif

TurntableMgr::TurntableMgr()
{
	// pointer for callback functions
	currentInstance = this;
}

void TurntableMgr::Initialize()
{
	// configure factory default CVs
	byte index = 0;
	index = configCVs.initCV(index, CV_AddressLSB, 50);
	index = configCVs.initCV(index, CV_AddressMSB, 0);
	index = configCVs.initCV(index, CV_WarmupTimeout, 5, 0, 30);
	configCVs.initCV16(index, CV_IdleTimeout, 300, 0, 600);

	// configure default siding positions (set up to match current layout)
	index = 0;
	index = sidingCVs.initCV16(index, 1, 0, 0, 180U * stepsPerDegree);
	index = sidingCVs.initCV16(index, 2, 13968, 0, 180U * stepsPerDegree);
	index = sidingCVs.initCV16(index, 3, 12384, 0, 180U * stepsPerDegree);
	index = sidingCVs.initCV16(index, 4, 10816, 0, 180U * stepsPerDegree);
	index = sidingCVs.initCV16(index, 5, 9216, 0, 180U * stepsPerDegree);
	index = sidingCVs.initCV16(index, 6, 7600, 0, 180U * stepsPerDegree);
	index = sidingCVs.initCV16(index, 7, 6000, 0, 180U * stepsPerDegree);
	index = sidingCVs.initCV16(index, 8, 15600, 0, 180U * stepsPerDegree);
	sidingCVs.initCV16(index, 9, 10 * stepsPerDegree, 0, 180U * stepsPerDegree);

	// load config and last saved state
	LoadState();
	LoadConfig();

	// setup the stepper
	configureStepper();

	// configure callbacks
	idleTimer.SetTimerHandler(WrapperIdleTimerHandler);
	warmupTimer.SetTimerHandler(WrapperWarmupTimerHandler);
	errorTimer.SetTimerHandler(WrapperErrorTimerHandler);

	#if defined(WITH_DCC)
	// Configure and initialize the DCC packet processor (accessory decoder in output address mode)
	const byte cv29 = DCCdecoder::CV29_ACCESSORY_DECODER | DCCdecoder::CV29_OUTPUT_ADDRESS_MODE;
	dcc.SetupDecoder(0, 0, cv29, false);
	byte addr = (configCVs.getCV(CV_AddressMSB) << 8) + configCVs.getCV(CV_AddressLSB);
	dcc.SetAddress(addr);

	// configure dcc event handlers
	dcc.SetBasicAccessoryDecoderPacketHandler(WrapperDCCAccPacket);
	dcc.SetExtendedAccessoryDecoderPacketHandler(WrapperDCCExtPacket);
	dcc.SetBasicAccessoryPomPacketHandler(WrapperDCCAccPomPacket);
	dcc.SetBitstreamMaxErrorHandler(WrapperMaxBitErrors);
	dcc.SetPacketMaxErrorHandler(WrapperMaxPacketErrors);
	dcc.SetDecodingErrorHandler(WrapperDCCDecodingError);
	#endif

	#if defined(WITH_TOUCHSCREEN)
	touchpad.Init();
	touchpad.SetGraphicButtonHandler(WrapperGraphicButtonHandler);
	#endif // defined(WITH_TOUCHSCREEN)

	// start the state machine
	currentState = IDLE;
	currentStateFunction = &TurntableMgr::stateIdle;
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

	errorTimer.Update();
}


void TurntableMgr::stateMoving()
{
	if (subState == 0)     // transition to state
	{

		#if defined(WITH_DCC)
		dcc.SuspendBitstream();
		#endif   // WITH_DCC

		flasher.SetLED(RgbLed::RED, RgbLed::FLASH, 500, 500);

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

	if (accelStepper.distanceToGo() == 0)    // move is done
	{
		SaveState();    // TODO: add checks here to minimize flash/eeprom writes
						// TODO: need to save current siding, but this is the wrong place to save state
		raiseEvent(MOVE_DONE);
	}
}

void TurntableMgr::stateSeek()
{
	hallSensor.Update();

	switch (subState)
	{
	case 0:                 // transition to seek state

		#if defined(WITH_DCC)
		dcc.SuspendBitstream();
		#endif

		// start moving in a complete clockwise circle at normal speed
		accelStepper.setMaxSpeed(stepperMaxSpeed / 2);
		accelStepper.setAcceleration(stepperAcceleration);
		accelStepper.move(360 * stepsPerDegree);

		attachInterrupt(digitalPinToInterrupt(hallSensorPin), HallIrq, RISING);

		subState = 1;
		break;
	case 1:    // CW rotation, waiting for hall sensor to go low, indicating magnet has entered its detection area
		if (!hallSensor.SwitchState())
		{
			flasher.SetLED(RgbLed::RED, RgbLed::ON);
			subState = 2;   // found it, go to next state
		}
		break;
	case 2:    // CW rotation, waiting for IRQ callback to set subState = 3
		//if (hallSensor.SwitchState()) 
		//{
		//	homePosition = findFullStep(accelStepper.currentPosition());
		//	subState = 3;
		//}
		break;
	case 3:    // now we've swung past in the CW direction and our home position is set (in the IRQ)
		flasher.SetLED(RgbLed::RED, RgbLed::OFF);
		detachInterrupt(digitalPinToInterrupt(hallSensorPin));
		accelStepper.stop();     // set the stepper to stop with its current speed/accel settings
		subState = 4;
		break;
	case 4:    // waiting for CW motion to stop
		if (accelStepper.distanceToGo() == 0)
		{
			// now that we have stopped, set our home position
			int16_t delta = accelStepper.currentPosition() - homePosition;
			accelStepper.setCurrentPosition(delta);
		}
		break;
	}

	// When we stop after substate 5, or if we miss a hall sensor event and go full circle, end the move
	if (accelStepper.distanceToGo() == 0)
		raiseEvent(MOVE_DONE);      // motion has stopped, raise event to exit seek state

	// do the update functions for this state
	//hallSensor.Update();
	accelStepper.run();
	//hallSensor.Update();
	//flasher.Update();
}


void TurntableMgr::stateCalibrate()
{
	if (subState == 0)     // transition to state
	{

		flasher.SetLED(RgbLed::RED, RgbLed::FLASH, 500, 500);

		#if defined(WITH_DCC)
		dcc.SuspendBitstream();
		#endif   // WITH_DCC

		accelStepper.setMaxSpeed(stepperMaxSpeed);
		accelStepper.setAcceleration(10 * stepperAcceleration);

		subState = 1;
	}

	// do the update functions for this state
	uint32_t currentMillis = millis();
	accelStepper.run();
	flasher.Update(currentMillis);

	#if defined(WITH_TOUCHSCREEN)
	touchpad.Update();
	#endif // defined(WITH_TOUCHSCREEN)

	// if the current move is done, then set up the next cal move, or exit
	if (accelStepper.distanceToGo() == 0)
	{
		switch (calCmd.type)
		{
		case CalCmd::none:
			raiseEvent(MOVE_DONE);
			break;
		case CalCmd::continuous:
			accelStepper.move(calCmd.calSteps);
			break;
		case CalCmd::incremental:
			accelStepper.move(calCmd.calSteps);
			calCmd.type = CalCmd::none;
			break;
		default:
			break;
		}
	}
}


void TurntableMgr::statePowered()
{
	if (subState == 0)     // transition to state
	{

		flasher.SetLED(RgbLed::ON);         // turn off the warning light

		#if defined(WITH_DCC)
		dcc.ResumeBitstream();               // resume listening for dcc commands
		#endif // deinfed(WITH_DCC)

		// start the timer for the transition to idle state
		idleTimer.StartTimer(1000UL * configCVs.getCV(CV_IdleTimeout));
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
	errorTimer.Update(currentMillis);
}

void TurntableMgr::stateWarmup()
{
	if (subState == 0)     // transition to state
	{

		#if defined(WITH_DCC)
		dcc.SuspendBitstream();
		#endif   // WITH_DCC

		flasher.SetLED(RgbLed::RED, RgbLed::FLASH, 500, 500);

		// start the timer for the transition to moving state
		warmupTimer.StartTimer(1000UL * configCVs.getCV(CV_WarmupTimeout));
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

	// set stepper position to correspond to current siding
	accelStepper.setCurrentPosition(sidingCVs.getCV(currentSiding));
}


void TurntableMgr::moveToSiding()
{
	// get stepper position for desired siding
	const int32_t targetPos = moveCmd.targetPos;  // put into int32 for later calcs

	// get current stepper position in positive half circle equivalent
	const uint16_t currentPos = findBasicPosition(accelStepper.currentPosition());

	// steps needed for move (go in opposite direction if ouside +/-90 deg)
	int32_t moveSteps = targetPos - currentPos;
	if (moveSteps > 90L * stepsPerDegree) moveSteps -= 180L * stepsPerDegree;
	if (moveSteps < -90L * stepsPerDegree) moveSteps += 180L * stepsPerDegree;

	// update steps for reverse move if needed
	if (moveCmd.type == MoveCmd::reverse)
	{
		if (moveSteps > 0) 
			moveSteps -= 180L * stepsPerDegree;
		else
			moveSteps += 180L * stepsPerDegree;
	}

	moveCmd.type = MoveCmd::normal;    // reset for normal move if it was reversed

	// do the move
	if (moveSteps != 0) accelStepper.move(moveSteps);

	previousSiding = currentSiding;   // for debug purposes
}


// convert stepper motor position to a positive position in the first half circle of rotation
uint16_t TurntableMgr::findBasicPosition(int32_t pos)
{
	int32_t p = pos;
	const uint16_t halfCircleSteps = 180 * stepsPerDegree;

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
		return microsteps - remainder + stepperMicroSteps;
	}
}



// event handlers and static wrappers

void TurntableMgr::WrapperIdleTimerHandler() { currentInstance->raiseEvent(IDLETIMER); }
void TurntableMgr::WrapperWarmupTimerHandler() { currentInstance->raiseEvent(WARMUPTIMER); }
void TurntableMgr::WrapperErrorTimerHandler() {	currentInstance->flasher.SetLED(RgbLed::OFF); }

void TurntableMgr::WrapperGraphicButtonHandler(byte buttonID, bool state)
{
	currentInstance->CommandHandler(buttonID, state);
}

void TurntableMgr::WrapperDCCAccPacket(int boardAddress, int outputAddress, byte activate, byte data)
{
	if (data == 1) currentInstance->CommandHandler(1, true);    // accessory command for siding 1
}

void TurntableMgr::WrapperDCCExtPacket(int boardAddress, int outputAddress, byte data)
{
	currentInstance->CommandHandler(data, true);
}

void TurntableMgr::WrapperDCCAccPomPacket(int boardAddress, int outputAddress, byte instructionType, int cv, byte data)
{
	currentInstance->DCCPomHandler(outputAddress, instructionType, cv, data);
}

void TurntableMgr::WrapperMaxBitErrors(byte errorCode)
{
}

void TurntableMgr::WrapperMaxPacketErrors(byte errorCode)
{
}

void TurntableMgr::WrapperDCCDecodingError(byte errorCode)
{
}


void TurntableMgr::SaveState()
{
	stateVars.currentState = currentState;
	stateVars.currentSiding = currentSiding;
	stateVars.isValid = true;

	#if defined(ADAFRUIT_METRO_M0_EXPRESS)
	flashState.write(stateVars);
	#else
	EEPROM.put(0, stateVars);
	#endif
}

void TurntableMgr::LoadState()
{
	#if defined(ADAFRUIT_METRO_M0_EXPRESS)
	StateVars tempStateVars = flashState.read();
	const bool firstBoot = !tempStateVars.isValid;  // this is false on read of unitialized flash
	#else
	const bool firstBoot = (EEPROM.read(0) == 255);
	#endif

	if (!firstBoot)   // if not first boot, then load stored state, otherwise use default statevars initialization
	{

		#if defined(ADAFRUIT_METRO_M0_EXPRESS)
		stateVars = tempStateVars;
		#else
		EEPROM.get(0, stateVars);    // get the last state from eeprom
		#endif // defined(ADAFRUIT_METRO_M0_EXPRESS)

		currentState = stateVars.currentState;
		currentSiding = stateVars.currentSiding;
	}
}

void TurntableMgr::SaveConfig()
{
	// copy working CVs to our storage object
	for (byte i = 0; i < numCVindexes; i++)
		configVars.CVs[i] = configCVs.cv[i].cvValue;
	for (byte i = 0; i < numSidingIndexes; i++)
		configVars.sidingSteps[i] = sidingCVs.cv[i].cvValue;

	// store the config
	#if defined(ADAFRUIT_METRO_M0_EXPRESS)
	flashConfig.write(configVars);
	#else
	EEPROM.put(sizeof(stateVars), configVars);   // store the config vars struct starting after the state vars in EEPROM
	#endif
}

void TurntableMgr::LoadConfig()
{
	LoadConfig(false);
}

void TurntableMgr::LoadConfig(bool reset)
{
	//// determine reset or normal boot
	//#if defined(ADAFRUIT_METRO_M0_EXPRESS)
	//const bool firstBoot = !stateVars.isValid;   //  isValid should be false on first boot, but
	//											 //  must load state from flash before loading config
	//#else
	//const bool firstBoot = (EEPROM.read(0) == 255);    // default value for unwritten eeprom
	//#endif

	// if this is the first boot on fresh eeprom/flash, or reset requested
	if (!stateVars.isValid || reset)   
	{

		//load default values for config vars and sidings
		configCVs.resetCVs();
		sidingCVs.resetCVs();

		SaveState();
		SaveConfig();
	}
	else   // normal boot, load config from eeprom/flash
	{

		#if defined(ADAFRUIT_METRO_M0_EXPRESS)
		configVars = flashConfig.read();
		#else
		EEPROM.get(sizeof(stateVars), configVars);   // load the config vars struct starting after the state vars in EEPROM
		#endif

		// copy stored config to working CVs
		for (byte i = 0; i < numCVindexes; i++)
			configCVs.cv[i].cvValue = configVars.CVs[i];
		for (byte i = 0; i < numSidingIndexes; i++)
			sidingCVs.cv[i].cvValue = configVars.sidingSteps[i];
	}
}

void TurntableMgr::SetSidingCal()
{
	// update the siding position in our cv struct
	long pos = accelStepper.currentPosition();
	uint16_t basicPos = findBasicPosition(pos);
	int32_t fullstepPos = findFullStep(basicPos);
	sidingCVs.setCV(currentSiding, fullstepPos);

	// store the turntable state and cv struct to nvram
	SaveConfig();
}


void TurntableMgr::HallIrq()
{
	currentInstance->homePosition = findFullStep(currentInstance->accelStepper.currentPosition());
	currentInstance->subState = 3;   //  for seek state 3 to begin slowing down
}


void TurntableMgr::CommandHandler(byte buttonID, bool state)
{

	if (state)        // button was pressed or dcc command received
	{
		switch (buttonID)
		{
		case Touchpad::numpad1:
		case Touchpad::numpad2:
		case Touchpad::numpad3:
		case Touchpad::numpad4:
		case Touchpad::numpad5:
		case Touchpad::numpad6:
		case Touchpad::numpad7:
		case Touchpad::numpad8:
		case Touchpad::numpad9:
			currentSiding = buttonID;
			moveCmd.targetPos = sidingCVs.getCV(currentSiding);
			raiseEvent(BUTTON_SIDING);
			break;
		case Touchpad::modeRun:
			break;
		case Touchpad::modeSetup:
			break;
		case Touchpad::runReverse:
			moveCmd.type = MoveCmd::reverse;  // this gets reset after the reverse move is complete
			break;

		// calibration commands
		case Touchpad::setupStepCW:
			calCmd.type = CalCmd::continuous;
			calCmd.calSteps = stepperMicroSteps;
			raiseEvent(BUTTON_CAL);
			break;
		case Touchpad::setupStepCCW:
			calCmd.type = CalCmd::continuous;
			calCmd.calSteps = -1L * stepperMicroSteps;
			raiseEvent(BUTTON_CAL);
			break;
		case Touchpad::setup10CW:
			calCmd.type = CalCmd::incremental;
			calCmd.calSteps = 10 * stepsPerDegree;
			raiseEvent(BUTTON_CAL);
			break;
		case Touchpad::setup10CCW:
			calCmd.type = CalCmd::incremental;
			calCmd.calSteps = -10L * stepsPerDegree;
			raiseEvent(BUTTON_CAL);
			break;
		case Touchpad::setup30CW:
			calCmd.type = CalCmd::incremental;
			calCmd.calSteps = 30 * stepsPerDegree;
			raiseEvent(BUTTON_CAL);
			break;
		case Touchpad::setup30CCW:
			calCmd.type = CalCmd::incremental;
			calCmd.calSteps = -30L * stepsPerDegree;
			raiseEvent(BUTTON_CAL);
			break;
		case Touchpad::setup90CW:
			calCmd.type = CalCmd::incremental;
			calCmd.calSteps = 90 * stepsPerDegree;
			raiseEvent(BUTTON_CAL);
			break;
		case Touchpad::setup90CCW:
			calCmd.type = CalCmd::incremental;
			calCmd.calSteps = -90L * stepsPerDegree;
			raiseEvent(BUTTON_CAL);
			break;


		case Touchpad::setupSet:
			SetSidingCal();
			break;
		case Touchpad::setupHome:
			raiseEvent(BUTTON_SEEK);
			break;
		case Touchpad::estop:
			raiseEvent(BUTTON_ESTOP);
			break;
		default:
			break;
		}
	}
	else             // button was released (n/a for dcc commands)
	{
		switch (buttonID)
		{
		case Touchpad::setupStepCW:
		case Touchpad::setupStepCCW:
			calCmd.type = CalCmd::none;
		default:
			break;
		}
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

void TurntableMgr::DCCPomHandler(unsigned int Addr, byte instType, unsigned int CV, byte Value)
{
	// assume we are filtering repeated packets in the packet builder, so we don't check for that here
	// assume DCCdecoder is set to return only packets for this decoder's address.

	// TODO: check for and perform reset

	// set the cv
	if (configCVs.setCV(CV,Value))
	{
		errorTimer.StartTimer(250);
		flasher.SetLED(RgbLed::RED, RgbLed::FLASH, 50, 50);
	}
	else
	{
		errorTimer.StartTimer(1000);
		flasher.SetLED(RgbLed::RED, RgbLed::FLASH, 250, 250);
	}

	#if defined(WITH_DCC)
	// update dcc address
	byte addr = (configCVs.getCV(CV_AddressMSB) << 8) + configCVs.getCV(CV_AddressLSB);
	dcc.SetAddress(addr);
	#endif

	// save the new config
	SaveConfig();
}
