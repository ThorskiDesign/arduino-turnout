// TurntableMgr.h

#ifndef _TURNTABLEMGR_h
#define _TURNTABLEMGR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


// build dcc and/or touchscreen control
//#define WITH_DCC
#define WITH_TOUCHSCREEN

#include "Button.h"
#include "EventTimer.h"
#include "RGB_LED.h"
#include "AccelStepper.h"
#include "Adafruit_MotorShield.h"

#if defined(WITH_DCC)
#include "DCCdecoder.h"
#endif // WITH_DCC

#if defined(WITH_TOUCHSCREEN)
#include "Touchpad.h"
//#include "Adafruit_ILI9341.h"
//#include <Adafruit_FT6206.h>
//#include <Wire.h>
#endif // WITH_TOUCHSCREEN

#if !defined(ADAFRUIT_METRO_M0_EXPRESS)
#include "EEPROM.h"
#endif

#if defined (ADAFRUIT_METRO_M0_EXPRESS)
#include "FlashStorage.h"
#endif



class TurntableMgr
{
public:
	TurntableMgr();
	void Initialize();
	void Update();

private:

	// hardware assignments ============================================================================

	//const byte HWirqPin = 2;       set in bitstream.h
	enum : byte { hallSensorPin = 3 };
	//const byte microSDPin = 4;
	//const byte backlightPin = 5;
	//const byte speakerPin = 6;
	//const byte touchscreenIntPin = 7;
	//const byte ICRPin = 8;         set in bitstream.h
	//const byte TFT_DC_Pin = 9;
	//const byte TFT_CS_Pin = 10;
	//const byte ICSP_MOSI_Pin = 11;   // these are the defaults for the adafruilt display
	//const byte ICSP_MISO_Pin = 12;
	//const byte ICS_PSCLK_Pin = 13;
	enum : byte { LEDPin = 14 };


	// State machine ======================================================================================

	enum ttState : byte
	{
		IDLE,           // stationary, with motor powered off, listening for dcc and touchscreen
		POWERED,        // stationary, with motor powered on, listening for dcc and touchscreen, flasher on
		WARMUP,         // stationary but with pending move, dcc and touchscreen suspended, flasher on
		MOVING,         // rotating, dcc and touchscreen suspended, flasher on
		SEEK,           // seeking the hall sensor at high speed in the clockwise direction
	};

	ttState currentState = IDLE;    // test - simulate power loss while moving
	byte subState = 0;    // track sequence of events within a state, 0 = transition steps for entering that state

	// state pointer and functions
	typedef void(TurntableMgr::*StateFunctionPointer)();

	//  keep this array in the same size/order as the ttState enum
	StateFunctionPointer ttStateFunctions[5] =
	{
		&TurntableMgr::stateIdle,
		&TurntableMgr::statePowered,
		&TurntableMgr::stateWarmup,
		&TurntableMgr::stateMoving,
		&TurntableMgr::stateSeek,
	};

	StateFunctionPointer currentStateFunction = 0;

	// the state transition functions
	void stateIdle();
	void statePowered();
	void stateWarmup();
	void stateMoving();
	void stateSeek();

	// the state transition events
	enum ttEvent : byte
	{
		NONE,
		ANY,
		BUTTON_SIDING,
		MOVE_DONE,
		IDLETIMER,
		WARMUPTIMER,
		BUTTON_SEEK,
	};

	void raiseEvent(ttEvent event);

	// state transition mapping
	struct stateTransMatrixRow
	{
		ttState currState;
		ttEvent event;
		ttState nextState;
	};

	stateTransMatrixRow stateTransMatrix[7] =
	{
		// CURR STATE     // EVENT           // NEXT STATE
		{ IDLE,           BUTTON_SIDING,     WARMUP, },
		{ WARMUP,         WARMUPTIMER,       MOVING },
		{ POWERED,        BUTTON_SIDING,     MOVING, },
		{ POWERED,        IDLETIMER,         IDLE },
		{ MOVING,         MOVE_DONE,         POWERED, },
		{ SEEK,           MOVE_DONE,         IDLE },
		{ IDLE,           BUTTON_SEEK,       SEEK},
	};



	// Turntable locals  ========================================================================

	byte currentSiding = 0;
	byte lastSiding = 0;

	Button hallSensor{ hallSensorPin, true };
	EventTimer idleTimer;
	EventTimer warmupTimer;
	RgbLed flasher{ LEDPin };

	enum : uint16_t
	{
		idleTimeout = 5000,      // 5 sec, for testing
		warmupTimeout = 5000,    // 5 sec
	};


	// tft display and touchscreen setup   ========================================================

	#if defined(WITH_TOUCHSCREEN)
	Touchpad touchpad;
	#endif    // defined(WITH_TOUCHSCREEN)

	enum : byte { pageRun, pageSetup };
	byte currentPage = pageRun;


	// stepper motor and related =================================================================

	enum : uint16_t
	{
		stepperStepsPerRev = 200,
		stepperMicroSteps = 16,
		ttGearRatio = 18,
		motorShieldPort = 2,
		stepperMaxSpeed = 500,
		stepperAcceleration = 25,
		stepperLowSpeed = 50,
		stepsPerSiding = ttGearRatio * stepperStepsPerRev * stepperMicroSteps / 36,  // for 10 degree spacing between sidings
		halfCircleSteps = ttGearRatio * stepperStepsPerRev * stepperMicroSteps / 2,
	};

	Adafruit_MotorShield motorShield;
	Adafruit_StepperMotor* afStepper;
	AccelStepper accelStepper;

	void configureStepper();
	void moveToSiding();
	void reverseSiding();
	static uint16_t findBasicPosition(int32_t pos);
	static int32_t findFullStep(int32_t microsteps);


	// DCC  ======================================================================================

	#if defined(WITH_DCC)
	DCCdecoder dcc;
	#endif	// WITH_DCC

	byte dccAddress = 1;     // the dcc address of the decoder

	// define our available cv's  (allowable range 33-81 per 9.2.2)
	enum : byte
	{
		CV_AddressLSB = 1,
		CV_AddressMSB = 9,
	};

	// factory reset cv's
	enum : byte
	{
		CV_reset = 55,
		CV_softResetValue = 11,
		CV_hardResetValue = 55,
	};

	// set up cv's
	struct CV
	{
		const byte cvNum;           // cv number
		uint16_t cvValue;           // current cv value
		const bool softReset;       // should this cv get reset during a soft reset
		const uint16_t cvDefault;   // default value for the cv
	};

	enum : byte { numCVs = 2, numSidings = 10 };
	struct ConfigVars
	{
		CV CVs[numCVs];
		CV sidingSteps[numSidings];       // TODO: separate struct for these?
	};

	ConfigVars configVars =
	{
	{
			{ CV_AddressLSB, 1, false, 1 },
			{ CV_AddressMSB, 0, false, 0 },
		},
{
			{ 0, 0, false, 0 * stepsPerSiding },
			{ 0, 0, false, 1 * stepsPerSiding },
			{ 0, 0, false, 2 * stepsPerSiding },
			{ 0, 0, false, 3 * stepsPerSiding },
			{ 0, 0, false, 4 * stepsPerSiding },
			{ 0, 0, false, 5 * stepsPerSiding },
			{ 0, 0, false, 6 * stepsPerSiding },
			{ 0, 0, false, 7 * stepsPerSiding },
			{ 0, 0, false, 8 * stepsPerSiding },
			{ 0, 0, false, 9 * stepsPerSiding },
		}
	};

	struct StateVars
	{
		ttState state;
		byte currentSiding;
	};

	StateVars stateVars = { IDLE, 0 };

	uint16_t getCV(byte cv);
	void SaveConfig();
	void SaveState();
	void LoadConfig();
	void LoadConfig(bool reset);
	void LoadState();


	// event handlers  ===========================================================================

	// pointer to allow us to access member objects from callbacks
	static TurntableMgr* currentInstance;

	// event handler functions
	void GraphicButtonHandler(byte buttonID, bool state);
	static void StepperClockwiseStep();
	static void StepperCounterclockwiseStep();

	// wrappers for callbacks
	//static void WrapperHallSensorHandler(bool ButtonState);
	static void WrapperIdleTimerHandler();
	static void WrapperWarmupTimerHandler();
	static void WrapperGraphicButtonHandler(byte buttonID, bool state);
};

#endif
