// TurntableMgr.h

#ifndef _TURNTABLEMGR_h
#define _TURNTABLEMGR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "DCCdecoder.h"
#include "GraphicButton.h"
#include "Adafruit_ILI9341.h"
#include <Adafruit_FT6206.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>



class TurntableMgr
{
public:
	TurntableMgr();
	void Initialize();
	void Update();

private:
	// hardware assignments
	//const byte HWirqPin = 2;       set in bitstream.h
	const byte hallSensorPin = 3;
	const byte microSDPin = 4;
	const byte backlightPin = 5;
	const byte speakerPin = 6;
	const byte touchscreenIntPin = 7;
	//const byte ICRPin = 8;         set in bitstream.h
	const byte TFT_DC_Pin = 9;
	const byte TFT_CS_Pin = 10;
	const byte ICSP_MOSI_Pin = 11;   // these are the defaults for the adafruilt display
	const byte ICSP_MISO_Pin = 12;
	const byte ICS_PSCLK_Pin = 13;
	const byte LEDPin = 14;

	// operational modes
	enum ttMode
	{
		IDLE,              // tt stationary, with motor powered off, listening for dcc and touchscreen
		POWERED,           // tt stationary, with motor powered on, listening for dcc and touchscreen
		HOMING,            // seeking hall sensor zero reference. dcc and touchscreen suspended.
		CALIBRATE,         // setting siding step counts. dcc suspended. listening for touchscreen.
		RUNNING_CW,        // tt rotating clockwise. dcc and touchscreen suspended.
		RUNNING_CCW        // tt rotating counterclockwise. dcc and touchscreen suspended.
	};

	const bool dccIsActive[6] = { true, true, false, false, false, false };
	const bool tsIsActive[6] = { true, true, false, true, false, false };
	const bool stepperIsRunning[6] = { false, false, true, true, true, true };

	ttMode turntableMode = IDLE;
	byte currentSiding = 0;
	byte lastSiding = 0;

	// pointer to allow us to access member objects from callbacks
	static TurntableMgr* currentInstance;


	// tft display and touchscreen setup   =================================================

#define TFT_rotation 0
	const uint16_t white = 0xFFFF;

	Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS_Pin, TFT_DC_Pin);      // display
	Adafruit_FT6206 ctp = Adafruit_FT6206();                              // touchscreen

	const byte numButtons = 3;
	GraphicButton button[3]{
		{ &tft, GraphicButton::TOGGLE, GraphicButton::ROUNDRECT, 10, 10, 80, 40, "1", 1 },
		{ &tft, GraphicButton::TOGGLE, GraphicButton::ROUNDRECT, 10, 60, 80, 40, "2", 2 },
		{ &tft, GraphicButton::TOGGLE, GraphicButton::ROUNDRECT, 10, 110, 80, 40, "3", 3 },
	};

	int lastbtn = -1;

	void ConfigureButtons();
	void PollTouchscreen();


	// stepper motor and related =================================================================
	const byte stepperStepsPerRev = 200;
	const byte stepperMicroSteps = 16;
	const uint16_t stepperMaxSpeed = 500;
	const uint16_t stepperAcceleration = 25;
	const byte ttGearRatio = 18;
	const byte motorShieldPort = 2;

	const uint16_t stepsPerSiding = (uint16_t)ttGearRatio * stepperStepsPerRev * stepperMicroSteps / 36;  // for 10 degree spacing between sidings
	const uint16_t halfCircleSteps = (uint16_t)ttGearRatio * stepperStepsPerRev * stepperMicroSteps / 2;

	Adafruit_MotorShield motorShield;
	Adafruit_StepperMotor* afStepper;
	AccelStepper accelStepper;

	void ConfigureStepper();
	void moveToSiding(byte siding);
	uint16_t BasicPosition(int32_t pos);


	// DCC  ======================================================================================

	DCCdecoder dcc;
	byte dccAddress = 1;     // the dcc address of the decoder

	// define our available cv's  (allowable range 33-81 per 9.2.2)
	// these are programmable via DCC like normal
	const byte CV_AddressLSB = 1;
	const byte CV_AddressMSB = 9;

	// factory default settings
	const byte CV_reset = 55;
	const byte CV_softResetValue = 11;
	const byte CV_hardResetValue = 55;

	// set up default cv's
	struct CVPair
	{
		const uint16_t  CV;
		const uint8_t   Value;
		const bool      SoftReset;
	};

	const CVPair FactoryDefaultCVs[2] =
	{
		{ CV_AddressLSB, 1, false },
		{ CV_AddressMSB, 0, false },
	};

	// set up 16 bit vars for eeprom storage
	// these are not accessible or programmable via DCC, because they are 16 bit
	const byte varStartAddress = 101;
	const byte addrSiding1Steps = varStartAddress;
	const byte addrSiding2Steps = varStartAddress + sizeof(uint16_t);
	const byte addrSiding3Steps = varStartAddress + 2 * sizeof(uint16_t);

	struct CV16bit
	{
		const uint16_t  address;
		const uint16_t  Value;
		const bool      SoftReset;
	};

	const CV16bit FactoryDefaultSettings[3] =
	{
		{ addrSiding1Steps, 0, false },
		{ addrSiding2Steps, 4 * stepsPerSiding, false },
		{ addrSiding2Steps, 8 * stepsPerSiding, false },
	};


	// event handlers  ===========================================================================

	void ButtonEventHandler(bool state, unsigned int data);
	static void StepperClockwiseStep();
	static void StepperCounterclockwiseStep();

	// wrappers for callbacks
	static void WrapperButtonHandler(void* p, bool state, unsigned int data);
};

#endif
