// Touchpad.h

#ifndef _TOUCHPAD_h
#define _TOUCHPAD_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


#include "GraphicButton.h"
#include "Adafruit_ILI9341.h"
#include <Adafruit_FT6206.h>
#include <Wire.h>


class Touchpad
{
public:
	typedef void(*GraphicButtonHandler)(byte buttonID, bool state);

	Touchpad();
	void Init();
	void Update();
	void Update(uint32_t curMillis);
	void SetGraphicButtonHandler(GraphicButtonHandler handler);
	void SetButtonPress(byte buttonID, bool isPressed);

	enum buttonIDs : byte
	{
		// these are available via dcc extended accessory command
		runReverse = 0,

		numpad1 = 1,
		numpad2 = 2,
		numpad3 = 3,
		numpad4 = 4,
		numpad5 = 5,
		numpad6 = 6,
		numpad7 = 7,
		numpad8 = 8,
		numpad9 = 9,
		numpad10 = 10,
		numpad11 = 11,
		numpad12 = 12,
		numpad13 = 13,
		numpad14 = 14,
		numpad15 = 15,
		numpad16 = 16,
		numpad17 = 17,
		numpad18 = 18,

		setup10CW = 21,
		setup10CCW = 22,
		setup30CW = 23,
		setup30CCW = 24,
		setup90CW = 25,
		setup90CCW = 26,

		setupStepCW = 31,
		setupStepCCW = 32,

		// these are only available via touchscreen
		setupHome = 40,
		setupSet = 41,

		modeRun1 = 50,
		modeRun2 = 51,
		modeSetup = 52,

		estop = 110,
	};


private:

	// hardware assignments ============================================================================
	enum : byte
	{
		microSDPin = 4,
		backlightPin = 5,
		touchscreenIntPin = 7,
		TFT_DC_Pin = 9,
		TFT_CS_Pin = 10,
		ICSP_MOSI_Pin = 11,   // these are the defaults for the adafruilt display
		ICSP_MISO_Pin = 12,
		ICS_PSCLK_Pin = 13,
	};

	// State machine ======================================================================================

	enum touchpadState : byte
	{
		IDLE,           // touchscreen is untouched, no touch to handle, but should still update display
		TOUCHED,        // touchscreen is touched
		SLEEP,          // touchscreen backlight is off
	};

	touchpadState currentState = IDLE;
	byte subState = 0;    // track sequence of events within a state, 0 = transition steps for entering that state

	// state pointer and functions
	typedef void(Touchpad::*StateFunctionPointer)();
	StateFunctionPointer currentStateFunction = 0;

	// the state transition functions
	void transitionToIdle();
	void runIdle();
	void transitionToTouched();
	void runTouched();
	void transitionToSleep();
	void runSleep();


	// tft display and touchscreen setup   ========================================================

	Adafruit_ILI9341 display = Adafruit_ILI9341(TFT_CS_Pin, TFT_DC_Pin);      // display
	Adafruit_FT6206 touchscreen = Adafruit_FT6206();                              // touchscreen

	#define TFT_rotation 0
	enum : uint16_t { white = 0xFFFF };
	uint16_t touchx = 0;
	uint16_t touchy = 0;

	enum : byte
	{
		debounceTouch = 50,       // ms, to eliminate spurious touch/release cycles
		debounceRelease = 100,
	};

	enum : uint32_t { sleepTimeout = 5 * 60 * 1000 };

	uint32_t lastDebounceTime = 0;
	uint32_t lastTouchTime = 0;
	uint32_t currentMillis = 0;

	void ConfigureTouchscreen();
	void ConfigureRunPage1();
	void ConfigureRunPage2();
	void ConfigureSetupPage();

	// button setup
	enum : byte {
		numButtons = 14,
		groupNone = 0,
		groupSiding = 1,
	};
	GraphicButton button[numButtons];

	// event handlers
	void ButtonPress(GraphicButton* btn);
	void ButtonRelease(GraphicButton* b);

	GraphicButtonHandler graphicButtonHandler = 0;
};

#endif
