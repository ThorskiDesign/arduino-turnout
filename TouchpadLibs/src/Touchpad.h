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


//#define numpad0 0
//#define numpad1 1
//#define numpad2 2
//#define numpad3 3
//#define numpad4 4
//#define numpad5 5
//#define numpad6 6
//#define numpad7 7
//#define numpad8 8
//#define numpad9 9
//#define modeRun 20
//#define modeSetup 21
//#define runReverse 30
//#define setupCW 40
//#define setupCCW 41
//#define setupSet 42


class Touchpad
{
public:
	typedef void(*GraphicButtonHandler)(byte buttonID, bool state);

	Touchpad();
	void Init();
	void Update();
	void Update(uint32_t curMillis);
	void SetGraphicButtonHandler(GraphicButtonHandler handler);

	// button IDs
	//const byte numpad0 = 0;
	//const byte numpad1 = 1;
	//const byte numpad2 = 2;
	//const byte numpad3 = 3;
	//const byte numpad4 = 4;
	//const byte numpad5 = 5;
	//const byte numpad6 = 6;
	//const byte numpad7 = 7;
	//const byte numpad8 = 8;
	//const byte numpad9 = 9;
	//const byte modeRun = 20;
	//const byte modeSetup = 21;
	//const byte runReverse = 30;
	//const byte setupCW = 40;
	//const byte setupCCW = 41;
	//const byte setupSet = 42;

	enum buttonIDs : byte
	{
		numpad0 = 0,
		numpad1 = 1,
		numpad2 = 2,
		numpad3 = 3,
		numpad4 = 4,
		numpad5 = 5,
		numpad6 = 6,
		numpad7 = 7,
		numpad8 = 8,
		numpad9 = 9,
		modeRun = 20,
		modeSetup = 21,
		runReverse = 30,
		setupCW = 40,
		setupCCW = 41,
		setupSet = 42,
		setupHome = 43,
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


	// tft display and touchscreen setup   ========================================================

	Adafruit_ILI9341 display = Adafruit_ILI9341(TFT_CS_Pin, TFT_DC_Pin);      // display
	Adafruit_FT6206 touchscreen = Adafruit_FT6206();                              // touchscreen

	#define TFT_rotation 0
	enum : uint16_t { white = 0xFFFF };
	uint16_t touchx = 0;
	uint16_t touchy = 0;

	uint16_t debounceTouch = 50;   // ms, to eliminate spurious touch/release cycles
	uint16_t debounceRelease = 100;
	uint32_t lastDebounceTime = 0;
	uint32_t currentMillis = 0;

	void ConfigureTouchscreen();
	void ConfigureRunPage();
	void ConfigureSetupPage();
	void ConfigureNumpad();

	// button setup
	enum : byte { numButtons = 15 };
	GraphicButton button[numButtons];

	// event handlers
	bool IsSidingButton(byte buttonID);
	void ButtonPress(GraphicButton* btn);
	void ButtonRelease(GraphicButton* b);

	GraphicButtonHandler graphicButtonHandler = 0;
};

#endif
