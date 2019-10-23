// TurntableMgr.h

#ifndef _TURNTABLEMGR_h
#define _TURNTABLEMGR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "Bitstream.h"
#include "DCCpacket.h"
#include "DCCdecoder.h"
#include "GraphicButton.h"
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Adafruit_FT6206.h>



// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10


class TurntableMgr
{
public:
	TurntableMgr();
	void Initialize();
	void Update();

private:
	// tft display
	// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
	Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

	// touchscreen
	Adafruit_FT6206 ctp = Adafruit_FT6206();

	const byte numButtons = 4;
	GraphicButton button[4]{
		{ &tft, GraphicButton::TOGGLE, GraphicButton::ROUNDRECT, 10, 10, 80, 40, "Run", 0 },
		{ &tft, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, 100, 10, 80, 40, "Setup", 1 },
		{ &tft, GraphicButton::TOGGLE, GraphicButton::ROUNDRECT, 10, 60, 80, 40, "Run", 2 },
		{ &tft, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, 100, 60, 80, 40, "Setup", 3 },
	};

	int lastbtn = -1;

	void ConfigureButtons();

	// event handlers
	void ButtonEventHandler(bool State, unsigned int Data);

	// wrappers for callbacks
	static void WrapperButtonHandler(void* p, bool State, unsigned int Data);
};

#endif
