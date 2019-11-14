// GraphicButton.h

#ifndef _GRAPHICBUTTON_h
#define _GRAPHICBUTTON_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif


#include "Adafruit_ILI9341.h"
//#include "Fonts/FreeSansBold12pt7b.h"


class GraphicButton
{
public:
	typedef void(*GraphicButtonHandler)(void* callbackObject, bool state, unsigned int data);

	enum GraphicButtonType : byte {
		TOGGLE,      // Button behaves as a latching on/off switch
		MOMENTARY,     // Button behaves as a momentary pushbutton
		INDICATOR      // Button is for graphic indication only, no input
	};

	enum GraphicButtonShape : byte {
		RECTANGLE,
		ROUNDRECT,
		CIRCLE
	};

	GraphicButton();
	GraphicButton(Adafruit_ILI9341* tft, GraphicButtonType t, GraphicButtonShape shp,
		unsigned int xp, unsigned int yp, byte xs, byte ys, String lbl, byte id);
	void SetLabel(String l, bool show);
	//void SetColors(unsigned int on, unsigned int off, unsigned int text, unsigned int border);

	void SetState(bool state);
	void SetActive(bool active);

	bool Press(unsigned int x, unsigned int y);   // button press and release
	void Release();

	void SetButtonHandler(void* cbObject, GraphicButtonHandler handler);


private:
	// Color definitions from Adafruit
	const uint16_t black = 0x0000;
	const uint16_t white = 0xFFFF;
	const uint16_t ttmenuOn = 0x059E;
	const uint16_t ttmenuOff = 0x84D6;

	const uint16_t onColor = ttmenuOn;
	const uint16_t offColor = ttmenuOff;
	const uint16_t textColor = black;
	const uint16_t borderColor = black;

	Adafruit_ILI9341* tftdisplay;
	
	GraphicButtonType type = TOGGLE;
	GraphicButtonShape shape = ROUNDRECT;
	unsigned int xpos = 0;
	unsigned int ypos = 0;
	byte xsize = 70;   // also radius for circular buttons
	byte ysize = 40;
	byte textx = 0;
	byte texty = 0;
	byte cornerRadius = 12;
	byte borderThickness = 2;
	byte textSize = 2;
	String label = "";
	byte btnID = 0;
	unsigned int xmin, xmax, ymin, ymax;    //  bounding box limits in screen coordinates

	// flags
	//byte flags = 0b00000001;
	//const byte flagShowLabel = 1 << 0;
	//const byte flagActive = 1 << 1;
	//const byte flagSwitchState = 1 << 2;
	//const byte flagIsPressed = 1 << 3;
	//const byte flagRedrawPending = 1 << 4;

	bool showLabel = true;
	bool active = false;          // should the button be displayed and touchable
	bool switchState = false;     // is the logical state of the button on or off
	bool btnIsPressed = false;     // is the touchscreen pressed
	bool redrawPending = false;     // flag to skip redraw in spots if we are going to do it anyway

	void UpdateBoundingBox();  // update the bounding box for the current size and position
	void DrawButton();         // draw the button with the current settings

	GraphicButtonHandler buttonHandler = 0;
	void* callbackObject;
};

#endif
