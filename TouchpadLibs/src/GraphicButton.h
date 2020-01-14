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
	enum GraphicButtonType : byte {
		LATCHING,      // Button behaves as a latching switch, disabled by pressing another button
		TOGGLE,        // Button behaves as an on/off toggle switch
		MOMENTARY,     // Button behaves as a momentary pushbutton
		INDICATOR      // Button is for graphic indication only, no input
	};

	enum GraphicButtonShape : byte {
		RECTANGLE,
		ROUNDRECT,
		CIRCLE
	};

	GraphicButton();
	void Init(Adafruit_ILI9341* tft, GraphicButtonType t, GraphicButtonShape shp,
		unsigned int xp, unsigned int yp, byte xs, byte ys, String lbl, byte id);
	void SetLabel(String l, bool show);
	void SetActive(bool isActive);
	bool Contains(unsigned int x, unsigned int y);
	void Press(bool state);
	bool IsPressed();
	void DrawButton();         // draw the button with the current settings
	GraphicButtonType Type();
	byte ButtonID();

private:
	enum : uint16_t
	{
	onColor = 0x059E,      // light blue
	offColor = 0x84D6,     // gray blue
	textColor = 0x0000,    // black
	borderColor = 0x0000,  // black
	};

	Adafruit_ILI9341* tftdisplay = 0;
	
	GraphicButtonType type = LATCHING;
	GraphicButtonShape shape = ROUNDRECT;
	unsigned int xpos = 0;
	unsigned int ypos = 0;
	byte xsize = 70;   // also radius for circular buttons
	byte ysize = 40;
	byte cornerRadius = 12;
	
	enum : byte { borderThickness = 2 };
	
	byte textSize = 2;
	String label;
	byte btnID = 0;
	unsigned int xmin = 0, xmax = 0, ymin = 0, ymax = 0;    //  bounding box limits in screen coordinates

	bool showLabel = true;
	bool active = true;          // should the button be displayed and touchable
	bool state = false;     // is the logical state of the button on or off

	void UpdateBoundingBox();  // update the bounding box for the current size and position
};

#endif
