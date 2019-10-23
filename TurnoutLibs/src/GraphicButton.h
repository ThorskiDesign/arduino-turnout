// GraphicButton.h

#ifndef _GRAPHICBUTTON_h
#define _GRAPHICBUTTON_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
//#include "Fonts/FreeSansBold12pt7b.h"

// Color definitions from Adafruit
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

#define TTMENU_ON  0x059E
#define TTMENU_OFF 0x84D6

class GraphicButton
{
public:
	typedef void(*GraphicButtonHandler)(void* callbackObject, bool state, unsigned int data);

	enum GraphicButtonType {
		TOGGLE,      // Button behaves as a latching on/off switch
		MOMENTARY,     // Button behaves as a momentary pushbutton
		INDICATOR      // Button is for graphic indication only, no input
	};

	enum GraphicButtonShape {
		RECTANGLE,
		ROUNDRECT,
		CIRCLE
	};

	GraphicButton(Adafruit_ILI9341* tft, GraphicButtonType type, GraphicButtonShape shp,
		unsigned int xpos, unsigned int ypos, unsigned int xsize, unsigned int ysize, String lbl, byte id);
	void SetLabel(String l, bool show);
	void SetColors(unsigned int on, unsigned int off, unsigned int text, unsigned int border);

	void SetState(bool state);
	void SetActive(bool active);

	bool Press(unsigned int xpos, unsigned int ypos);   // button press and release
	void Release();

	void SetButtonHandler(void* cbObject, GraphicButtonHandler handler);


private:
	Adafruit_ILI9341* tftdisplay;
	GraphicButtonType type = TOGGLE;
	GraphicButtonShape shape = ROUNDRECT;
	unsigned int xpos = 0;
	unsigned int ypos = 0;
	unsigned int xsize = 70;   // also radius for circular buttons
	unsigned int ysize = 40;
	unsigned int textx = 0;
	unsigned int texty = 0;
	byte cornerRadius = 12;
	byte borderThickness = 2;
	byte textSize = 2;
	bool showLabel = true;
	String label = "";
	byte btnID = 0;
	unsigned int xmin, xmax, ymin, ymax;    //  bounding box limits in screen coordinates

	bool active = false;          // should the button be displayed and touchable
	bool switchState = false;     // is the logical state of the button on or off
	bool btnIsPressed = false;     // is the touchscreen pressed
	bool redrawPending = false;     // flag to skip redraw in spots if we are going to do it anyway

	unsigned int onColor = TTMENU_ON;
	unsigned int offColor = TTMENU_OFF;
	unsigned int textColor = BLACK;
	unsigned int borderColor = BLACK;

	void UpdateBoundingBox();  // update the bounding box for the current size and position
	void DrawButton();         // draw the button with the current settings

	GraphicButtonHandler buttonHandler = 0;
	void* callbackObject;
};

#endif
