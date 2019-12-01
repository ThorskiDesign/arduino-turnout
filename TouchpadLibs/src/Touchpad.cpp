
#include "Touchpad.h"

Touchpad::Touchpad()
{
}

void Touchpad::Init()
{

	ConfigureTouchscreen();
	ConfigureRunPage();
	transitionToIdle();
}

void Touchpad::SetGraphicButtonHandler(GraphicButtonHandler handler)
{
	graphicButtonHandler = handler;
}

void Touchpad::Update()
{
	currentMillis = millis();
	
	// perform the current state function
	if (currentStateFunction)
		(*this.*currentStateFunction)();
}

void Touchpad::Update(uint32_t curMillis)
{
	currentMillis = curMillis;

	// perform the current state function
	if (currentStateFunction)
		(*this.*currentStateFunction)();
}

void Touchpad::transitionToIdle()
{
	for (auto& b : button)
	{
		if (b.Type() == GraphicButton::MOMENTARY && b.IsPressed())
		{
			ButtonRelease(&b);
		}
	}

	currentState = IDLE;
	currentStateFunction = &Touchpad::runIdle;
	subState = 0;
}

void Touchpad::runIdle()
{
	// reset debounce timer
	if (!touchscreen.touched()) lastDebounceTime = currentMillis;

	// check for touchscreen touch
	if (touchscreen.touched() && (currentMillis - lastDebounceTime > debounceTouch))
		transitionToTouched();
}

void Touchpad::transitionToTouched()
{
	const TS_Point p = touchscreen.getPoint();

	// flip touchscreen coords around to match the tft coords
	#if TFT_rotation == 0
	touchx = map(p.x, 0, 240, 240, 0);
	touchy = map(p.y, 0, 320, 320, 0);
	#endif

	#if TFT_rotation == 1
	const unsigned int x = map(p.y, 0, 320, 320, 0);
	const unsigned int y = p.x;
	#endif

	// loop through buttons to check
	for (auto& b : button)
	{
		if (b.Contains(touchx, touchy))
		{
			if (b.Type() == GraphicButton::MOMENTARY)
			{
				ButtonPress(&b);
			}
			if (b.Type() == GraphicButton::LATCHING)
			{
				if (!b.IsPressed())
					ButtonPress(&b);
				//else
				//	ButtonRelease(&b);
			}
		}
	}

	currentState = TOUCHED;
	currentStateFunction = &Touchpad::runTouched;
	subState = 0;
}

void Touchpad::runTouched()
{
	// reset debounce timer
	if (touchscreen.touched()) lastDebounceTime = currentMillis;

	// check for touchscreen release
	if (!touchscreen.touched() && (currentMillis - lastDebounceTime > debounceRelease))
		transitionToIdle();
}


void Touchpad::ConfigureTouchscreen()
{

	// setup display
	pinMode(backlightPin, OUTPUT);
	digitalWrite(backlightPin, HIGH);

	display.begin();
	display.setRotation(TFT_rotation);

	#ifdef _DEBUG
	// read diagnostics (optional but can help debug problems)
	uint8_t x = display.readcommand8(ILI9341_RDMODE);
	Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
	x = display.readcommand8(ILI9341_RDMADCTL);
	Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
	x = display.readcommand8(ILI9341_RDPIXFMT);
	Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
	x = display.readcommand8(ILI9341_RDIMGFMT);
	Serial.print("Image Format: 0x"); Serial.println(x, HEX);
	x = display.readcommand8(ILI9341_RDSELFDIAG);
	Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);
	#endif  // DEBUG

	// setup touchscreen
	touchscreen.begin();
}

void Touchpad::ConfigureRunPage()
{
	display.fillScreen(white);

	ConfigureNumpad();

	// mode buttons
	button[9].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * 100) + 20, (0 * 50) + 10, 90, 40, "Run", modeRun);
	button[9].Press(true);
	button[10].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * 100) + 20, (0 * 50) + 10, 90, 40, "Setup", modeSetup);

	// reverse button
	button[11].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, 20, (5 * 50) + 5, 200, 40, "Reverse", runReverse);

	// deactivate unused buttons
	for (byte i = 12; i < numButtons; i++)
		button[i].SetActive(false);

	for (auto& b : button)
		b.DrawButton();
}

void Touchpad::ConfigureSetupPage()
{
	display.fillScreen(white);

	ConfigureNumpad();

	// mode buttons
	button[9].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * 100) + 20, (0 * 50) + 10, 90, 40, "Run", modeRun);
	button[10].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * 100) + 20, (0 * 50) + 10, 90, 40, "Setup", modeSetup);
	button[10].Press(true);

	// home button
	button[11].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * 100) + 20, (1 * 50) + 10, 90, 40, "Home", setupHome);
	
	// calibration buttons
	button[12].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (0 * 70) + 20, (5 * 50) + 5, 60, 40, "CW", setupCW);
	button[13].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (1 * 70) + 20, (5 * 50) + 5, 60, 40, "CCW", setupCCW);
	button[14].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (2 * 70) + 20, (5 * 50) + 5, 60, 40, "Set", setupSet);

	// deactivate unused buttons
	for (byte i = 15; i < numButtons; i++)
		button[i].SetActive(false);

	for (auto& b : button)
		b.DrawButton();
}


void Touchpad::ConfigureNumpad()
{
	const byte xs = 70;
	const byte ys = 50;
	const byte xoff = 20;
	const byte yoff = 5;

	// siding buttons
	button[0].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (2 * ys) + yoff, 60, 40, "1", numpad1);
	button[1].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (2 * ys) + yoff, 60, 40, "2", numpad2);
	button[2].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (2 * ys) + yoff, 60, 40, "3", numpad3);
	button[3].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (3 * ys) + yoff, 60, 40, "4", numpad4);
	button[4].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (3 * ys) + yoff, 60, 40, "5", numpad5);
	button[5].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (3 * ys) + yoff, 60, 40, "6", numpad6);
	button[6].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (4 * ys) + yoff, 60, 40, "7", numpad7);
	button[7].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (4 * ys) + yoff, 60, 40, "8", numpad8);
	button[8].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (4 * ys) + yoff, 60, 40, "9", numpad9);
}

bool Touchpad::IsSidingButton(byte buttonID)
{
	switch (buttonID)
	{
	case numpad0:
	case numpad1:
	case numpad2:
	case numpad3:
	case numpad4:
	case numpad5:
	case numpad6:
	case numpad7:
	case numpad8:
	case numpad9:
		return  true;
	default:
		return false;
	}
}

void Touchpad::ButtonPress(GraphicButton* btn)
{
	// if this is a mode button, switch pages
	if (btn->ButtonID() == modeRun) ConfigureRunPage();
	if (btn->ButtonID() == modeSetup) ConfigureSetupPage();

	// if this is a siding button, disable other siding buttons
	if (IsSidingButton(btn->ButtonID()))
		for (auto& b : button)
			if (IsSidingButton(b.ButtonID()))
				b.Press(false);

	// set new button state
	btn->Press(true);

	// perform callback
	if (graphicButtonHandler) graphicButtonHandler(btn->ButtonID(), true);
}

void Touchpad::ButtonRelease(GraphicButton* b)
{
	// set new button state
	b->Press(false);

	// perform callback
	if (graphicButtonHandler) graphicButtonHandler(b->ButtonID(), false);
}
