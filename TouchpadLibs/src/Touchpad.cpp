
#include "Touchpad.h"

Touchpad::Touchpad()
{
}

void Touchpad::Init()
{

	ConfigureTouchscreen();
	ConfigureRunPage1();
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
			if (graphicButtonHandler) graphicButtonHandler(b.ButtonID(), false);
		}
	}

	lastTouchTime = currentMillis;

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

	// check for sleep
	if (currentMillis - lastTouchTime > sleepTimeout)
		transitionToSleep();
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
				if (graphicButtonHandler) graphicButtonHandler(b.ButtonID(), true);
			}
			if (b.Type() == GraphicButton::LATCHING && !b.IsPressed())
			{
				ButtonPress(&b);
				if (graphicButtonHandler) graphicButtonHandler(b.ButtonID(), true);
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

void Touchpad::transitionToSleep()
{
	// turn off the display backlight
	digitalWrite(backlightPin, LOW);

	currentState = SLEEP;
	currentStateFunction = &Touchpad::runSleep;
	subState = 0;
}

void Touchpad::runSleep()
{
	// wait for a touch/release cycle, then transition to idle
	switch (subState)
	{
	case 0:   // waiting for touch
		if (!touchscreen.touched()) lastDebounceTime = currentMillis;
		if (touchscreen.touched() && (currentMillis - lastDebounceTime > debounceTouch))
			subState = 1;
		break;
	case 1:   // waiting for release
		if (touchscreen.touched()) lastDebounceTime = currentMillis;
		if (!touchscreen.touched() && (currentMillis - lastDebounceTime > debounceRelease))
		{
			digitalWrite(backlightPin, HIGH);
			transitionToIdle();
		}
		break;
	}
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

void Touchpad::ConfigureRunPage1()
{
	display.fillScreen(white);

	// siding buttons
	byte xs = 70;
	byte ys = 50;
	byte xoff = 20;
	byte yoff = 20;
	button[0].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (2 * ys) + yoff, 60, 40, "1", numpad1, groupSiding);
	button[1].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (2 * ys) + yoff, 60, 40, "2", numpad2, groupSiding);
	button[2].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (2 * ys) + yoff, 60, 40, "3", numpad3, groupSiding);
	button[3].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (3 * ys) + yoff, 60, 40, "4", numpad4, groupSiding);
	button[4].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (3 * ys) + yoff, 60, 40, "5", numpad5, groupSiding);
	button[5].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (3 * ys) + yoff, 60, 40, "6", numpad6, groupSiding);
	button[6].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (4 * ys) + yoff, 60, 40, "7", numpad7, groupSiding);
	button[7].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (4 * ys) + yoff, 60, 40, "8", numpad8, groupSiding);
	button[8].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (4 * ys) + yoff, 60, 40, "9", numpad9, groupSiding);

	// reverse button
	button[12].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (5 * ys) + yoff, 200, 40, "Reverse", runReverse, groupSiding);

	// mode buttons
	xs = 80;
	ys = 40;
	xoff = 0;
	yoff = 0;
	button[9].Init(&display, GraphicButton::LATCHING, GraphicButton::RECTANGLE, (0 * xs) + xoff, (0 * ys) + yoff, xs, ys, "1-9", modeRun1, groupNone);
	button[10].Init(&display, GraphicButton::LATCHING, GraphicButton::RECTANGLE, (1 * xs) + xoff, (0 * ys) + yoff, xs, ys, "10-18", modeRun2, groupNone);
	button[11].Init(&display, GraphicButton::LATCHING, GraphicButton::RECTANGLE, (2 * xs) + xoff, (0 * ys) + yoff, xs, ys, "Setup", modeSetup, groupNone);
	button[9].Press(true);

	// emergency stop button
	xs = 110;
	ys = 50;
	xoff = 20;
	yoff = 10;
	button[13].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (1 * ys) + yoff, 90, 40, "EStop", estop, groupNone);

	// deactivate unused buttons
	for (byte i = 14; i < numButtons; i++)
		button[i].SetActive(false);

	for (auto& b : button)
		b.DrawButton();
}

void Touchpad::ConfigureRunPage2()
{
	display.fillScreen(white);

	// siding buttons
	byte xs = 70;
	byte ys = 50;
	byte xoff = 20;
	byte yoff = 20;
	button[0].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (2 * ys) + yoff, 60, 40, "10", numpad10, groupSiding);
	button[1].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (2 * ys) + yoff, 60, 40, "11", numpad11, groupSiding);
	button[2].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (2 * ys) + yoff, 60, 40, "12", numpad12, groupSiding);
	button[3].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (3 * ys) + yoff, 60, 40, "13", numpad13, groupSiding);
	button[4].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (3 * ys) + yoff, 60, 40, "14", numpad14, groupSiding);
	button[5].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (3 * ys) + yoff, 60, 40, "15", numpad15, groupSiding);
	button[6].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (4 * ys) + yoff, 60, 40, "16", numpad16, groupSiding);
	button[7].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (4 * ys) + yoff, 60, 40, "17", numpad17, groupSiding);
	button[8].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (4 * ys) + yoff, 60, 40, "18", numpad18, groupSiding);

	// reverse button
	button[12].Init(&display, GraphicButton::LATCHING, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (5 * ys) + yoff, 200, 40, "Reverse", runReverse, groupSiding);

	// mode buttons
	xs = 80;
	ys = 40;
	xoff = 0;
	yoff = 0;
	button[9].Init(&display, GraphicButton::LATCHING, GraphicButton::RECTANGLE, (0 * xs) + xoff, (0 * ys) + yoff, xs, ys, "1-9", modeRun1, groupNone);
	button[10].Init(&display, GraphicButton::LATCHING, GraphicButton::RECTANGLE, (1 * xs) + xoff, (0 * ys) + yoff, xs, ys, "10-18", modeRun2, groupNone);
	button[11].Init(&display, GraphicButton::LATCHING, GraphicButton::RECTANGLE, (2 * xs) + xoff, (0 * ys) + yoff, xs, ys, "Setup", modeSetup, groupNone);
	button[10].Press(true);

	// emergency stop button
	xs = 110;
	ys = 50;
	xoff = 20;
	yoff = 10;
	button[13].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (1 * ys) + yoff, 90, 40, "EStop", estop, groupNone);

	// deactivate unused buttons
	for (byte i = 14; i < numButtons; i++)
		button[i].SetActive(false);

	for (auto& b : button)
		b.DrawButton();
}

void Touchpad::ConfigureSetupPage()
{
	display.fillScreen(white);

	// calibration buttons
	byte xs = 70;
	byte ys = 50;
	byte xoff = 20;
	byte yoff = 20;
	button[0].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (2 * ys) + yoff, 60, 40, "+10", setup10CW, groupNone);
	button[1].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (2 * ys) + yoff, 60, 40, "-10", setup10CCW, groupNone);
	button[2].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (3 * ys) + yoff, 60, 40, "+30", setup30CW, groupNone);
	button[3].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (3 * ys) + yoff, 60, 40, "-30", setup30CCW, groupNone);
	button[4].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (4 * ys) + yoff, 60, 40, "+90", setup90CW, groupNone);
	button[5].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (4 * ys) + yoff, 60, 40, "-90", setup90CCW, groupNone);
	button[6].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (5 * ys) + yoff, 60, 40, "CW", setupStepCW, groupNone);
	button[7].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (5 * ys) + yoff, 60, 40, "Set", setupSet, groupNone);
	button[8].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (2 * xs) + xoff, (5 * ys) + yoff, 60, 40, "CCW", setupStepCCW, groupNone);

	// mode buttons
	xs = 80;
	ys = 40;
	xoff = 0;
	yoff = 0;
	button[9].Init(&display, GraphicButton::LATCHING, GraphicButton::RECTANGLE, (0 * xs) + xoff, (0 * ys) + yoff, xs, ys, "1-9", modeRun1, groupNone);
	button[10].Init(&display, GraphicButton::LATCHING, GraphicButton::RECTANGLE, (1 * xs) + xoff, (0 * ys) + yoff, xs, ys, "10-18", modeRun2, groupNone);
	button[11].Init(&display, GraphicButton::LATCHING, GraphicButton::RECTANGLE, (2 * xs) + xoff, (0 * ys) + yoff, xs, ys, "Setup", modeSetup, groupNone);
	button[11].Press(true);

	// emergency stop button
	xs = 110;
	ys = 50;
	xoff = 20;
	yoff = 10;
	button[12].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (1 * xs) + xoff, (1 * ys) + yoff, 90, 40, "EStop", estop, groupNone);

	// home button
	button[13].Init(&display, GraphicButton::MOMENTARY, GraphicButton::ROUNDRECT, (0 * xs) + xoff, (1 * ys) + yoff, 90, 40, "Home", setupHome, groupNone);

	// deactivate unused buttons
	for (byte i = 14; i < numButtons; i++)
		button[i].SetActive(false);

	for (auto& b : button)
		b.DrawButton();
}

void Touchpad::SetButtonPress(byte buttonID, bool isPressed)
{
	// find the button matching the requested ID
	byte i = 0;
	while ((i < numButtons) && (button[i].ButtonID() != buttonID)) i++;

	// if there is no match just return
	if (i == numButtons) return;

	// set the button
	if (button[i].IsPressed() != isPressed)
	{
		if (isPressed)
			ButtonPress(&button[i]);
		else
			ButtonRelease(&button[i]);
	}
}

void Touchpad::ButtonPress(GraphicButton* btn)
{
	switch (btn->ButtonID())
	{
	// if this is a mode button, switch pages
	case modeRun1:
		ConfigureRunPage1();
		break;
	case modeRun2:
		ConfigureRunPage2();
		break;
	case modeSetup:
		ConfigureSetupPage();
		break;

	// other buttons
	default:
		// if the button is in a group, disable the group members
		byte group = btn->ButtonGroup();
		if (group != groupNone)
			for (auto& b : button)
				if (b.ButtonGroup() == group)
					b.Press(false);

		// set new button state
		btn->Press(true);
		break;
	}
}

void Touchpad::ButtonRelease(GraphicButton* b)
{
	// set new button state
	b->Press(false);
}
