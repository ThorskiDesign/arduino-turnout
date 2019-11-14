
#include "GraphicButton.h"

GraphicButton::GraphicButton()
{
}

GraphicButton::GraphicButton(Adafruit_ILI9341* tft, GraphicButtonType t, GraphicButtonShape shp,
	unsigned int xp, unsigned int yp, byte xs, byte ys, String lbl, byte id)
{
	tftdisplay = tft;
	type = t;
	shape = shp;
	xpos = xp;
	ypos = yp;
	xsize = xs;
	ysize = ys;
	label = lbl;
	btnID = id;

	UpdateBoundingBox();
}

void GraphicButton::SetLabel(String l, bool show)
{
	label = l;
	showLabel = show;
	UpdateBoundingBox();
	if (!redrawPending) DrawButton();
}

//void GraphicButton::SetColors(unsigned int on, unsigned int off, unsigned int text, unsigned int border)
//{
//	onColor = on;
//	offColor = off;
//	textColor = text;
//	borderColor = border;
//	if (!redrawPending) DrawButton();
//}

void GraphicButton::SetState(const bool state)
{
	switchState = state;
	DrawButton();
}

void GraphicButton::SetActive(const bool a)
{
	active = a;
	DrawButton();
}

bool GraphicButton::Press(const unsigned int x, const unsigned int y)
{
	if (!active) return false;
	if (btnIsPressed) return false;
	if (type == INDICATOR) return false;

	const bool inBounds = x >= xmin && x <= xmax && y >= ymin && y <= ymax;

	if (inBounds)
	{
		if (type == TOGGLE)	switchState = !switchState;
		if (type == MOMENTARY) switchState = true;

		btnIsPressed = true;
		redrawPending = true;
		if (buttonHandler) buttonHandler(callbackObject, switchState, btnID);
		DrawButton();

#ifdef _DEBUG
		Serial.print("Button pressed: "); Serial.println(btnID);
#endif // _DEBUG

		return true;
	}

	return false;
}

void GraphicButton::Release()
{
	if (!active) return;
	if (!btnIsPressed) return;

	if (type == MOMENTARY)
	{
		switchState = false;
		redrawPending = true;
		if (buttonHandler) buttonHandler(callbackObject, switchState, btnID);
		DrawButton();
	}

#ifdef _DEBUG
	Serial.print("Button released: "); Serial.println(btnID);
#endif // _DEBUG

	btnIsPressed = false;
	return;
}

void GraphicButton::SetButtonHandler(void* cbObject, const GraphicButtonHandler handler)
{
	callbackObject = cbObject;
	buttonHandler = handler;
}

void GraphicButton::UpdateBoundingBox()
{
	unsigned int centerx = 0;
	unsigned int centery = 0;

	switch (shape)
	{
	case GraphicButton::RECTANGLE:
	case GraphicButton::ROUNDRECT:
		xmin = xpos;
		xmax = xpos + xsize;
		ymin = ypos;
		ymax = ypos + ysize;
		centerx = (xmin + xmax) / 2;
		centery = (ymin + ymax) / 2;
		cornerRadius = xsize / 6;
		if (ysize > xsize) cornerRadius = ysize / 6;
		break;
	case GraphicButton::CIRCLE:
		xmin = xpos - xsize;   // radius is xsize
		xmax = xpos + xsize;
		ymin = ypos - xsize;
		ymax = ypos + xsize;
		centerx = xpos;
		centery = ypos;
		break;
	default:
		break;
	}

	// set text origin (same for all shapes once we have the center point)
	int x1, y1;
	unsigned int w, h;
//	tftdisplay->setFont(&FreeSansBold12pt7b);
	tftdisplay->setTextSize(textSize);
	tftdisplay->getTextBounds(label, 0, 0, &x1, &y1, &w, &h);

	textx = centerx - w / 2;
	texty = centery - h / 2;   // y is top line for standard font
//	texty = centery + h / 2;   // y is baseline for highrez fonts
}

void GraphicButton::DrawButton()
{
#ifdef _DEBUG
	Serial.print("draw button: "); Serial.println(btnID);
#endif // _DEBUG

	const unsigned int Color = switchState ? onColor : offColor;
	byte t = borderThickness;

	switch (shape)
	{
	case GraphicButton::RECTANGLE:
		tftdisplay->fillRect(xpos, ypos, xsize, ysize, Color);
		tftdisplay->drawRect(xpos, ypos, xsize, ysize, borderColor);
		//tftdisplay->fillRect(xpos + t, ypos + t, xsize - 2 * t, ysize - 2 * t, Color);
		break;
	case GraphicButton::ROUNDRECT:
		tftdisplay->fillRoundRect(xpos, ypos, xsize, ysize, cornerRadius, Color);
		tftdisplay->drawRoundRect(xpos, ypos, xsize, ysize, cornerRadius, borderColor);
		//tftdisplay->fillRoundRect(xpos + t, ypos + t, xsize - 2 * t, ysize - 2 * t, cornerRadius, Color);
		break;
	case GraphicButton::CIRCLE:
		tftdisplay->fillCircle(xpos, ypos, xsize, Color);
		tftdisplay->drawCircle(xpos, ypos, xsize, borderColor);
		//tftdisplay->fillCircle(xpos, ypos, xsize - 2 * t, Color);
		break;
	default:
		break;
	}

	if (showLabel)
	{
		tftdisplay->setCursor(textx, texty);
		tftdisplay->setTextColor(textColor);
//		tftdisplay->setFont(&FreeSansBold12pt7b);
		tftdisplay->setTextSize(textSize);
		tftdisplay->print(label);
	}

	redrawPending = false;
}
