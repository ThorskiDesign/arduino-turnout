
#include "GraphicButton.h"

GraphicButton::GraphicButton()
{
}

void GraphicButton::Init(Adafruit_ILI9341* tft, GraphicButtonType t, GraphicButtonShape shp,
	unsigned int xp, unsigned int yp, byte xs, byte ys, String lbl, byte id, byte group)
{
	tftdisplay = tft;
	type = t;
	shape = shp;
	xpos = xp;
	ypos = yp;
	xsize = xs;
	ysize = ys;
	btnID = id;

	label = lbl;

	showLabel = true;
	active = true;
	state = false;
	buttonGroup = group;

	UpdateBoundingBox();
}

void GraphicButton::SetLabel(String l, bool show)
{
	label = l;
	showLabel = show;
	UpdateBoundingBox();
}

void GraphicButton::SetActive(bool isActive)
{
	active = isActive;
}

bool GraphicButton::Contains(const unsigned int x, const unsigned int y)
{
	if (!active) return false;
	
	const bool inBounds = x >= xmin && x <= xmax && y >= ymin && y <= ymax;
	return inBounds;
}

void GraphicButton::Press(bool State)
{
	if (state == State) return;
	state = State;
	DrawButton();
}

bool GraphicButton::IsPressed()
{
	return state;
}

void GraphicButton::UpdateBoundingBox()
{
	switch (shape)
	{
	case GraphicButton::RECTANGLE:
	case GraphicButton::ROUNDRECT:
		xmin = xpos;
		xmax = xpos + xsize;
		ymin = ypos;
		ymax = ypos + ysize;
		cornerRadius = xsize / 6;
		if (ysize > xsize) cornerRadius = ysize / 6;
		break;
	case GraphicButton::CIRCLE:
		xmin = xpos - xsize;   // radius is xsize
		xmax = xpos + xsize;
		ymin = ypos - xsize;
		ymax = ypos + xsize;
		break;
	default:
		break;
	}


}

void GraphicButton::DrawButton()
{
	if (!active) return;
	
	const unsigned int color = state ? onColor : offColor;
	byte t = borderThickness;

	uint16_t centerx = 0;
	uint16_t centery = 0;

	switch (shape)
	{
	case GraphicButton::RECTANGLE:
		centerx = (xmin + xmax) / 2;
		centery = (ymin + ymax) / 2;
		tftdisplay->fillRect(xpos, ypos, xsize, ysize, color);
		tftdisplay->drawRect(xpos, ypos, xsize, ysize, borderColor);
		break;
	case GraphicButton::ROUNDRECT:
		centerx = (xmin + xmax) / 2;
		centery = (ymin + ymax) / 2;
		tftdisplay->fillRoundRect(xpos, ypos, xsize, ysize, cornerRadius, color);
		tftdisplay->drawRoundRect(xpos, ypos, xsize, ysize, cornerRadius, borderColor);
		break;
	case GraphicButton::CIRCLE:
		centerx = xpos;
		centery = ypos;
		tftdisplay->fillCircle(xpos, ypos, xsize, color);
		tftdisplay->drawCircle(xpos, ypos, xsize, borderColor);
		break;
	default:
		break;
	}

	if (showLabel)
	{
		// set text origin (same for all shapes once we have the center point)
		int16_t x1, y1;
		uint16_t w, h;
		tftdisplay->setTextSize(textSize);
		tftdisplay->getTextBounds(label, 0, 0, &x1, &y1, &w, &h);

		const unsigned int textx = centerx - w / 2;
		const unsigned int texty = centery - h / 2;   // y is top line for standard font

		tftdisplay->setCursor(textx, texty);
		tftdisplay->setTextColor(textColor);
		tftdisplay->setTextSize(textSize);
		tftdisplay->print(label);
	}
}

GraphicButton::GraphicButtonType GraphicButton::Type()
{
	return type;
}

byte GraphicButton::ButtonID()
{
	return btnID;
}

byte GraphicButton::ButtonGroup()
{
	return buttonGroup;
}
