
#include "TurntableMgr.h"

TurntableMgr::TurntableMgr()
{
}

void TurntableMgr::Initialize()
{
	// setup display
	tft.begin();
	tft.setRotation(1);
	tft.fillScreen(WHITE);

#ifdef _DEBUG
	// read diagnostics (optional but can help debug problems)
	uint8_t x = tft.readcommand8(ILI9341_RDMODE);
	Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDMADCTL);
	Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDPIXFMT);
	Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDIMGFMT);
	Serial.print("Image Format: 0x"); Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDSELFDIAG);
	Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX);
#endif  // DEBUG

	// setup touchscreen
	ctp.begin();

	ConfigureButtons();
}

void TurntableMgr::Update()
{
	bool touched = ctp.touched();
	if (touched && lastbtn == -1)
	{
		TS_Point p = ctp.getPoint();

		// flip touchscreen coords around to match the tft coords (only valid for tft rotation = 1)
		unsigned int x = map(p.y, 0, 320, 320, 0);
		unsigned int y = p.x;

		//Serial.print("ts coords:  "); Serial.print(x); Serial.print("  "); Serial.println(y);

		// loop through buttons to check
		byte i = 0;
		while (lastbtn == -1 && i < numButtons)
		{
			if (button[i].Press(x, y))
				lastbtn = i;
			i++;
		}
	}

	if (!touched)
	{
		button[lastbtn].Release();
		lastbtn = -1;
	}
}

void TurntableMgr::ConfigureButtons()
{
	for (int i = 0; i < numButtons; i++)
	{
		button[i].SetButtonHandler(this, WrapperButtonHandler);
		button[i].SetActive(true);
	}
}

void TurntableMgr::WrapperButtonHandler(void* p, bool State, unsigned int Data)
{
	((TurntableMgr*)p)->ButtonEventHandler(State, Data);
}

void TurntableMgr::ButtonEventHandler(bool State, unsigned int Data)
{
	if (State)
	{
		button[Data].SetLabel("On", true);
		//button[0].SetState(true);
	}
	else
	{
		button[Data].SetLabel("Off", true);
//		button[0].SetState(false);
	}
}
