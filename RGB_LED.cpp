
#include "RGB_LED.h"

// Create a colored LED
RgbLed::RgbLed(byte PinR, byte PinG, byte PinB)
{
	// assign from params
	pinR = PinR;
	pinG = PinG;
	pinB = PinB;

	// set up pin modes and initial led config
	pinMode(pinR, OUTPUT);
	pinMode(pinG, OUTPUT);
	pinMode(pinB, OUTPUT);
	SetLED(WHITE, OFF);
}

void RgbLed::SetLED(bool State)
{
	if (State) { SetLED(ledColor,ON); }
	if (!State) { SetLED(ledColor,OFF); }
}

// Set the LED to a particular color and mode
void RgbLed::SetLED(ColorType C, ModeType T)
{
	if (C == ledColor && T == ledMode) return;

	//Serial.print("color: "); Serial.println(C, DEC);
	ledColor = C;
	ledMode = T;
	if (ledMode == ON) TurnColorsOn();
	if (ledMode == OFF) TurnColorsOff();
	if (ledMode == FLASH)
	{
		previousMillis = millis();
		ledState = LOW;
		TurnColorsOff();
	}
}

// Set the LED to a particular color and mode, with specified flash intervals.
void RgbLed::SetLED(ColorType C, ModeType T, int On, int Off)
{
	onTime = On;
	offTime = Off;
	SetLED(C, T);
}

// Turn the LED on and off if it is in flash mode.
void RgbLed::Update(unsigned long CurrentMillis)
{
	if (ledMode == FLASH)
	{
		if ((ledState == HIGH) && (CurrentMillis - previousMillis >= onTime))
		{
			ledState = LOW;                   // Turn it off
			previousMillis = CurrentMillis;   // Remember the time
			TurnColorsOff();
		}
		if ((ledState == LOW) && (CurrentMillis - previousMillis >= offTime))
		{
			ledState = HIGH;                  // turn it on
			previousMillis = CurrentMillis;   // Remember the time
			TurnColorsOn();
		}
	}
}

// In case we want to call update without specifying millis.
void RgbLed::Update()
{
	Update(millis());
}

// Turn on the individual elements as needed for the current color.
void RgbLed::TurnColorsOn()
{
	digitalWrite(pinR, redState[ledColor]);
	digitalWrite(pinG, greenState[ledColor]);
	digitalWrite(pinB, blueState[ledColor]);
}

// Turn off the individual elements.
void RgbLed::TurnColorsOff()
{
	digitalWrite(pinR, LOW);
	digitalWrite(pinG, LOW);
	digitalWrite(pinB, LOW);
}
