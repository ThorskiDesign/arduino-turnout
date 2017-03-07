// RGB_LED.h

#ifndef _RGB_LED_h
#define _RGB_LED_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class RgbLed
{
public:
	enum ModeType {OFF, ON, FLASH};   // mode of the LED
	enum ColorType {RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE};    // available colors

public:
	RgbLed(byte PinR, byte PinG, byte PinB);
	void SetLED(bool State);
	void SetLED(ColorType C, ModeType T);
	void SetLED(ColorType C, ModeType T, int On, int Off);
	void Update(unsigned long CurrentMillis);
	void Update();

private:
	byte pinR;            // the number of the red pin
	byte pinG;            // the number of the green pin
	byte pinB;            // the number of the blue pin
	int onTime = 500;     // milliseconds of on-time
	int offTime = 500;    // milliseconds of off-time

	// these maintain the state for a flashing LED
	bool ledState = LOW;                   // ledState used to set the LED
	unsigned long previousMillis = 0;      // will store last time LED was updated

	// constituent colors for the seven color types
	// order must correspond to the ordering in the ColorType enum
	const bool redState[7] = {HIGH, LOW, LOW, HIGH, LOW, HIGH, HIGH};
	const bool greenState[7] = {LOW, HIGH, LOW, HIGH, HIGH, LOW, HIGH};
	const bool blueState[7] = {LOW, LOW, HIGH, LOW, HIGH, HIGH, HIGH};

	ModeType ledMode;		// type of LED
	ColorType ledColor;     // color of LED

private:
	void TurnColorsOn();
	void TurnColorsOff();
};

#endif
