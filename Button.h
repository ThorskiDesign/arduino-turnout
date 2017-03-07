// button.h

#ifndef _BUTTON_h
#define _BUTTON_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class Button
{
public:
	Button(byte Pin, bool EnablePullup);
	void Update(unsigned long CurrentMillis);
	void Update();
	byte SwitchState();
	byte RawState();
	int NumUpdates();
	int NumInterrupts();
	bool HasChanged();
	void SetButtonPressHandler(void (*Handler)(bool));

private:
	bool readEnable = false;            // enable reading the switch after debounce interval
	unsigned long interruptTime = 0;    // time of the last change in the raw switch state
	byte debounceTime = 10;             // debounce interval (ms)
	byte pin = 0;                       // pin the button is attached to
	bool lastRawState = HIGH;           // the last raw state of the pin
	bool switchState = HIGH;            // the current debounced state of the switch
	int numUpdates = 0;                 // number of times the debounced state has changed
	int numInterrupts = 0;              // number of times the raw state has changed
	bool hasChanged = false;            // has the state of the switch changed (this is reset after reading the value)
	void (*buttonPressHandler)(bool);   // pointer to handler for button press event
};

#endif
