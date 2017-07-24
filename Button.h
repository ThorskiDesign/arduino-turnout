
/*

Button

A class providing a debounced button input.

Summary:

Creates a debounced button object on a pin and sets the use of the internal pullup resistor. The update
method checks for changes in the pin state. If a change is detected, a start time is set. After the
debounce interval has elapsed without further state changes, the button press handler is called to 
report the current state. The debounced button state may also be accessed directly using the
SwitchState method.

Example usage:

	Button button(ButtonPin, true);            // create a button object on a given pin, with pullup enabled
	button.Update(currentMillis);              // update state of the debounce logic
	button.SetButtonPressHandler(handler);     // set the handler for the button press

*/


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
    typedef void (*ButtonPressHandlerFunc)(bool switchState);

    Button(byte Pin, bool EnablePullup);
	void Update(unsigned long CurrentMillis);
	void Update();
	byte SwitchState();
	byte RawState();
	int NumUpdates();
	int NumInterrupts();
	bool HasChanged();
	void SetButtonPressHandler(ButtonPressHandlerFunc Handler);

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
	ButtonPressHandlerFunc buttonPressHandler = 0;   // pointer to handler for button press event
};

#endif
