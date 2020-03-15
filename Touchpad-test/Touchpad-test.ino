/*
 Name:		Touchpad_test.ino
 Created:	11/15/2019 7:15:02 AM
 Author:	eric
*/


#include "Touchpad.h"
Touchpad touchpad;

// the setup function runs once when you press reset or power the board
void setup() 
{

	touchpad.Init();
}

// the loop function runs over and over again until power down or reset
void loop() 
{

	touchpad.Update(millis());
}
