
#include "TurnoutMgr.h"
#include <Servo.h>
#include <NmraDcc.h>


TurnoutMgr TurnoutManager;


// ========================================================================================================
// global callback wrappers
// TODO: figure out how to call these directly in TurnoutManager so we don't clutter up global scope

void HandleButtonPressWrapper(bool ButtonState) { TurnoutManager.ButtonEventHandler(ButtonState); }
void HandleServoMoveDoneWrapper() { TurnoutManager.ServoMoveDoneHandler(); }
void HandleServoPowerOffWrapper() { TurnoutManager.ServoPowerOffHandler(); }
void HandleResetTimerWrapper() { TurnoutManager.ResetTimerHandler(); }
void HandleErrorTimerWrapper() { TurnoutManager.ErrorTimerHandler(); }
void HandleOSStraightWrapper(bool ButtonState) { TurnoutManager.OSStraightHandler(ButtonState); }
void HandleOSCurvedWrapper(bool ButtonState) { TurnoutManager.OSCurvedHandler(ButtonState); }


// ========================================================================================================
// dcc lib callbacks
// TODO: figure out how to call these directly in TurnoutManager so we don't clutter up global scope

// This function is called whenever a normal DCC Turnout Packet is received
void notifyDccAccTurnoutOutput( uint16_t Addr, uint8_t Direction, uint8_t OutputPower )
{
	//Serial.println("In notifyDccAccTurnoutOutput callback.");
	TurnoutManager.DCCcommandHandler(Addr, Direction);
}

void notifyCVChange(uint16_t CV, uint8_t Value)
{
#ifdef _DEBUG
	Serial.print("notifyCVChange: CV: ") ;
	Serial.print(CV,DEC) ;
	Serial.print(" Value: ") ;
	Serial.println(Value, DEC) ;
#endif

	TurnoutManager.CVchangeHandler(CV, Value);
}



// ========================================================================================================
// millis timer interrupt
SIGNAL(TIMER0_COMPA_vect)
{
	TurnoutManager.UpdateSensors();
}



// ========================================================================================================
// main setup
void setup()
{
	Serial.begin(115200);

	// Set up interrupt for Timer0.
	// See https://learn.adafruit.com/multi-tasking-the-arduino-part-2/timers
	OCR0A = 0xAF;
	TIMSK0 |= _BV(OCIE0A);

	// initialize the turnout manager
	TurnoutManager.Initialize();
}



unsigned long loopCount = 0;
unsigned long loopCountStartTime = 0;
unsigned long loopInterval = 1000;

// ========================================================================================================
// main loop
void loop()
{
#ifdef _DEBUG
	loopCount++;
	if ((millis() - loopCountStartTime) > loopInterval)
	{
		Serial.print("loop count = ");
		Serial.println(loopCount,DEC);
		loopCountStartTime = millis();
		loopCount = 0;
	}

#endif

	TurnoutManager.UpdateDccProcess();
}
