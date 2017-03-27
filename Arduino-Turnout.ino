
#include <EEPROM.h>
#include <Servo.h>
#include "TurnoutMgr.h"


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

void HandleDCCAccPacket(int boardAddress, int outputAddress, byte activate, byte data)
{
    TurnoutManager.DCCcommandHandler(outputAddress, data);
}

void HandleDCCAccPomPacket(int boardAddress,int outputAddress, byte instructionType, int cv, byte data)
{
    TurnoutManager.DCCPomHandler(cv, data);
}



// ========================================================================================================
// main setup
void setup()
{
    // initialize the turnout manager
    TurnoutManager.Initialize();

#ifdef _DEBUG
    Serial.begin(115200);

    // for timing tests
    pinMode(10,OUTPUT);
    pinMode(11,OUTPUT);
#endif
}


// values for 'timer' used below
unsigned long loopCount = 0;
unsigned long loopInterval = 250;   // yields ~1.5ms interval


// ========================================================================================================
// main loop
void loop()
{
    // DCC heartbeat process
    TurnoutManager.UpdateDccProcess();

    // hokey 'timer' for sensor/button/servo heartbeat functions
    // we don't use a timer interrupt so that the h/w int for DCC is as accurate as possible
    loopCount++;
    if (loopCount > loopInterval)
    {
        // for testing timing
        //PORTB |= (1 << 2);     // pulse output pin 10
        //PORTB &= ~(1 << 2);

        TurnoutManager.UpdateSensors();
        loopCount = 0;
    }
}
