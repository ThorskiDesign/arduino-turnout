
#include <EEPROM.h>
#include <Servo.h>
#include "TurnoutMgr.h"


TurnoutMgr TurnoutManager;


void setup()
{
#ifdef _DEBUG
    Serial.begin(115200);
	delay(1000);   // delay for Serial.print in factory reset (??)
#endif

    // initialize the turnout manager
    TurnoutManager.Initialize();

    // for timing tests
    pinMode(12,OUTPUT);
	pinMode(13,OUTPUT);
}


void loop()
{
    // this checks for new bitsteam data, and updates timers, LEDs, servo, and sensors
    TurnoutManager.Update();      // 30-50 us
}
