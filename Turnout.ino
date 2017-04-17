
#include <EEPROM.h>
#include <Servo.h>
#include "TurnoutMgr.h"


TurnoutMgr TurnoutManager;


void setup()
{
    // initialize the turnout manager
    TurnoutManager.Initialize();

    // for timing tests
    pinMode(12,OUTPUT);

#ifdef _DEBUG
    Serial.begin(115200);
#endif
}


void loop()
{
    // this checks for new bitsteam data, and updates timers, LEDs, servo, and sensors
    TurnoutManager.Update();      // 30-50 us
}
