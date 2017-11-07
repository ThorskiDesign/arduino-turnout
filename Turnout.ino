
#include <EEPROM.h>
#include <Servo.h>
#include "TurnoutMgr.h"
#include "XoverMgr.h"


XoverMgr TurnoutManager;


void setup()
{
#ifdef _DEBUG
    // for timing tests
    pinMode(0,OUTPUT);
	pinMode(1,OUTPUT);

	Serial.begin(115200);
	delay(1000);   // delay for Serial.print in factory reset (??)
#endif

    // initialize the turnout manager
    TurnoutManager.Initialize();
}


void loop()
{
    // this checks for new bitsteam data, and updates timers, LEDs, servo, and sensors
    TurnoutManager.Update();      // 30-50 us
}
