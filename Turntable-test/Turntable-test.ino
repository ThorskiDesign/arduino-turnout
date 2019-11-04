/*
 Name:		Turntable_test.ino
 Created:	10/30/2019 10:44:36 AM
 Author:	eric
*/


#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>


//#define HW_DEBUG_PULSE() { PORTD = PORTD | (1 << 0); PORTD = PORTD & ~(1 << 0); }    // pulse pin 0
//#define HW_DEBUG_PULSE_ON() PORTD = PORTD | (1 << 0)                                 // set pin 0 high
//#define HW_DEBUG_PULSE_OFF() PORTD = PORTD & ~(1 << 0)                               // set pin 0 low
#define HW_DEBUG_PULSE() { PORTC = PORTC | (1 << 3); PORTC = PORTC & ~(1 << 4); }    // pulse pin 17
#define HW_DEBUG_PULSE_ON() PORTC = PORTC | (1 << 3)                                 // set pin 17 high
#define HW_DEBUG_PULSE_OFF() PORTC = PORTC & ~(1 << 3)                               // set pin 17 low


// global stuff

// Create the motor shield and stepper objects
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor* afStepper = AFMS.getStepper(200, 2);
AccelStepper accelStepper(forwardstep1, backwardstep1);

// hall sensors
const byte hallSensorCWpin = 2;
const byte hallSensorCCWpin = 3;

const uint16_t stepsPerDegree = 160;  // for 200 steps, 16 microsteps, and 18:1 gearing, a 1 degree turntable movement is 160 steps
int32_t commandedPosition = 0;

// error check vars
int32_t prevCWsteps = 0;
int32_t prevCCWsteps = 0;
volatile int32_t curCWsteps = 0;
volatile int32_t curCCWsteps = 0;
uint16_t errorCount = 0;


// step function callbacks
void forwardstep1()
{
	HW_DEBUG_PULSE_ON();
	afStepper->onestep(FORWARD, MICROSTEP);
	HW_DEBUG_PULSE_OFF();
}

void backwardstep1()
{
	HW_DEBUG_PULSE_ON();
	afStepper->onestep(BACKWARD, MICROSTEP);
	HW_DEBUG_PULSE_OFF();
}


// hall sensor interrupt callbacks
void hallSensorClockwiseISR()
{
	curCWsteps = accelStepper.currentPosition();
}

void hallSensorCounterclockwiseISR()
{
	curCCWsteps = accelStepper.currentPosition();
}


// the setup function runs once when you press reset or power the board
void setup()
{
	// for testing timing, make sure this matches the defines above
	pinMode(17, OUTPUT);

	Serial.begin(9600);           // set up Serial library at 9600 bps
	Serial.println("Turntable test!");

	// hall sensor setup
	pinMode(hallSensorCWpin, INPUT_PULLUP);
	pinMode(hallSensorCCWpin, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(hallSensorCWpin), hallSensorClockwiseISR, FALLING);
	attachInterrupt(digitalPinToInterrupt(hallSensorCCWpin), hallSensorCounterclockwiseISR, FALLING);

	// adafruit motorshield setup
	AFMS.begin();                            // create with the default PWM frequency 1.6KHz
	TWBR = ((F_CPU / 400000l) - 16) / 2;     // change I2C clock to 400kHz

	// accelstepper setup
	commandedPosition = 15 * stepsPerDegree;     // first movement is in the positive direction
	accelStepper.setMaxSpeed(500);
	accelStepper.setAcceleration(50);
	accelStepper.moveTo(commandedPosition);         // set up the initial movement
}

// the loop function runs over and over again until power down or reset
void loop()
{
	// If at the end of travel...
	if (accelStepper.distanceToGo() == 0)
	{
		// check hall sensor measurements for any differences
		if (commandedPosition > 0)   // if we were moving in positive (CW) direction
		{
			if ((curCWsteps != prevCWsteps) && (prevCWsteps != 0)) errorCount++;
			prevCWsteps = curCWsteps;
			Serial.print("CW Position: "); Serial.print(curCWsteps);
		}

		if (commandedPosition < 0)   // if we were moving in negative (CCW) direction
		{
			if ((curCCWsteps != prevCCWsteps) && (prevCCWsteps != 0)) errorCount++;
			prevCCWsteps = curCCWsteps;
			//Serial.print("CCW Position: "); Serial.print(curCCWsteps);
		}

		// display the current error count
		Serial.print("        Error count: "); Serial.println(errorCount);

		// now update commanded position and set stepper to move to the opposite end of travel
		commandedPosition = -commandedPosition;
		accelStepper.moveTo(commandedPosition);
	}

	// update the stepper as needed
	accelStepper.run();
}
