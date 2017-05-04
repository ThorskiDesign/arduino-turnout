
#ifndef _TURNOUTSERVO_h
#define _TURNOUTSERVO_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Servo.h>

class TurnoutServo : public Servo
{
 public:
    typedef void (*ServoEventHandler)();

    TurnoutServo(byte ServoPin, byte PowerPin);
	void Initialize(byte ExtentLow, byte ExtentHigh, bool Position);
	void Initialize(byte ExtentLow, byte ExtentHigh, int DurationLow, int DurationHigh, bool Position);
	void Update(unsigned long CurrentMillis);
	void Update();
	bool IsMoving();
	bool IsActive();
	void Set(bool Position, bool Rate);
	void SetExtent(bool Position, byte Extent);
	void SetDuration(bool Position, int Duration);
	void SetServoMoveDoneHandler(ServoEventHandler Handler);
	void SetServoPowerOffHandler(ServoEventHandler Handler);

private:
	enum ServoState { OFF, STARTING, MOVING, STOPPING };

    void ComputeSteps();

	byte servoPin;                      // pin the servo pwm signal should be sent to
	byte powerPin;                      // pin used to enable/disable power to the servo

	const byte numSteps = 30;           // number of discrete increments of servo motion
	byte steps[2][30];	                // step sequence going each direction
	byte currentStep = 0;               // counter to track steps in update loop
	byte extent[2] = { 90, 90 };        // servo angle at LOW and HIGH positions
	int duration[2] = { 2500, 0 };      // duration (ms) of movement at low and high rates (0 = no delay)
    int interval[2] = { 0, 0 };			// time interval between each step

    bool positionSet = 0;               // the commanded position (high or low) for the servo
	bool rateSet = 0;                   // the commanded rate of the servo
	ServoState servoState = OFF;        // the current state of the servo
	unsigned long startTime = 0;        // time at which the servo power will be turned on
	unsigned long stopTime = 0;	        // time at which the servo power will be turned off
	const int servoStartDelay = 100;    // delay before powering on the servo after starting pwm signal
	const int servoStopDelay = 500;     // time the servo remains active after the duration to ensure motion is complete
	unsigned long lastUpdate = 0;       // time of the last servo write

	ServoEventHandler servoMoveDoneHandler = 0;     // pointer to handler for when servo motion is complete
	ServoEventHandler servoPowerOffHandler = 0;     // pointer to handler for when servo power is shut off
};

#endif
