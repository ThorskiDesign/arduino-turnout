
/*

Turnout Servo

A class to manage a servo for a turnout.

Summary:

This class manages a servo used for driving the points of a turnout. It provides functionality 
to toggle the servo between each of two endpoints at either a high or low rate of speed.

Example usage:

		TurnoutServo servo(ServoPWMPin);      // create an instance of the turnout servo
		servo.Initialize(ExtentLow, ExtentHigh, Position);   // initialize the servo endpoints and current
		                                                     // position.
		servo.Set(Position, Rate);               // set the servo to a position at the given rate.
		servo.Update();                          // check and update the servo state and position.

Details:

The TurnoutServo object is created with a value specifying the pin used for the PWM signal. The servo 
is further initialized with the high and low extents representing the desired limits of motion. Fast 
and slow rates of travel may optionally be specified.

Servo control operates in three states - OFF, READY, and MOVING. In the OFF state, the PWM signal on the
servo pin is disabled and the pin output is set low. In the READY state, the PWM signal is active and the 
servo is ready to receive a move command. In the moving state, the servo is actively moving from one 
endpoint to the other. The servo is initially in the OFF state. The StartPWM method attaches the servo 
and sets the state to READY. When the MoveTo method is called, either by the Set or SetExtents methods, 
the desired position and rate are set, and the servo state is set to MOVING. After the motion is complete,
the state reverts to READY. The StopPWM method is used to disable the PWM signal and set the state to OFF.

The Update method performs the actual motion of the servo. While MOVING, when an interval has elapsed,
the servo is commanded to the next step. After the final step, the move done handler is called, and
the state is set back to READY.

The movement steps of the servo are computed when the extents and/or duration are altered, to 
avoid repeatedly doing so when moving the servo. The positions corresponding to a given step of
the motion are based on the extents and the number of steps. The time interval between each step
is based on the high or low rate duration and the number of steps.

*/


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

    TurnoutServo(byte ServoPin);
	void Initialize(byte ExtentLow, byte ExtentHigh, bool Position);
	void Initialize(byte ExtentLow, byte ExtentHigh, int DurationLow, int DurationHigh, bool Position);
	void Update(unsigned long CurrentMillis);
	void Update();
	bool IsMoving();
	bool IsActive();
	void Set(bool Position, bool Rate);
	void SetExtent(bool Position, byte Extent);
	void StartPWM();
	void StopPWM();
	void SetDuration(bool Position, int Duration);
	void SetServoMoveDoneHandler(ServoEventHandler Handler);

private:
	enum ServoState { 
		OFF,        // pwm is off and servo is detached
		READY,      // pwm is on, servo is ready to move
		MOVING };   // servo motion is in progress

    void ComputeSteps();
	void MoveTo(bool Position, bool Rate);

	byte servoPin;                      // pin the servo pwm signal should be sent to
	const byte numSteps = 30;           // number of discrete increments of servo motion
	byte steps[2][30];	                // step sequence going each direction
	byte currentStep = 0;               // counter to track steps in update loop
	byte extent[2] = { 90, 90 };        // servo angle at LOW and HIGH positions
	int duration[2] = { 2500, 0 };      // duration (ms) of movement at low and high rates (0 = no delay)
    int interval[2] = { 0, 0 };			// time interval between each step

    bool positionSet = 0;               // the commanded position for the servo
	bool rateSet = 0;                   // the commanded rate of the servo
	ServoState servoState = OFF;        // the current state of the servo
	unsigned long lastUpdate = 0;       // time of the last servo write

	ServoEventHandler servoMoveDoneHandler = 0;     // pointer to handler for when servo motion is complete
};

#endif
