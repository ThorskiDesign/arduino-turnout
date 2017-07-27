
/*

Turnout Servo

A class to manage a servo for a turnout.

Summary:

This class manages a servo used for driving the points of a turnout. It provides functionality 
to toggle the servo between each of two endpoints at either a high or low rate of speed. It handles 
control of the power circuit as well as the servo signal itself. 

Example usage:

		TurnoutServo servo(ServoPWMPin, ServoPowerPin);      // create an instance of the turnout servo using
		                                                     // specified pins for the pwm and power signals.
		servo.Initialize(ExtentLow, ExtentHigh, Position);   // initialize the servo endpoints and current
		                                                     // position.
		servo.Set(Position, Rate);               // set the servo to a position at the given rate.
		servo.Update();                          // check and update the servo state and position.

Details:

The TurnoutServo object is created with values specifying the pins used for the PWM signal as well 
as the signal controlling the servo power switch. The servo is further initialized with the high and 
low extents representing the desired limits of motion. Fast and slow rates of travel may optionally 
be specified.

When the Set method is called, the desired position and rate are set, the servo state is set to 
STARTING, the PWM signal is started, and the start and end times are calculated. The Update method 
checks the current millis() against the start and end times. When the start time is hit, the servo
power pin is turned on, and the state is set to MOVING. While MOVING, when an interval has elapsed,
the servo is commanded to the next step. After the final step, the move done handler is called, and
the state is set to STOPPING. After the stop time has elapsed, the servo power pin is turned off 
and the power off handler is called.

The movement steps of the servo are computed when the extents and/or duration are altered, to 
avoid repeatedly doing so when moving the servo. The positions corresponding to a given step of
the motion are based on the extents and the number of steps. The time interval between each step
is based on the high or low rate duration and the number of steps.

The power pin controls a high-side power switch, allowing the servo power to be disabled when it 
is not in use. Power is enabled shortly prior to beginning the servo motion, and is disabled after
the servo motion is completed. A callback is called when the servo power is turned off.

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
