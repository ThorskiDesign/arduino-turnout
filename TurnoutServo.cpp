
#include "TurnoutServo.h"


// Create a new TurnoutServo
TurnoutServo::TurnoutServo(byte ServoPin)
{
    servoPin = ServoPin;
}


// Initialize the TurnoutServo (e.g., with values read from eeprom for extents and last position)
void TurnoutServo::Initialize(byte ExtentLow, byte ExtentHigh, bool Position)
{
	pinMode(servoPin, OUTPUT);
	digitalWrite(servoPin, LOW);

	extent[LOW] = ExtentLow;
    extent[HIGH] = ExtentHigh;
    positionSet = Position;

    // update steps and intervals
    ComputeSteps();
}


// Initialize the TurnoutServo (e.g., with values read from eeprom for extents, rates, and last position)
void TurnoutServo::Initialize(byte ExtentLow, byte ExtentHigh, int DurationLow, int DurationHigh, bool Position)
{
	pinMode(servoPin, OUTPUT);
	digitalWrite(servoPin, LOW);
	
	extent[LOW] = ExtentLow;
    extent[HIGH] = ExtentHigh;
    duration[LOW] = DurationLow;
    duration[HIGH] = DurationHigh;
    positionSet = Position;

    // update steps and intervals
    ComputeSteps();
}


// Update the servo position to allow slow slewing of servo
void TurnoutServo::Update(unsigned long CurrentMillis)
{
    if (servoState == MOVING)
    {
        // check if step interval for the given rate has elapsed
        if (CurrentMillis > lastUpdate)
        {
            lastUpdate = CurrentMillis + interval[rateSet];

            // if we still have steps to go in this movement
            if (currentStep < numSteps)
            {
                write(steps[positionSet][currentStep]);      // set servo to new position
                currentStep++;
            }
            else
            {
                currentStep = 0;           // reset counter for next movement
                servoState = READY;        // move is done, set back to ready state
                if (servoMoveDoneHandler) servoMoveDoneHandler();    // raise event indicating servo motion is complete
            }
        }
        return;
    }
}


// In case we want to run update without specifying millis
void TurnoutServo::Update() { Update(millis()); }


// Returns the servo moving status
bool TurnoutServo::IsMoving() { return (servoState == MOVING); }


// Returns the servo active status
bool TurnoutServo::IsActive() { return (servoState != OFF); }


// Sets the servo to a position at the specified rate
void TurnoutServo::Set(bool Position, bool Rate)
{
    // if position is new, start servo motion
    if (Position != positionSet)
    {
		MoveTo(Position, Rate);
    }
}


// Set an extent for the servo (e.g., while adjusting turnout endpoints)
void TurnoutServo::SetExtent(bool Position, byte Extent)
{ 
    // set the new value
    extent[Position] = Extent;

    // update steps and intervals
    ComputeSteps();

	// if we're setting the extent for the current position, adjust the servo position
	if (Position == positionSet)
	{
#ifdef _DEBUG
		Serial.print("Setting new extent for position ");
		Serial.println(positionSet, DEC);
#endif // _DEBUG

		MoveTo(Position, HIGH);
	}
}


// configure and start up servo motion
void TurnoutServo::MoveTo(bool Position, bool Rate)
{
	if (servoState != READY) return;   // only go to the moving state from the ready state

	// update position and rate settings
	positionSet = Position;
	rateSet = Rate;
	servoState = MOVING;
}


void TurnoutServo::StartPWM()
{
	if (servoState != OFF) return;   // only go to the ready state from the off state or at the end of a move

	// ensure we are sending pulses for the current position
	write(extent[positionSet]);
	attach(servoPin);
	servoState = READY;
}


void TurnoutServo::StopPWM()
{
	if (servoState != READY) return;   // only go to the off state from the ready state

	detach();                            // stop sending pwm pulses
	digitalWrite(servoPin, LOW);         // force servo pin low
	servoState = OFF;
}


// Set the duration for slow/fast rate
void TurnoutServo::SetDuration(bool Position, int Duration)
{
    duration[Position] = Duration;

    // update intervals
    ComputeSteps();
}


// compute the steps going in each direction for easy lookup in the update loop
// set the timing interval for low and high rates (note may have small error in total elapsed time due to truncation)
void TurnoutServo::ComputeSteps()
{
    for (int i=0; i<numSteps; i++)
    {
        int movementRange = extent[1] - extent[0];              // total range of movement
        int increment = ((float)(i+1)/numSteps)*movementRange;  // increment for each step (force floating point division)
        steps[0][i] = extent[1]-increment;                      // steps going from high to low
        steps[1][i] = extent[0]+increment;                      // steps going from low to high

#ifdef _DEBUG
        Serial.print("low = ");
        Serial.print(steps[0][i],DEC);
        Serial.print("   high = ");
        Serial.println(steps[1][i],DEC);
#endif
    }

    interval[0]=duration[0]/numSteps;     // low rate interval
    interval[1]=duration[1]/numSteps;     // high rate interval
}


// Assign the callback function for when servo motion is done
void TurnoutServo::SetServoMoveDoneHandler(ServoEventHandler Handler) { servoMoveDoneHandler = Handler; }
