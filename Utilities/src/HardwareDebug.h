// HardwareDebug.h

#ifndef _HARDWAREDEBUG_h
#define _HARDWAREDEBUG_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


// make sure pinmode is set to output for pins 18 and 19 to use these
#define HW_DEBUG_PULSE_18() { PORTC = PORTC | (1 << 4); PORTC = PORTC & ~(1 << 4); }    // pulse pin 18
#define HW_DEBUG_PULSE_18_ON() PORTC = PORTC | (1 << 4)                                 // set pin 18 high
#define HW_DEBUG_PULSE_18_OFF() PORTC = PORTC & ~(1 << 4)                               // set pin 18 low
#define HW_DEBUG_PULSE_19() { PORTC = PORTC | (1 << 5); PORTC = PORTC & ~(1 << 5); }    // pulse pin 19
#define HW_DEBUG_PULSE_19_ON() PORTC = PORTC | (1 << 5)                                 // set pin 19 high
#define HW_DEBUG_PULSE_19_OFF() PORTC = PORTC & ~(1 << 5)                               // set pin 19 low

#define PWM_OUTPUT_PIN   6



// from https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
	char top;
#ifdef __arm__
	return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
	return &top - __brkval;
#else  // __arm__
	return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}


uint32_t hwDebugLastMillis = 0;
const uint32_t hwDebugUpdateInterval = 1000;    // ms


// display a value on the scope as a sequence of pulses
void hwDebugValueToPulses(uint32_t val, uint32_t minVal, uint32_t maxVal)
{
	uint32_t currentMillis = millis();
	if ((currentMillis - hwDebugLastMillis) < hwDebugUpdateInterval) return;
	hwDebugLastMillis = currentMillis;

	const byte maxPulses = 16;    // number of pulses to map value to
	byte n = map(val, minVal, maxVal, 0, maxPulses);

	for (byte i = 0; i < n; i++)
	{
		HW_DEBUG_PULSE_18();
	}
}


// display a value on the scope as a pwm signal
void hwDebugValueToPWM(uint32_t val, uint32_t minVal, uint32_t maxVal)
{
	uint32_t currentMillis = millis();
	if ((currentMillis - hwDebugLastMillis) < hwDebugUpdateInterval) return;
	hwDebugLastMillis = currentMillis;

	byte n = map(val, minVal, maxVal, 0, 255);
	analogWrite(PWM_OUTPUT_PIN, n);
}


// print to value to serial
void hwDebugValueToSerial(uint32_t val)
{
	uint32_t currentMillis = millis();
	if ((currentMillis - hwDebugLastMillis) < hwDebugUpdateInterval) return;
	hwDebugLastMillis = currentMillis;

	Serial.print("Debug value: "); Serial.println(val);
}

#endif
