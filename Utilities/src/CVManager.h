// CVManager.h

#ifndef _CVMANAGER_h
#define _CVMANAGER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class CVManager
{
public:
	explicit CVManager(byte numCVs);
	~CVManager();

	void resetCVs();
	int16_t getCVindex(byte cvNum);

	byte initCV(byte index, byte cvNum, byte CVDefault, byte rangeMin = 0, byte rangeMax = 255, bool softReset = true);
	byte initCV16(byte index, byte cvNum, uint16_t CVDefault, uint16_t rangeMin = 0, uint16_t rangeMax = 65535, bool softReset = true);
	uint16_t getCV(byte cvNum);
	bool setCV(byte cvNum, uint16_t value);
	
	struct CVstatic
	{
		byte cvNum = 0;             // cv number
		byte cvDefault = 0;         // default value for the cv
		byte rangeMin = 0;          // valid range of CV value
		byte rangeMax = 255;
		bool softReset = true;      // should this cv get reset during a soft reset
		bool is16bit = false;       // is this a 16 bit cv, so we aggregate it with the next index
	};

	struct CV
	{
		byte cvNum = 0;
		byte cvValue = 0;
	};

	byte numCVs;
	CVstatic* cvStatic;
	CV* cv;

private:

};

#endif
