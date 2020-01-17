
#include "CVManager.h"

CVManager::CVManager(byte num) : numCVs(num), cvStatic(new CVstatic[num]), cv(new CV[num])
{
}

CVManager::~CVManager()
{
	delete[] cvStatic;
	delete[] cv;
}

void CVManager::resetCVs()
{
	for (byte i = 0; i < numCVs; i++)
		cv[i].cvValue = cvStatic[i].cvDefault;
}

int16_t CVManager::getCVindex(byte cvNum)
{
	byte i = 0;
	while ((i < numCVs) && (cvStatic[i].cvNum != cvNum)) i++;    // find the index of the cv in the array

	// check for invalid cv provided
	if (i == numCVs) return -1;

	// otherwise return the cv index
	return i;
}

byte CVManager::initCV(byte index, byte cvNum, byte CVDefault, byte rangeMin, byte rangeMax, bool softReset)
{
	if (index >= numCVs) return 0;

	cvStatic[index].cvNum = cvNum;
	cvStatic[index].cvDefault = CVDefault;
	cvStatic[index].rangeMin = rangeMin;
	cvStatic[index].rangeMax = rangeMax;
	cvStatic[index].softReset = softReset;
	cvStatic[index].is16bit = false;

	// return next available index
	return (index < 255) ? index + 1 : 0;
}

byte CVManager::initCV16(byte index, byte cvNum, uint16_t CVDefault, uint16_t rangeMin, uint16_t rangeMax, bool softReset)
{
	if (index >= numCVs) return 0;

	cvStatic[index].cvNum = cvNum;
	cvStatic[index].cvDefault = highByte(CVDefault);
	cvStatic[index + 1].cvDefault = lowByte(CVDefault);
	cvStatic[index].rangeMin = highByte(rangeMin);
	cvStatic[index + 1].rangeMin = lowByte(rangeMin);
	cvStatic[index].rangeMax = highByte(rangeMax);
	cvStatic[index + 1].rangeMax = lowByte(rangeMax);
	cvStatic[index].softReset = softReset;
	cvStatic[index].is16bit = true;

	// return next available index
	return (index < 255) ? index + 2 : 0;
}

uint16_t CVManager::getCV(byte cvNum)
{
	const int16_t cvIndex = getCVindex(cvNum);
	if (cvIndex == -1) return 0;    // requested cv was not found in our collection

	if (cvStatic[cvIndex].is16bit)
		return (cv[cvIndex].cvValue << 8) + cv[cvIndex + 1].cvValue;
	else
		return cv[cvIndex].cvValue;
}

bool CVManager::setCV(byte cvNum, uint16_t value)
{
	const int16_t cvIndex = getCVindex(cvNum);
	if (cvIndex == -1) return false;    // requested cv was not found in our collection

	if (cvStatic[cvIndex].is16bit)
	{
		// check for valid range of data provided
		const uint16_t min = (cvStatic[cvIndex].rangeMin << 8) + cvStatic[cvIndex + 1].rangeMin;
		const uint16_t max = (cvStatic[cvIndex].rangeMax << 8) + cvStatic[cvIndex + 1].rangeMax;
		if (value < min || value > max) return false;

		// value supplied is ok, so store it
		cv[cvIndex].cvValue = highByte(value);
		cv[cvIndex + 1].cvValue = lowByte(value);
	}
	else
	{
		// check for valid range of data provided
		if (value < cvStatic[cvIndex].rangeMin || value > cvStatic[cvIndex].rangeMax) return false;

		// value supplied is ok, so store it
		cv[cvIndex].cvValue = value;
	}

	return true;
}

