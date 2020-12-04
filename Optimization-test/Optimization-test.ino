/*
 Name:		Optimization_test.ino
 Created:	11/7/2019 10:21:26 AM
 Author:	eric
*/

//#include "HardwareDebug.h"

//#include "SimpleQueue.h"
//#include "Bitstream.h"
//#include "DCCpacket.h"
#include "DCCdecoder.h"

//volatile static SimpleQueue simpleQueue;
//BitStream bitStream;
//DCCpacket dccpacket(true, false, 100);
DCCdecoder dcc;

// the setup function runs once when you press reset or power the board
void setup() {
	//bitStream.Resume();

	dcc.SetupDecoder(0, 0, 0, true);
	dcc.ResumeBitstream();
}

// the loop function runs over and over again until power down or reset
void loop() {

}
