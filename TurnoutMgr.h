
/*

Arduino Turnout Manager

A class for managing the top level operation of a DCC controlled, servo actuated model railroad
turnout.

Summary:

The TurnoutMgr class provides the top level functionality for the turnout. It handles the processing
of received DCC commands, controls the servo that drives the turnout, controls the LED indications,
controls optional relays for powering points or leads, monitors the pushbutton, and monitors the
occupancy sensors. It contains the hardware pin assignments for all the I/O, as well as the
definition and defaults for the CVs. A reset to default may be performed by holding the pushbutton
while the hardware is powered up. Options to swap the interpretation of the DCC command, the servo
endpoints, the occupancy sensors, and the relays are provided. Two auxiliary outputs are controllable
using extended accessory (signal aspect) commands.

Example Usage:

	TurnoutMgr TurnoutManager;          // create an instance of the turnout manager
	TurnoutManager.Initialize();        // initialize the turnout manager. call this in setup().
	TurnoutManager.Update();            // check for DCC commands, update sensors and actuators.
	                                       call this in loop().

Details:

DCC command processing takes place as follows. The raw bitstream is captured by the BitStream
object. When 32 bits have been captured, a callback to the packet decoder begins the assembly
of the DCC packet. After the packet decoder has assembled and checksummed a complete packet, a 
callback to the DCCdecoder initiates processing of the packet. Callbacks from the DCCdecoder 
to the TurnoutMgr trigger actions for normal accessory decoder packets, extended accessory 
decoder packets, and programming on main packets.

The constructor initializer list configures the objects for managing the button, LED, servo, and
various sensors and actuators. It also configures the BitStream and DCCpacket objects. The DCCpacket 
object is configured with checksum and filtering of repeat packets enabled.

The InitMain method performs the major setup for the class, including setting up the DCC packet
processor, reading the stored configuration from EEPROM (via the DCCdecoder lib), getting the stored
position of the turnout and configuring the servo, and setting up all the event handlers. It sets
the LED and relays, and starts the bitstream capture to listen for DCC commands. If a factory reset
is triggered in the Initialize method, the CVs are restored to their default settings, and a timer
is set which then runs the InitMain method. The Initialize method should be called once from the
main arduino setup() function.

The Update method should be called in the main arduino loop() method. This method processes timestamps
received by the BitStream object, which then sends them to the DCCpacket object to be assembled
into a full DCC packet. It also handles millis-related updates for the LED, sensors, timers and
servo. Packet error count per second is also checked. If it exceeds a configurable max value, a
reset of the bitstream object takes place.

The SetServo and SetRelays methods handle setting a new servo position, and then, after a callback
indicating the servo motion is complete, setting the relays and LEDs according to the new
position. The ButtonEventHandler, OSStraightHandler, and OSCurvedHandler respond to events from
the button and occupancy sensors, and trigger a change in the turnout position.

The DCCAccCommandHandler processes a basic accessory command, used to set the position of the
turnout. Occupancy sensors are checked prior to setting the turnout, with an error indication given
if they are occupied. The DCCExtCommandHandler processes an extended accessory command, using signal
aspects for turning the two auxilliary outputs on and off. The DCCPomHandler method processes a
program on main packet. It checks for a valid CV, stores the data via the DCCdecoder object, and
then re-reads the basic configuration for the turnout.

Event handler wrappers for the sensors, button, servo, timer, and DCC classes are static, so that 
they are accessible as callbacks from those classes. An instance variable provides access to the
instance of the turnout manager, where the actual callback handling takes place.

*/

#ifndef _TURNOUTMGR_h
#define _TURNOUTMGR_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "TurnoutBase.h"


class TurnoutMgr : protected TurnoutBase
{
public:
	TurnoutMgr();
	void Initialize();
	void Update();

private:
    // main functions
	void InitMain();
	void SetServo(bool ServoRate);
	void SetRelays();

	// Sensors and outputs
	TurnoutServo servo;
	Button osStraight;
	Button osCurved;
	OutputPin relayStraight;
	OutputPin relayCurved;

	// event handlers
	void ServoStartupHandler();
	void ServoMoveDoneHandler();
	void ServoPowerOffHandler();
	void ButtonEventHandler(bool ButtonState);
	void OSStraightHandler(bool ButtonState);
	void OSCurvedHandler(bool ButtonState);
	void DCCAccCommandHandler(unsigned int Addr, unsigned int Direction);
	void DCCPomHandler(unsigned int Addr, byte instType, unsigned int CV, byte Value);

	// pointer to allow us to access member objects from callbacks
	static TurnoutMgr *currentInstance;

	// Turnout manager event handler wrappers
	static void WrapperButtonPress(bool ButtonState);
	static void WrapperOSStraight(bool ButtonState);
	static void WrapperOSCurved(bool ButtonState);
	static void WrapperServoStartup();
    static void WrapperServoMoveDone();
    static void WrapperServoPowerOff();

	// DCC event handler wrappers
	static void WrapperDCCAccPacket(int boardAddress, int outputAddress, byte activate, byte data);
	static void WrapperDCCExtPacket(int boardAddress, int outputAddress, byte data);
	static void WrapperDCCAccPomPacket(int boardAddress, int outputAddress, byte instructionType, int cv, byte data);
	static void WrapperDCCDecodingError(byte errorCode);

	// wrappers for callbacks in TurnoutBase ================================================================

	// callbacks for bitstream and packet builder
	static void WrapperBitStream(unsigned long incomingBits);
	static void WrapperBitStreamError(byte errorCode);
	static void WrapperDCCPacket(byte *packetData, byte size);
	static void WrapperDCCPacketError(byte errorCode);

	// Turnout manager event handler wrappers
	static void WrapperResetTimer();
	static void WrapperErrorTimer();
};

#endif