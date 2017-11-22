
Arduino Turnout

A set of libraries for managing the operation of a DCC controlled, servo actuated model railroad 
turnout.

Overview

The software design consists of three main components – the DCC decoding, the hardware 
input/output, and the overall management of the turnout. The following sections provide an 
overview of each of the classes and describe their interaction. Detailed descriptions and 
example usages are provided in the header comments for each class.

DCC Decoding

The DCC decoding functionality is broken into three classes, each with a specific function. The 
BitStream class handles the capture of the raw DCC bitstream. It captures the raw pulse 
transitions in a queue, performs low level error checking to identify valid bits, and assembles 
and provides the captured bitstream via a callback. The Arduino input capture register is used 
in order to get the most accurate timing of the pulses, and to eliminate the influence of other 
ISRs that may be running. Bitstream capture using a hardware interrupt is also supported.

The DCCpacket class takes the raw bitstream and assembles it into valid DCC packets. It 
optionally enforces the DCC checksum, and can filter repeated DCC packets so that upstream 
classes can treat the packet delivery as reliable. Completed packets are provided to the 
DCCdecoder class via callback.

The DCCdecoder class takes a complete DCC packet and processes it, determining the packet type, 
parsing out the DCC address, and extracting the packet data. Callbacks for the various packet 
types provide the data to upstream classes.

The only part of the DCC decoding process that happens in an ISR is adding the pulse timer 
count to the queue – all other bitstream processing and packet decoding takes place as a normal 
process outside the ISR, significantly easing the constraints on processing time for the packet 
building, decoding, and other functions. The packet building and decoding may in fact be 
interrupted by the bitstream ISR if they run long, without the risk of missing a bit or 
degrading the packet processing.

The bitstream class is the only class that requires an actual DCC signal and an Arduino to unit 
test. The DCCpacket and DCCdecoder classes can be unit tested in any C environment, simplifying 
the use of test cases to verify performance.

Hardware Input/Output

Apart from the DCC input discussed above, the primary hardware I/O required for operation of 
the decoder includes a button input, an RGB LED output, one or more servo outputs, and 
occupancy sensor inputs. Relay outputs are provided for powering or grounding rails on the 
turnout where needed. In addition, two external auxiliary outputs are provided for low current 
loads such as LED lighting.

The Button class provides a basic debounced button input. It is used for the button on the 
enclosure, as well as for the occupancy sensor inputs, which may be either optical or contact 
closure inputs.

The OutputPin class is a simple wrapper around the digitalWrite functions for an output pin. It 
is used for the relays and two auxiliary outputs.

The RGB LED class facilitates control of the three output pins for the LED, as well as 
providing ON, OFF, and FLASH functionality.

The TurnoutServo class manages the control of the servo that drives the points of the turnout. 
It allows setting of the servo endpoints as well as high and low rate speeds. The servo motion 
is controlled based on the endpoints and the desired speeds. A callback provides notification 
of completed motion. Multiple servos may be controlled by the turnout manager for use on a 
crossover/turnout assembly.

Turnout Management

The overall management of the turnout is handled by the TurnoutMgr class. It provides the top 
level logic for acting on received DCC commands, responding to the occupancy sensors and button 
inputs, and controlling the servo, relays, and LED. It maintains the configuration of the 
turnout via CVs stored to EEPROM, allows changes via DCC program on main commands, and provides 
a reset-to-default option. It handles updating any objects that require time-based updates 
(servo, button, LED), and provides error handling and indication in the case of repeated DCC 
bitstream or packet errors. Two auxiliary outputs as well as temporary configuration settings 
are controllable using extended accessory (signal aspect) commands.

The servo power pin is turned on and off as needed, so that the servo is only powered when it 
is actually in use. The PWM signal is started before the power is enabled and stopped after the 
power is disabled, so that the servo always has a valid signal while it is powered. In the case 
of the crossover manager, the servo motions happen sequentially, followed by turning off the 
servo power pin.

A derived class provides management for a crossover, controlling four servos and four relays.
