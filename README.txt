
Arduino Turnout

A set of libraries for managing the operation of a DCC controlled, servo actuated model railroad 
turnout.


DCC decoding approach:

The DCC decoding functionality is broken into three classes, each with a specific function. The 
BitStream class handles the capture of the raw DCC bitstream. It performs sync and low level error
checking, and provides the captured bitstream via callback. The DCCpacket class takes the raw
bitstream and assembles it into valid DCC packets. It optionally enforces the DCC checksum, and 
can filter repeated DCC packets so that upstream classes can treat the packet delivery as 
reliable. The DCCdecoder class takes a complete DCC packet and processes it, determining the
packet type, parsing out the DCC address, and extracting the packet data. Callbacks for the 
various packet types provide the data to upstream classes.

The resoning behind this approach was to provide the most reliable bitstream capture possible,
while easing the constraints on processing time for the packet building, decoding, and other
functions. The raw bitstream capture is the only operation that has to be real-time. It happens 
in an ISR, and is completed within 7-10 microseconds of each interrupt. The packet building and 
decoding are not nearly as time sensitive, and in fact may be interrupted by the bitstream ISR 
if they run long, without the risk of missing a bit or degrading the packet processing. 

This approach also facilitates maintenace and reusability. The bitstream class can be modified
without affecting the other classes, and is the only class that requires the actual arduino hardware
to unit test. The DCCpacket and DCCdecoder classes can be unit tested in any C environmnet, 
simplifying the use of test cases to verify performance.