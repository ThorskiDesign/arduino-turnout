
#include "TurnoutMgr.h"


// ========================================================================================================
// Public Methods


// TurnoutMgr constructor
TurnoutMgr::TurnoutMgr():
    button(ButtonPin, true),
    osStraight(OSstraightPin, true),
    osCurved(OScurvedPin, true),
    led(LedPinR, LedPinG, LedPinB),
    servo(ServoPWMPin, ServoPowerPin),
    relayStraight(RelayStraightPin),
    relayCurved(RelayCurvedPin),
    bitStream(DCCPin, false, 48, 68, 88, 10000, 10),    // bitstream capture object, non-standard timings
    dccPacket(true, true, 250),                         // DCC packet builder
    dcc()                                               // DCC packet processor
{
}


// Check for factory reset, then proceed with main initialization
void TurnoutMgr::Initialize()
{
    // check for button hold on startup (for reset to defaults)
    if (button.RawState() == LOW) 
    {
        FactoryReset();
    }
    else
    {
        InitMain();
    }
}


// check for new bitstream data, update sensors and outputs, check for
// max number of packet errors and reset bitstream capture if necessary
void TurnoutMgr::Update()
{
    // if we have new data, go process it
    if (haveNewBits)
    {
        dccPacket.ProcessIncomingBits(bits);     // 120-160 us
        haveNewBits = false;
    }

    // do the updates to maintain flashing led and slow servo motion
    unsigned long currentMillis = millis();
    servo.Update(currentMillis);
    led.Update(currentMillis);

    // timer updates
    errorTimer.Update(currentMillis);
    resetTimer.Update(currentMillis);

    // update sensors
    button.Update(currentMillis);
    osStraight.Update(currentMillis);
    osCurved.Update(currentMillis);

    // check/reset error counts
    if (currentMillis - lastMillis > 1000)
    {
#ifdef _DEBUG
        Serial.print("Bit Error Count:  ");
        Serial.println(bitErrorCount, DEC);
        Serial.print("Packet Error Count:  ");
        Serial.println(packetErrorCount, DEC);
#endif
        // if we see repeated packet errors, reset bitream capture
        if (packetErrorCount > maxPacketErrors)
        {
            bitStream.Suspend();
            bitStream.Resume();
        }

        lastMillis = currentMillis;
        bitErrorCount = 0;
        packetErrorCount = 0;
    }
}



// ========================================================================================================
// Private Methods


// Initialize the turnout manager by setting up the dcc config, reading stored values from CVs, and setting up the servo
void TurnoutMgr::InitMain()
{
    // Configure and initialize the DCC packet processor (accessory decoder in output address mode)
    byte cv29 = DCCdecoder::CV29_ACCESSORY_DECODER | DCCdecoder::CV29_OUTPUT_ADDRESS_MODE;
    dcc.SetupDecoder(0, 0, cv29, false);

    // get variables from cv's
    dccAddress = dcc.Address();
    servoEndPointSwap = dcc.GetCV(CV_servoEndPointSwap);
    occupancySensorSwap = dcc.GetCV(CV_occupancySensorSwap);
    dccCommandSwap = dcc.GetCV(CV_dccCommandSwap);
    relaySwap = dcc.GetCV(CV_relaySwap);

    // set the current position based on the stored position
    position = (dcc.GetCV(CV_turnoutPosition) == 0) ? STRAIGHT : CURVED;

    // servo setup - get extents, rates, and last position from cv's
    State servoState = position;
    if (servoEndPointSwap) 
        servoState = (servoState == STRAIGHT) ? CURVED : STRAIGHT;  // swap the servo endpoints if needed
    servo.Initialize(
        dcc.GetCV(CV_servoMinTravel),
        dcc.GetCV(CV_servoMaxTravel),
        dcc.GetCV(CV_servoLowSpeed) * 100,
        dcc.GetCV(CV_servoHighSpeed) * 100,
        servoState);

    // set pointer to this instance of the turnout manager, so that we can reference it in callbacks
    currentInstance = this;

    // set callbacks for the bitstream capture
    bitStream.SetDataFullHandler(WrapperBitStream);
    bitStream.SetErrorHandler(WrapperBitStreamError);

    // set callbacks for the packet builder
    dccPacket.SetPacketCompleteHandler(WrapperDCCPacket);
    dccPacket.SetPacketErrorHandler(WrapperDCCPacketError);

    // configure dcc event handlers
    dcc.SetBasicAccessoryDecoderPacketHandler(WrapperDCCAccPacket);
    dcc.SetBasicAccessoryPomPacketHandler(WrapperDCCAccPomPacket);
    dcc.SetExtendedAccessoryDecoderPacketHandler(WrapperDCCExtAccPacket);
    dcc.SetDecodingErrorHandler(WrapperDCCDecodingError);

    // configure other event handlers
    button.SetButtonPressHandler(WrapperButtonPress);
    servo.SetServoMoveDoneHandler(WrapperServoMoveDon);
    servo.SetServoPowerOffHandler(WrapperServoPowerOff);
    osStraight.SetButtonPressHandler(WrapperOSStraight);
    osCurved.SetButtonPressHandler(WrapperOSCurved);
    errorTimer.SetTimerHandler(WrapperErrorTimer);

    // initlize led and relays
    SetRelays();

    // finally, kick off the bitstream capture
    bitStream.Resume();

#ifdef _DEBUG
    Serial.print("DCC init done, using dcc address ");
    Serial.println(dccAddress, DEC);
    Serial.println("Servo init done.");
    Serial.print("Servo position read from CVs is ");
    Serial.println(position, DEC);
#endif
}


// perform a reset to factory defaults
void TurnoutMgr::FactoryReset()
{
#ifdef _DEBUG
    Serial.println("Reset to defaults initiated.");
#endif

    factoryReset = true;    // set flag indicating we are in reset
    unsigned long resetDelay = 2500;  // time to flash led so we have indication of reset occuring

    // normal initilization will resume after this timer expires
    resetTimer.StartTimer(resetDelay);
    resetTimer.SetTimerHandler(&WrapperResetTimer);

    led.SetLED(RgbLed::MAGENTA, RgbLed::FLASH);

    // do the reset
    unsigned int numCVs = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
    for (unsigned int cv = 0; cv < numCVs; cv++)
    {
        dcc.SetCV( FactoryDefaultCVs[cv].CV, FactoryDefaultCVs[cv].Value);
    }

#ifdef _DEBUG
    Serial.println("Reset to defaults completed.");
#endif
}


// set the led and relays for the current position
void TurnoutMgr::SetRelays()
{
    // set the led solid for the current position
    led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::ON);

    // enable the appropriate relay, swapping if needed
    if ((position == STRAIGHT && !relaySwap) || (position == CURVED && relaySwap)) 
    {
        relayStraight.SetRelay(HIGH);
    }

    if ((position == CURVED && !relaySwap) || ((position == STRAIGHT) && relaySwap))
    {
        relayCurved.SetRelay(HIGH);
    }
}


// set the turnout to a new position (also disables relays prior to starting servo motion)
void TurnoutMgr::SetServo(bool ServoRate)
{
    // suspend the bitstream capture, since the servo/led related interrupts cause timing errors in the bitstream.
    // bitstream capture is resumed in the ServoMoveDoneHandler method.
    bitStream.Suspend();

    // turn off the relays
    relayStraight.SetRelay(LOW);
    relayCurved.SetRelay(LOW);

    // set the servo
    State servoState = position;
    if (servoEndPointSwap) 
        servoState = (servoState == STRAIGHT) ? CURVED : STRAIGHT;  // swap the servo endpoints if needed
    servo.Set(servoState, ServoRate);

#ifdef _DEBUG
    Serial.print("Setting servo to ");
    Serial.print(servoState, DEC);
    Serial.print(" at rate ");
    Serial.println(ServoRate, DEC);
#endif

    // store new position to cv
    dcc.SetCV(CV_turnoutPosition,position);
}



// ========================================================================================================
// Event Handlers


// handle the reset timer callback
void TurnoutMgr::ResetTimerHandler()
{
    // run the main init after the reset timer expires
    factoryReset = false;
    InitMain();
}


// handle the error timer callback
void TurnoutMgr::ErrorTimerHandler()
{
    // all we need to do here is turn the led back on normally
    led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::ON);
}


// do things after the servo is powered off
void TurnoutMgr::ServoPowerOffHandler() { }


// do things after the servo finishes moving to its new position
void TurnoutMgr::ServoMoveDoneHandler() 
{ 
    SetRelays();          // set the relays for the new position
    bitStream.Resume();   // resume the bitstream capture stopped prior to beginning servo motion
}


// handle a button press
void TurnoutMgr::ButtonEventHandler(bool ButtonState)
{
    // check button state (HIGH so we respond after button release)
    if (ButtonState == HIGH)
    {
        // proceed only if both occupancy sensors are inactive (i.e., sensors override button press)
        if (osStraight.SwitchState() == HIGH && osCurved.SwitchState() == HIGH)
        {
            // toggle from current position and set new position
            position = (State) !position;
            led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::FLASH);
            SetServo(LOW);
        }
        else
        {
            // button error indication
            led.SetLED(RgbLed::BLUE, RgbLed::FLASH);

            // normal led will resume after this timer expires
            errorTimer.StartTimer(2500);
        }
    }
}


// handle straight occupancy sensor signal
void TurnoutMgr::OSStraightHandler(bool ButtonState)
{
    State newPos = (occupancySensorSwap) ? CURVED : STRAIGHT;

    // check occupancy sensor state (LOW so we respond when train detected)
    if (ButtonState == LOW && newPos != position)
    {
        position = newPos;
        led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::FLASH);
        SetServo(HIGH);
    }
}


// handle curved occupancy sensor signal
void TurnoutMgr::OSCurvedHandler(bool ButtonState)
{
    State newPos = (occupancySensorSwap) ? STRAIGHT : CURVED;

    // check occupancy sensor state (LOW so we respond when train detected)
    if (ButtonState == LOW && newPos != position)
    {
        position = newPos;
        led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::FLASH);
        SetServo(HIGH);
    }
}


// handle the dcc interpreter callback
void TurnoutMgr::DCCcommandHandler(unsigned int Addr, unsigned int Direction)
{
    // assume we are filtering repeated packets in the packet builder, so we don't check for that here

    if (Addr != dccAddress) return;   // exit if this command is not for our address

    State dccState;
    dccState = (Direction == 0) ? CURVED : STRAIGHT;
    if (dccCommandSwap) dccState = (State) !dccState; // swap the interpretation of dcc command if needed

    // if we are already in the desired position, just exit
    if (dccState == position) return;

#ifdef _DEBUG
    Serial.print("In class callback for dcc command direction ");
    Serial.println(Direction, DEC);
#endif

    // proceed only if both occupancy sensors are inactive (i.e., sensors override dcc command)
    if (osStraight.SwitchState() == HIGH && osCurved.SwitchState() == HIGH)
    {
        // set switch state based on dcc command
        position = dccState;
        led.SetLED((position == STRAIGHT) ? RgbLed::GREEN : RgbLed::RED, RgbLed::FLASH);
        SetServo(LOW);
    }
    else
    {
        // command error indication
        led.SetLED(RgbLed::BLUE, RgbLed::FLASH);

        // normal led will resume after this timer expires
        errorTimer.StartTimer(2500);
    }
}


// handle setting CVs
void TurnoutMgr::DCCPomHandler(unsigned int Addr, byte instType, unsigned int CV, byte Value)
{
    // assume we are filtering repeated packets in the packet builder, so we don't check for that here

    if (Addr != dccAddress) return;   // exit if this command is not for our address

#ifdef _DEBUG
    Serial.print("In class callback for dcc program on main, CV: ");
    Serial.print(CV, DEC);
    Serial.print(", Value: ");
    Serial.println(Value, DEC);
#endif

    // set up timer for LED indication, normal led will resume after this timer expires
    errorTimer.StartTimer(2000);
    errorTimer.SetTimerHandler(&WrapperErrorTimer);

    // check against our defined CVs to verify that the CV is valid
    boolean isValidCV = false;
    unsigned int numCVs = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
    for (unsigned int i = 0; i < numCVs; i++)
    {
        if (CV == FactoryDefaultCVs[i].CV) isValidCV = true;
    }

    // provide indication and exit if CV is invalid
    if (!isValidCV)
    {
        led.SetLED(RgbLed::YELLOW, RgbLed::FLASH);
        return;
    }

    // provide feedback that we are programming a valid CV
    led.SetLED(RgbLed::YELLOW, RgbLed::ON);

    // set the cv
    dcc.SetCV(CV, Value);

    // read back values from eeprom
    dccAddress = dcc.Address();
    servoEndPointSwap = dcc.GetCV(CV_servoEndPointSwap);
    occupancySensorSwap = dcc.GetCV(CV_occupancySensorSwap);
    dccCommandSwap = dcc.GetCV(CV_dccCommandSwap);
    relaySwap = dcc.GetCV(CV_relaySwap);

    // read back and update servo vars from eeprom
    servo.SetExtent(LOW,dcc.GetCV(CV_servoMinTravel));
    servo.SetExtent(HIGH,dcc.GetCV(CV_servoMaxTravel));
    servo.SetDuration(LOW,dcc.GetCV(CV_servoLowSpeed) * 100);
    servo.SetDuration(HIGH,dcc.GetCV(CV_servoHighSpeed) * 100);
}



// callbacks for bitstream and packet builder =============================================================

TurnoutMgr* TurnoutMgr::currentInstance=0;    // pointer to allow us to access member objects from callbacks

// this is called from the ISR for the bitstream capture, so keep it short.
// we just copy the data and set a flag that it's ready.
void TurnoutMgr::WrapperBitStream(unsigned long incomingBits) 
{
    noInterrupts();   // disable interrupts here, but shouldn't affect next dcc pulse, since this will be right after one
    currentInstance->bits = incomingBits;
    interrupts();

    currentInstance->haveNewBits = true;
}

void TurnoutMgr::WrapperBitStreamError(byte errorCode) 
{
    // TODO: add optional LED indication
    currentInstance->bitErrorCount++;
}


// this is called by the packet builder when a complete packet is ready, to kick off the actual decoding
void TurnoutMgr::WrapperDCCPacket(byte *packetData, byte size) 
{
    // kick off the packet processor
    currentInstance->dcc.ProcessPacket(packetData, size);
}

void TurnoutMgr::WrapperDCCPacketError(byte errorCode) 
{
    // TODO: add optional LED indication
    currentInstance->packetErrorCount++;
}


// ========================================================================================================
// sensor/timer callback wrappers

void TurnoutMgr::WrapperButtonPress(bool ButtonState) { currentInstance->ButtonEventHandler(ButtonState); }
void TurnoutMgr::WrapperServoMoveDon() { currentInstance->ServoMoveDoneHandler(); }
void TurnoutMgr::WrapperServoPowerOff() { currentInstance->ServoPowerOffHandler(); }
void TurnoutMgr::WrapperResetTimer() { currentInstance->ResetTimerHandler(); }
void TurnoutMgr::WrapperErrorTimer() { currentInstance->ErrorTimerHandler(); }
void TurnoutMgr::WrapperOSStraight(bool ButtonState) { currentInstance->OSStraightHandler(ButtonState); }
void TurnoutMgr::WrapperOSCurved(bool ButtonState) { currentInstance->OSCurvedHandler(ButtonState); }


// ========================================================================================================
// dcc processor callback wrappers

void TurnoutMgr::WrapperDCCAccPacket(int boardAddress, int outputAddress, byte activate, byte data)
{
    currentInstance->DCCcommandHandler(outputAddress, data);
}

void TurnoutMgr::WrapperDCCAccPomPacket(int boardAddress,int outputAddress, byte instructionType, int cv, byte data)
{
    currentInstance->DCCPomHandler(outputAddress, instructionType, cv, data);
}

void TurnoutMgr::WrapperDCCExtAccPacket(int boardAddress, int outputAddress, byte data)
{
}

void TurnoutMgr::WrapperDCCDecodingError(byte errorCode)
{
    // TODO: add optional LED indication

#ifdef _DEBUG
    Serial.print("Packet error, code: ");
    Serial.println(errorCode,DEC);
#endif
}

