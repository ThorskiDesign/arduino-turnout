/*

This file is part of Arduino Turnout
Copyright (C) 2017-2018 Eric Thorstenson

Arduino Turnout is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Arduino Turnout is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.

*/


#include "Bitstream.h"


// global stuff
BitStream bitStream;
boolean haveNewBits = false;
boolean currentBit = 0;
volatile unsigned long bits = 0;
byte numBits = 0;
int y = 0;
unsigned long errorCount = 0;


void BitStreamHandler(unsigned long incomingBits)
{
    //noInterrupts();   // disable interrupts here, but shouldn't affect next dcc pulse, since this will be right after one
    bits = incomingBits;
    //interrupts();

    haveNewBits = true;
}


void BitErrorHandler(byte errorCode)
{
    errorCount++;
}


void setup()
    // put your setup code here, to run once:
{
    // for testing timing
    pinMode(18, OUTPUT);
	pinMode(19, OUTPUT);

    Serial.begin(115200);

    bitStream.SetDataFullHandler(&BitStreamHandler);
    bitStream.SetErrorHandler(&BitErrorHandler);

    bitStream.Resume();
}


unsigned long lastMillis = 0;
unsigned long lastMillis2 = 0;

void loop()
    // put your main code here, to run repeatedly:
{
	//PORTC = PORTC | (1 << 5); PORTC = PORTC & ~(1 << 5);      // pulse pin 19

	bitStream.ProcessTimestamps();

    unsigned long currentMillis = millis();

    // test suspend/resume
    if (currentMillis - lastMillis2 > 5000)
    {
        Serial.println("Suspend/Resume");
        bitStream.Suspend();
        bitStream.Resume();
        lastMillis2 = currentMillis;
    }

    // print debug info
    if (currentMillis - lastMillis > 1000)
    {
        lastMillis = currentMillis;

        int j = 1;
        for (int i = 0; i < 32; i++)
            //for (unsigned long mask = 0x80000000; mask; mask >>=1)
        {
            unsigned long mask = (1ul << i);
            Serial.print((mask & bits) ? '1' : '0');
            if (j % 4 == 0) Serial.print(" ");
            j++;
        }

        Serial.print("      Errors: ");
        Serial.println(errorCount, DEC);

        errorCount = 0;
    }
}
