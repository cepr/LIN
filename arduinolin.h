/*
 Copyright 2011 G. Andrew Stone
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Neither the name of the copyright holder nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ARDUINOLIN_H
#define ARDUINOLIN_H

#include "abstractlin.h"
#include "Arduino.h"
#include <HardwareSerial.h>

#define LIN_SERIAL            HardwareSerial

class ArduinoLin: public AbstractLin
{
protected:
    LIN_SERIAL& serial;
    uint8_t txPin;               //  what pin # is used to transmit (needed to generate the BREAK signal)
    uint8_t serialOn;            //  whether the serial port is "begin"ed or "end"ed.  Optimization so we don't begin twice.

    // Abstract UART methods
    virtual void serialFlush(void);
    virtual void serialBreak(void);
    virtual void serialWrite(uint8_t b);
    virtual void serialWrite(const uint8_t* data, uint8_t length);
    virtual size_t serialAvailable(void);
    virtual uint8_t serialRead(void);
    virtual void enableTx(void);
    virtual void disableTx(void);

    // Abstract timer methods
    virtual uint32_t millis();

public:
    ArduinoLin(LIN_SERIAL& ser=Serial,uint8_t txPin=1);
    virtual void begin(int speed);
};

#endif // ARDUINOLIN_H
