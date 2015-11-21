////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Teensy
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//  The MPU-9250 driver code is based on code generously supplied by
//  staslock@gmail.com (www.clickdrive.io)

#include "RTIMUHal.h"
#include <SPI.h>
#include "pins.h"

RTIMUHal::RTIMUHal()
{
}

RTIMUHal::~RTIMUHal()
{
}

bool RTIMUHal::HALOpen()
{
    SPI.begin();
    pinMode(m_SPISelect, OUTPUT);
    SPI.setSCK(SPI_SCLK);
    SPI.setMOSI(SPI_MOSI);
    SPI.setMISO(SPI_MISO);
    m_SPISettings = SPISettings(m_SPISpeed, MSBFIRST, SPI_MODE0);
    return true;
}

void RTIMUHal::HALClose()
{
    SPI.end();
}

bool RTIMUHal::HALRead(unsigned char regAddr, unsigned char length, unsigned char *data, const char *msg)
{
    SPI.beginTransaction(m_SPISettings);
    digitalWrite(m_SPISelect, LOW);
    SPI.transfer(regAddr | 0x80);
    for (int i = 0; i < length; i++)
        data[i] = SPI.transfer(0);
    digitalWrite(m_SPISelect, HIGH);
    SPI.endTransaction();
    return true;
}

bool RTIMUHal::HALWrite(unsigned char regAddr, unsigned char length, unsigned char const *data, const char *msg)
{
    SPI.beginTransaction(m_SPISettings);
    digitalWrite(m_SPISelect, LOW);
    SPI.transfer(regAddr);
    for (int i = 0; i < length; i++)
        SPI.transfer(data[i]);
    digitalWrite(m_SPISelect, HIGH);
    SPI.endTransaction();
    return true;
}

bool RTIMUHal::HALWrite(unsigned char regAddr, unsigned char const data, const char *msg)
{
    SPI.beginTransaction(m_SPISettings);
    digitalWrite(m_SPISelect, LOW);
    SPI.transfer(regAddr);
    SPI.transfer(data);
    digitalWrite(m_SPISelect, HIGH);
    SPI.endTransaction();
    return true;
}

void RTIMUHal::delayMs(int milliSeconds)
{
    delay(milliSeconds);
}
