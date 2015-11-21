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

#ifndef _RTIMUSETTINGS_H
#define _RTIMUSETTINGS_H

#include "RTMath.h"
#include "RTIMUHal.h"
#include <SPI.h>
#include <EEPROM.h>
#include "pins.h"

#define IMU_CHIP_SELECT  SPI_CS

//  Defines for EEPROM config

#define RTIMULIB_CAL_DATA_VALID_LOW         0xfc            // pattern to detect valid config - low byte
#define RTIMULIB_CAL_DATA_VALID_HIGH        0x15            // pattern to detect valid config - high byte

typedef struct
{
    unsigned char validL;                                   // should contain the valid pattern if a good config
    unsigned char validH;                                   // should contain the valid pattern if a good config
    unsigned char magValid;                                 // true if data valid
    unsigned char pad;
    RTFLOAT magMin[3];                                      // min values
    RTFLOAT magMax[3];                                      // max values
} RTIMULIB_CAL_DATA;

class RTIMUSettings : public RTIMUHal
{
public:
    RTIMUSettings(const char *productType = "RTIMULib");

    //  This function tries to find an IMU. It stops at the first valid one
    //  and returns true or else false
    bool discoverIMU(int& imuType);

    //  This function sets the settings to default values.
    void setDefaults();

    //  This function loads the local variables from the settings file or uses defaults
    virtual bool loadSettings();

    //  This function saves the local variables to the settings file
    virtual bool saveSettings();

    //  These are the local variables
    int m_imuType;                                          // type code of imu in use
    int m_fusionType;                                       // fusion algorithm type code
    int m_axisRotation;                                     // axis rotation code

    bool m_compassCalValid;                                 // true if there is valid compass calibration data
    RTVector3 m_compassCalMin;                              // the minimum values
    RTVector3 m_compassCalMax;                              // the maximum values

    bool m_compassCalEllipsoidValid;                        // true if the ellipsoid calibration data is valid
    RTVector3 m_compassCalEllipsoidOffset;                  // the ellipsoid offset
    float m_compassCalEllipsoidCorr[3][3];                  // the correction matrix

    float m_compassAdjDeclination;                          // magnetic declination adjustment - subtracted from measured

    bool m_accelCalValid;                                   // true if there is valid accel calibration data
    RTVector3 m_accelCalMin;                                // the minimum values
    RTVector3 m_accelCalMax;                                // the maximum values

    bool m_gyroBiasValid;                                   // true if the recorded gyro bias is valid
    RTVector3 m_gyroBias;                                   // the recorded gyro bias

    //  IMU-specific vars

    //  MPU9250
    int m_MPU9250GyroAccelSampleRate;                       // the sample rate (samples per second) for gyro and accel
    int m_MPU9250CompassSampleRate;                         // same for the compass
    int m_MPU9250GyroLpf;                                   // low pass filter code for the gyro
    int m_MPU9250AccelLpf;                                  // low pass filter code for the accel
    int m_MPU9250GyroFsr;                                   // FSR code for the gyro
    int m_MPU9250AccelFsr;                                  // FSR code for the accel

private:
    void EEErase(byte device);
    void EEWrite(byte device, RTIMULIB_CAL_DATA * calData);
    boolean EERead(byte device, RTIMULIB_CAL_DATA * calData);
};

#endif // _RTIMUSETTINGS_H

