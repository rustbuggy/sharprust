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


#include "RTIMUSettings.h"
#include "RTIMUMPU9250.h"

#define BUFFER_SIZE 200

RTIMUSettings::RTIMUSettings(const char *productType)
{
  loadSettings();
}

bool RTIMUSettings::discoverIMU(int& imuType)
{
  unsigned char result;

  //  auto detect on SPI bus
  m_SPIBus = 0;
  m_SPISelect = IMU_CHIP_SELECT;

  if (HALOpen()) {
    if (HALRead(MPU9250_WHO_AM_I, 1, &result, "")) {
      if (result == MPU9250_ID) {
        imuType = RTIMU_TYPE_MPU9250;
        HAL_INFO1("Detected MPU9250 on SPI bus 0, select %d\n", IMU_CHIP_SELECT);
        return true;
      }
    }
    HALClose();
  }

  HAL_ERROR("No IMU detected\n");
  return false;
}

void RTIMUSettings::setDefaults()
{
  //  preset general defaults
  //m_imuType = RTIMU_TYPE_MPU9250;
  m_imuType = RTIMU_TYPE_AUTODISCOVER;
  m_SPIBus = 0;
  m_SPISelect = IMU_CHIP_SELECT;
  m_SPISpeed = 500000;
  m_fusionType = RTFUSION_TYPE_RTQF;
  m_axisRotation = RTIMU_XNORTH_YEAST;
  m_compassCalValid = false;
  m_compassCalEllipsoidValid = false;
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      m_compassCalEllipsoidCorr[i][j] = 0;
    }
  }
  m_compassCalEllipsoidCorr[0][0] = 1;
  m_compassCalEllipsoidCorr[1][1] = 1;
  m_compassCalEllipsoidCorr[2][2] = 1;

  m_compassAdjDeclination = 0;

  m_accelCalValid = false;
  m_gyroBiasValid = false;

  //  MPU9250 defaults
  m_MPU9250GyroAccelSampleRate = 80;
  m_MPU9250CompassSampleRate = 40;
  m_MPU9250GyroLpf = MPU9250_GYRO_LPF_41;
  m_MPU9250AccelLpf = MPU9250_ACCEL_LPF_41;
  m_MPU9250GyroFsr = MPU9250_GYROFSR_1000;
  m_MPU9250AccelFsr = MPU9250_ACCELFSR_8;
}

bool RTIMUSettings::loadSettings()
{
  setDefaults();

  // see if EEPROM has valid cal data
  m_compassCalValid = false;

  RTIMULIB_CAL_DATA calData;
  if (EERead(0, &calData)) {
    if (calData.magValid != 1) {
      return true;
    }
  }
  m_compassCalValid = true;
  m_compassCalMin.setX(calData.magMin[0]);
  m_compassCalMin.setY(calData.magMin[1]);
  m_compassCalMin.setZ(calData.magMin[2]);
  m_compassCalMax.setX(calData.magMax[0]);
  m_compassCalMax.setY(calData.magMax[1]);
  m_compassCalMax.setZ(calData.magMax[2]);
  return true;
}

bool RTIMUSettings::saveSettings()
{
  RTIMULIB_CAL_DATA calData;

  calData.magValid = m_compassCalValid;
  calData.magMin[0] = m_compassCalMin.x();
  calData.magMin[1] = m_compassCalMin.y();
  calData.magMin[2] = m_compassCalMin.z();
  calData.magMax[0] = m_compassCalMax.x();
  calData.magMax[1] = m_compassCalMax.y();
  calData.magMax[2] = m_compassCalMax.z();

  EEWrite(0, &calData);
  return true;
}

void RTIMUSettings::EEErase(byte device)
{
  EEPROM.write(sizeof(RTIMULIB_CAL_DATA) * device, 0);    // just destroy the valid byte
}

void RTIMUSettings::EEWrite(byte device, RTIMULIB_CAL_DATA *calData)
{
  byte *ptr = (byte *)calData;
  byte length = sizeof(RTIMULIB_CAL_DATA);
  int eeprom = sizeof(RTIMULIB_CAL_DATA) * device;

  calData->validL = RTIMULIB_CAL_DATA_VALID_LOW;
  calData->validH = RTIMULIB_CAL_DATA_VALID_HIGH;

  for (byte i = 0; i < length; i++)
    EEPROM.write(eeprom + i, *ptr++);
}

boolean RTIMUSettings::EERead(byte device, RTIMULIB_CAL_DATA *calData)
{
  byte *ptr = (byte *)calData;
  byte length = sizeof(RTIMULIB_CAL_DATA);
  int eeprom = sizeof(RTIMULIB_CAL_DATA) * device;

  calData->magValid = false;

  if ((EEPROM.read(eeprom) != RTIMULIB_CAL_DATA_VALID_LOW) ||
      (EEPROM.read(eeprom + 1) != RTIMULIB_CAL_DATA_VALID_HIGH)) {
    return false;                                  // invalid data
  }

  for (byte i = 0; i < length; i++)
    *ptr++ = EEPROM.read(eeprom + i);
  return true;
}

