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

#include "Accel.h"


Accel::Accel() {
}

Accel::~Accel() {
  delete settings;
}

void Accel::setup()
{
  int errcode;

  settings = new RTIMUSettings();
  imu = RTIMU::createIMU(settings);                        // create the imu object

  //Serial3.print("Accel starting using device "); Serial3.println(imu->IMUName());
  if ((errcode = imu->IMUInit()) < 0) {
    //Serial3.print("Failed to init IMU: "); Serial3.println(errcode);
  }

  if (imu->getCompassCalibrationValid())
  {
    //Serial3.println("Using compass calibration");
  }
  else
  {
    //Serial3.println("No valid compass calibration data");
  }

  lastRate = millis();
  lastDisplay = lastRate;
  sampleCount = 0;

  gravity.setScalar(0);
  gravity.setX(0);
  gravity.setY(0);
  gravity.setZ(1);
}

RTVector3& Accel::getRealAccel(uint32_t time)
{
  unsigned long delta;
  RTQuaternion rotatedGravity;
  RTQuaternion fusedConjugate;
  RTQuaternion qTemp;
  RTIMU_DATA imuData;

  //Serial3.println("getRealAccel");

  if (imu->IMURead()) {                                // get the latest data if ready yet
    imuData = imu->getIMUData();

    //  do gravity rotation and subtraction
    // create the conjugate of the pose
    fusedConjugate = imuData.fusionQPose.conjugate();

    // currTime do the rotation - takes two steps with qTemp as the intermediate variable
    qTemp = gravity * imuData.fusionQPose;
    rotatedGravity = fusedConjugate * qTemp;

    // currTime adjust the measured Accel and change the signs to make sense
    realAccel.setX(-(imuData.accel.x() - rotatedGravity.x()));
    realAccel.setY(-(imuData.accel.y() - rotatedGravity.y()));
    realAccel.setZ(-(imuData.accel.z() - rotatedGravity.z()));

    sampleCount++;
    if ((delta = time - lastRate) >= 1000) {
      imu->IMUGyroBiasValid();
      //Serial3.print("Sample rate: "); Serial3.print(sampleCount);
      //if (!imu->IMUGyroBiasValid())
      //  Serial3.println(", calculating gyro bias");
      //else
      //  Serial3.println();

      sampleCount = 0;
      lastRate = time;
    }

    //if ((time - lastDisplay) >= 100) {
    //  lastDisplay = time;
    //  if (realAccel.length() > 0.1)
    //    Serial3.println(RTMath::displayRadians("Accel:", realAccel));
    //}
  }

  return realAccel;
}

