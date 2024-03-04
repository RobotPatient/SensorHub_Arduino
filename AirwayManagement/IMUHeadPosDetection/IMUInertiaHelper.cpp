/**
* Copyright (c) 2024 RobotPatient Simulators BV. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       IMUInertiaHelper.cpp
* @date       2024-02-20
* @version    v1.0.0
* 
* Created by J.A. Korten on 20/02/2024.
*
* The IMU Inertia Helper library is very loosely coupled as it does not use particular sensor details but just their Vector3D input.
*
*/

#include "IMUInertiaHelper.h"
#include "Vector3D.h"

IMUInertiaHelper::IMUInertiaHelper() {
}

IMUInertiaHelper::IMUInertiaHelper(Vector3D thresholds) {
  _thresholds = thresholds;
}

/*!
 * @brief This method records the base line of the sensor.
 * Note: First, make sure that you let the sensor become motionless first (± 1.5 seconds after initialization) before you call this method
 * Note: Second, make sure the sensor system itself is not in motion as well of course!!!
 */

void IMUInertiaHelper::recordCurrentValues(Vector3D currentValues) {
  _inertiaBase = currentValues;
}

/*!
 * @brief This method is a helper to determine if the (sensor reading) value falls within or outside of the acceptable margins that define inertia.
 */

bool IMUInertiaHelper::twoTailedInBetween(float value, float baseValue, float threshold) {
  return (value > baseValue - threshold) && (value < baseValue + threshold);
}

/*!
 * @brief This method determines if the (sensor readings) values fall within or outside of the acceptable margins that define inertia.
 */

bool IMUInertiaHelper::checkForStasis(Vector3D currentValues, bool printResult, String label) {
  bool result = false;

  bool resultX = twoTailedInBetween(currentValues.x, _inertiaBase.x, _thresholds.x);
  bool resultY = twoTailedInBetween(currentValues.y, _inertiaBase.y, _thresholds.y);
  bool resultZ = twoTailedInBetween(currentValues.z, _inertiaBase.z, _thresholds.z);

  result = resultX && resultY && resultZ;
  if (printResult) {
    printValues(currentValues, label, result);
    printThresholds();
  }

  return result;
}


bool IMUInertiaHelper::isDelayNeeded() {
  static unsigned long lastSensorDataUpdate = 0;
  bool delayNeeded = (lastSensorDataUpdate < millis() + min_update_interval);
  if (!delayNeeded) {
    lastSensorDataUpdate = millis();
  }
  return delayNeeded;
}

void IMUInertiaHelper::checkIfDelayIsNeeded() {
  if (isDelayNeeded()) {
    delay(min_update_interval);
  }
}

void IMUInertiaHelper::updateSensorData(BMI270 *imu) {
  checkIfDelayIsNeeded();
  imu->getSensorData();
}

void IMUInertiaHelper::printValues(Vector3D values, String label, bool inStasis) {
  String stasisLabel = "";
  if (!inStasis) {
    stasisLabel = " not";
  }
  Serial.print("Sensor " + label + " is" + stasisLabel + " in stasis. (");
  Serial.print(values.x);
  Serial.print(",");
  Serial.print(values.y);
  Serial.print(",");
  Serial.print(values.z);
  Serial.print(")");
}

void IMUInertiaHelper::printThresholds() {
  Serial.print(" thresholds [");
  Serial.print(_inertiaBase.x);
  Serial.print(",");
  Serial.print(_inertiaBase.y);
  Serial.print(",");
  Serial.print(_inertiaBase.z);
  Serial.println("]");
}
