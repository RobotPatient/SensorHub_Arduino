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
* @file       IMUInertiaController.h
* @date       2024-02-20
* @version    v1.0.0
* 
* Created by J.A. Korten on 20/02/2024.
*
* The IMU Inertia Helper library is very loosely coupled as it does not use particular sensor details but just their Vector3D input.
*
*/

#include "IMUInertiaHelper.h"
#include "IMUHelper.h"
#include "SensorFusionHelper.h"


void updateSensor3DVector(BMI270 *imu, IMUInertiaHelper *helper, Sensor_fusion_method method) {
  helper->updateSensorData(imu);

  Vector3D accel = Vector3D(imu->data.accelX, imu->data.accelY, imu->data.accelZ);

  helper->recordCurrentValues(accel);
}

Quaternion updateSensor(BMI270 *imu, IMUInertiaHelper *helper, Sensor_fusion_method method, String sensorLabel) {
  static Quaternion orientation = Quaternion(1.0, 0.0, 0.0, 0.0);

  helper->updateSensorData(imu);

  // updateSensorData(BMI270 *imu, Quaternion *q, Sensor_fusion_method method)

  Vector3D currentValues = Vector3D(imu->data.accelX, imu->data.accelY, imu->data.accelZ);
  const bool REPORT_SENSOR_STATE = false;
  bool inStasis = helper->checkForStasis(currentValues, REPORT_SENSOR_STATE, sensorLabel);

  if (inStasis) {
    // recalibrate if needed.
  }

  Vector3D accel3D = Vector3D(imu->data.accelX, imu->data.accelY, imu->data.accelZ);
  Vector3D gyro3D = Vector3D(imu->data.gyroX, imu->data.gyroY, imu->data.gyroZ);

  // Convert gyroscope readings to radians per second
  gyro3D.x *= DEG_TO_RAD;
  gyro3D.y *= DEG_TO_RAD;
  gyro3D.z *= DEG_TO_RAD;

  Quaternion deltaQ = computeQuaternion(accel3D, gyro3D, method);
  // Update orientation quaternion
  orientation = orientation * deltaQ;
  orientation.normalize();  // Normalize quaternion
  
  return orientation;
}
