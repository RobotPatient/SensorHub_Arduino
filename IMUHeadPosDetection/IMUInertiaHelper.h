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
* @file       IMUInertiaHelper.h
* @date       2024-02-20
* @version    v1.0.0
* 
* Created by J.A. Korten on 20/02/2024.
*
*/

#ifndef IMUInertiaHelper_h
#define IMUInertiaHelper_h

#include <Arduino.h>
#include "Vector3D.h"
#include "InertiaMeasurements.h"
#include "SparkFun_BMI270_Arduino_Library.h"


#define min_update_interval 25  // Consideration: maybe use the constructor to update this?

class IMUInertiaHelper {
public:

  IMUInertiaHelper();
  IMUInertiaHelper(Vector3D thresholds);

  void recordCurrentValues(Vector3D currentValues);

  bool checkForStasis(Vector3D currentValues, bool printResult = false, String label = "");
  bool twoTailedInBetween(float value, float baseValue, float threshold);
  void printValues(Vector3D values, String label, bool inStasis);
  void printThresholds();

  void updateSensorData(BMI270 *imu);
  bool isDelayNeeded();
  void checkIfDelayIsNeeded();

private:
  Vector3D _thresholds = Vector3D(0.01, 0.01, 0.01);
  Vector3D _inertiaBase = Vector3D(0.0, 0.0, 0.0);
};

#endif
