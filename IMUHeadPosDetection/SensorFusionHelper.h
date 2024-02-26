
/*
  Helper for the sensor fusion algorithms for the IMU
  J.A. Korten Feb 21, 2024

  SensorFusionHelper.h file
*/

#ifndef IMU_SENSOR_FUSION_HELPER_H
#define IMU_SENSOR_FUSION_HELPER_H

#include "Quaternion.h"
#include "Vector3D.h"
#include "Arduino.h"


// Mahony filter constants
#define TWO_KP (2.0f * 0.5f)  // 2 * proportional gain
#define TWO_KI (2.0f * 0.1f)  // 2 * integral gain

// Madgwick filter constants
#define BETA 0.1f  // Algorithm gain parameter

//#define beta 0.1f               // Filter gain
#define sampleRate 1000         // Sample rate in Hz
#define dt (1.0f / sampleRate)  // Time interval


const int interval = 1000 / sampleRate;
const int min_update_interval = 25;


typedef enum {
  SF_Mahony,
  SF_Madgwick
} Sensor_fusion_method;

class SensorFusionHelper {
public:

  SensorFusionHelper();
  SensorFusionHelper(Sensor_fusion_method method);

  void printAlgorithm();
  Quaternion computeQuaternion(Vector3D accel3D, Vector3D gyro3D);
  Quaternion computeQuaternionDeltaMahony(Vector3D accel3D, Vector3D gyro3D);
  Quaternion computeQuaternionDeltaMadgwick(Vector3D accel3D, Vector3D gyro3D);

  Quaternion getOrientation();
  void setOrientation(Quaternion q);
private:
  Sensor_fusion_method _method;
  Quaternion _q; // = Quaternion(1.0, 0.0, 0.0, 0.0);  // Initialize quaternion

  //Quaternion q(1, 0, 0, 0);  // Initial orientation quaternion
  //Quaternion localQuaternion = Quaternion(1.0, 0.0, 0.0, 0.0); // possibly not needed?
  // Quaternion headQuaternion(1.0, 0.0, 0.0, 0.0);
};


#endif /* IMU_SENSOR_FUSION_HELPER_H */