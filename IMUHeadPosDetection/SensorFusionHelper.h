
/*
  Helper for the sensor fusion algorithms for the IMU
  J.A. Korten Feb 21, 2024

  SensorFusionHelper.h file
*/

#ifndef IMU_FUSION_HELPER_H
#define IMU_FUSION_HELPER_H

#include "Quaternion.h"
#include "Vector3D.h"

// Mahony filter constants
#define TWO_KP (2.0f * 0.5f)  // 2 * proportional gain
#define TWO_KI (2.0f * 0.1f)  // 2 * integral gain

// Madgwick filter constants
#define BETA 0.1f  // Algorithm gain parameter

//Quaternion q(1, 0, 0, 0);  // Initial orientation quaternion
Quaternion bodyQuaternion(1.0, 0.0, 0.0, 0.0);
Quaternion headQuaternion(1.0, 0.0, 0.0, 0.0);

//#define beta 0.1f               // Filter gain
#define sampleRate 1000         // Sample rate in Hz
#define dt (1.0f / sampleRate)  // Time interval


const int interval = 1000 / sampleRate;
const int min_update_interval = 25;

Quaternion computeQuaternionDeltaMahony(Vector3D accel3D, Vector3D gyro3D);
Quaternion computeQuaternionDeltaMadgwick(Vector3D accel3D, Vector3D gyro3D);

typedef enum {
    SF_Mahony,
    SF_Madgwick
} Sensor_fusion_method;

void printAlgorithm(Sensor_fusion_method method) {
  switch (method) {
    case SF_Mahony:
      Serial.println("Fusion method: Mahony filter.");
      break;
    case SF_Madgwick:
      Serial.println("Fusion method: Madgwick filter.");
      break;
    default:
      Serial.println("Unknown sensor fusion method.");
      break;
  }
}

Quaternion computeQuaternion(Vector3D accel3D, Vector3D gyro3D, Sensor_fusion_method method) {
  switch (method) {
    case SF_Mahony:
      return computeQuaternionDeltaMahony(accel3D, gyro3D);
    case SF_Madgwick:
      return computeQuaternionDeltaMadgwick(accel3D, gyro3D);
    default:
      Serial.println("Unknown filter algoritm selected...");
      break;
  }
  return Quaternion(1.0, 0.0, 0.0, 0.0);
}

// float ax, float ay, float az, float gx, float gy, float gz, float dt
Quaternion computeQuaternionDeltaMadgwick(Vector3D accel3D, Vector3D gyro3D) {
  static Quaternion q(1.0, 0.0, 0.0, 0.0);  // Initialize quaternion

  float ax = accel3D.x;
  float ay = accel3D.y;
  float az = accel3D.z;

  float gx = gyro3D.x;
  float gy = gyro3D.y;
  float gz = gyro3D.z;

  // Convert gyroscope readings to radians per second
  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  // Normalize accelerometer readings
  float accelMag = sqrt(ax * ax + ay * ay + az * az);
  if (accelMag > 0.0) {
    ax /= accelMag;
    ay /= accelMag;
    az /= accelMag;
  }

  // Compute quaternion gradient
  float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
  float s0 = q0 * 2.0f, s1 = q1 * 2.0f, s2 = q2 * 2.0f, s3 = q3 * 2.0f;
  float qDot1 = -s1 * gx - s2 * gy - s3 * gz;
  float qDot2 = s0 * gx + s2 * gz - s3 * gy;
  float qDot3 = s0 * gy - s1 * gz + s3 * gx;
  float qDot4 = s0 * gz + s1 * gy - s2 * gx;

  // Compute feedback terms
  float qDot[4] = { qDot1, qDot2, qDot3, qDot4 };
  float qHatDot[4] = { 0.5f * (0 - q1 * BETA), 0.5f * (0 - q2 * BETA), 0.5f * (0 - q3 * BETA), 0 };

  // Compute quaternion delta
  Quaternion deltaQ = Quaternion(qHatDot[0] - qDot[0], qHatDot[1] - qDot[1], qHatDot[2] - qDot[2], qHatDot[3] - qDot[3]) * dt;
  deltaQ.normalize();

  // Update orientation quaternion
  q = q * deltaQ;
  q.normalize();

  return deltaQ;
}


Quaternion computeQuaternionDeltaMahony(Vector3D accel3D, Vector3D gyro3D) {

  float ax = accel3D.x;
  float ay = accel3D.y;
  float az = accel3D.z;

  float gx = gyro3D.x;
  float gy = gyro3D.y;
  float gz = gyro3D.z;

  static Quaternion q(1.0, 0.0, 0.0, 0.0);                               // Initialize quaternion
  static float integralFBx = 0.0, integralFBy = 0.0, integralFBz = 0.0;  // Integral error terms

  // Convert gyroscope readings to radians per second
  gx *= DEG_TO_RAD;
  gy *= DEG_TO_RAD;
  gz *= DEG_TO_RAD;

  // Normalize accelerometer readings
  float accelMag = sqrt(ax * ax + ay * ay + az * az);
  if (accelMag > 0.0) {
    ax /= accelMag;
    ay /= accelMag;
    az /= accelMag;
  }

  // Compute error quaternion
  Quaternion qError = q.conjugate() * Quaternion(0.0, ax, ay, az);
  qError.normalize();

  // Compute feedback terms
  float q0 = q.w, q1 = q.x, q2 = q.y, q3 = q.z;
  float eInt[3] = { integralFBx, integralFBy, integralFBz };
  float eProp[3] = { TWO_KP * qError.x, TWO_KP * qError.y, TWO_KP * qError.z };
  float eInteg[3] = { TWO_KI * eInt[0], TWO_KI * eInt[1], TWO_KI * eInt[2] };

  // Compute angular error correction
  float ex = eProp[0] + eInteg[0];
  float ey = eProp[1] + eInteg[1];
  float ez = eProp[2] + eInteg[2];

  // Compute quaternion delta
  Quaternion deltaQ = Quaternion(1.0, gx - ex, gy - ey, gz - ez) * 0.5 * dt;
  deltaQ.normalize();

  // Update integral error terms
  integralFBx += ex * dt;
  integralFBy += ey * dt;
  integralFBz += ez * dt;

  // Update orientation quaternion
  q = q * deltaQ;
  q.normalize();

  return deltaQ;
}

#endif /* IMU_FUSION_HELPER_H */