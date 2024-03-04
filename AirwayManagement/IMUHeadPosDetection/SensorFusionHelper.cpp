
/*
  Helper for the sensor fusion algorithms for the IMU
  J.A. Korten Feb 21, 2024

  SensorFusionHelper.cpp file
*/

#include "SensorFusionHelper.h"
#include "Quaternion.h"

SensorFusionHelper::SensorFusionHelper() {
  SensorFusionHelper(SF_Mahony);
}

SensorFusionHelper::SensorFusionHelper(Sensor_fusion_method method) {
  _method = method;
  _q = Quaternion(1.0, 0.0, 0.0, 0.0);
}

void SensorFusionHelper::printAlgorithm() {
  switch (_method) {
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

Quaternion SensorFusionHelper::computeQuaternion(Vector3D accel3D, Vector3D gyro3D) {
  switch (_method) {
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
Quaternion SensorFusionHelper::computeQuaternionDeltaMadgwick(Vector3D accel3D, Vector3D gyro3D) {
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
  float q0 = _q.w, q1 = _q.x, q2 = _q.y, q3 = _q.z;
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
  _q = _q * deltaQ;
  _q.normalize();

  return deltaQ;
}

Quaternion SensorFusionHelper::getOrientation() {
  return _q;
}

void SensorFusionHelper::setOrientation(Quaternion q) {
  _q = q;
}


Quaternion SensorFusionHelper::computeQuaternionDeltaMahony(Vector3D accel3D, Vector3D gyro3D) {
  float ax = accel3D.x;
  float ay = accel3D.y;
  float az = accel3D.z;

  float gx = gyro3D.x;
  float gy = gyro3D.y;
  float gz = gyro3D.z;

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
  Quaternion qError = _q.conjugate() * Quaternion(0.0, ax, ay, az);
  qError.normalize();

  // Compute feedback terms
  float q0 = _q.w, _q1 = _q.x, q2 = _q.y, q3 = _q.z;
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
  _q = _q * deltaQ;
  _q.normalize();

  return deltaQ;
}