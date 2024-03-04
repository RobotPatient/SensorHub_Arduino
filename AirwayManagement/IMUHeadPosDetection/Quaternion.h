///
// Created by J.A. Korten on 19/02/2024.
//

#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include "Vector3D.h"

class Quaternion {
public:
  float w, x, y, z;

  // Default constructor
  Quaternion()
    : w(1.0), x(0.0), y(0.0), z(0.0) {}

  // Constructor
  Quaternion(float w_, float x_, float y_, float z_) {
    w = w_;
    x = x_;
    y = y_;
    z = z_;
  }

  // Normalize quaternion
  void normalize() {
    float norm = sqrt(w * w + x * x + y * y + z * z);
    w /= norm;
    x /= norm;
    y /= norm;
    z /= norm;
  }

  // Convert quaternion to Euler angles
  void toEulerAngles(float& roll, float& pitch, float& yaw) const {
    // Roll (x-axis rotation)
    float sinr = 2.0f * (w * x + y * z);
    float cosr = 1.0f - 2.0f * (x * x + y * y);
    roll = atan2(sinr, cosr);

    // Pitch (y-axis rotation)
    float sinp = 2.0f * (w * y - z * x);
    if (fabs(sinp) >= 1)
      pitch = copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
    else
      pitch = asin(sinp);

    // Yaw (z-axis rotation)
    float siny = 2.0f * (w * z + x * y);
    float cosy = 1.0f - 2.0f * (y * y + z * z);
    yaw = atan2(siny, cosy);
  }

  // Quaternion multiplication
  Quaternion operator*(const Quaternion& q) const {
    return Quaternion(
      w * q.w - x * q.x - y * q.y - z * q.z,
      w * q.x + x * q.w + y * q.z - z * q.y,
      w * q.y - x * q.z + y * q.w + z * q.x,
      w * q.z + x * q.y - y * q.x + z * q.w);
  }

  // Quaternion multiplication by a scalar
  Quaternion operator*(const float scalar) const {
    return Quaternion(w * scalar, x * scalar, y * scalar, z * scalar);
  }

  // Quaternion addition
  Quaternion operator+(const Quaternion& q) {
    Quaternion result;
    result.w = w + q.w;
    result.x = x + q.x;
    result.y = y + q.y;
    result.z = z + q.z;
    return result;
  }

  // Quaternion addition assignment operator
  Quaternion& operator+=(const Quaternion& q) {
    w += q.w;
    x += q.x;
    y += q.y;
    z += q.z;
    return *this;
  }

  // Rotate vector using quaternion
  Vector3D rotate(const Vector3D& v) {
    Quaternion p(0, v.x, v.y, v.z);
    Quaternion conj = conjugate();
    Quaternion rotated = (*this) * p * conj;
    return Vector3D(rotated.x, rotated.y, rotated.z);
  }

  // Compute conjugate of quaternion
  Quaternion conjugate() const {
    return Quaternion(w, -x, -y, -z);
  }

  // Print vector to Serial
  void printToSerial() const {
    Serial.print("(");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(y);
    Serial.print(", ");
    Serial.print(z);
    Serial.println(")");
  }

  void printEulerAngles(float roll, float pitch, float yaw, String label) {
    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" pitch: ");
    Serial.print(pitch);
    Serial.print(" yaw: ");
    Serial.print(yaw);
    Serial.print(" (");
    Serial.print(label);
    Serial.print(")");
  }

  Quaternion operator-(const Quaternion& rhs) const {
    return Quaternion(w - rhs.w, x - rhs.x, y - rhs.y, z - rhs.z);
  }

  // Other quaternion operations...
};


#endif /* QUATERNION_HPP */