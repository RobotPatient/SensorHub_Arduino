///
// Created by J.A. Korten on 19/02/2024.
//

#ifndef VECTOR3D_HPP
#define VECTOR3D_HPP

class Vector3D {
public:
  float x, y, z;

  // Default constructor
  Vector3D()
    : x(0.0), y(0.0), z(0.0) {}

  // Constructor
  Vector3D(float x_, float y_, float z_) {
    x = x_;
    y = y_;
    z = z_;
  }

  // Vector addition
  Vector3D operator+(const Vector3D& v) const {
    return Vector3D(x + v.x, y + v.y, z + v.z);
  }

  // Vector subtraction
  Vector3D operator-(const Vector3D& v) const {
    return Vector3D(x - v.x, y - v.y, z - v.z);
  }

  // Scalar multiplication
  Vector3D operator*(float scalar) const {
    return Vector3D(x * scalar, y * scalar, z * scalar);
  }

  // Scalar division
  Vector3D operator/(float scalar) const {
    return Vector3D(x / scalar, y / scalar, z / scalar);
  }

  // Dot product
  float dot(const Vector3D& v) const {
    return x * v.x + y * v.y + z * v.z;
  }

  // Cross product
  Vector3D cross(const Vector3D& v) const {
    return Vector3D(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
  }

  // Vector magnitude
  float magnitude() const {
    return sqrt(x * x + y * y + z * z);
  }

  // Normalize vector
  Vector3D normalize() const {
    float mag = magnitude();
    if (mag != 0.0f)
      return *this / mag;
    else
      return *this;
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

};



#endif /* VECTOR3D_HPP */
