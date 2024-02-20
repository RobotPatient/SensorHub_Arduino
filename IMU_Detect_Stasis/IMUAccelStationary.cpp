///
// Created by J.A. Korten on 19/02/2024.
//

#include "IMUAccelStationary.h"

#define MAX_ITEMS 5
#define MIN_ITEMS 5

const int WINDOW_SIZE = MAX_ITEMS;  // Number of samples for standard deviation calculation
float magnitudes[WINDOW_SIZE];


IMUAccelStationary::IMUAccelStationary(float threshold, unsigned long duration)
  : _threshold(threshold), _duration(duration), _lastStationaryTime(0), _lastStationaryState(false), _initialStateChecked(false) {}

bool IMUAccelStationary::isStationaryAndUpdate() {
  // Check if enough readings are available in the array
  if (_arrayIndex < MIN_ITEMS) {
    return false;  // Not enough readings for calculation
  }

  // Calculate magnitudes of accelerometer readings
  for (int i = 0; i < _arrayIndex; i++) {
    float ax = _accelerationArray[i].x;
    float ay = _accelerationArray[i].y;
    float az = _accelerationArray[i].z;
    magnitudes[i] = sqrt(ax * ax + ay * ay + az * az);
  }

  // Calculate standard deviation
  float stdDev = calculateStandardDeviation(magnitudes, WINDOW_SIZE);

#ifdef DEBUG
  Serial.print(stdDev);
  Serial.print(" < ");
  Serial.println(_threshold);
#endif

  // Determine stationary state based on threshold
  if (stdDev < _threshold) {
    if (millis() - _lastStationaryTime >= _duration) {
      _lastStationaryTime = millis();
      if (_lastStationaryState) {
        return false;
      } else {
        _lastStationaryState = true;
        return true;
      }
    }
  } else {
    _lastStationaryState = false;
    return false;
  }
  return false;
}

float IMUAccelStationary::calculateStandardDeviation(float* values, int size) {
  if (size <= 0) {
    return 0.0;  // Return 0 if the array size is 0 or negative
  }

  float mean = 0.0;
  float variance = 0.0;

  // Calculate the mean
  for (int i = 0; i < size; i++) {
    mean += values[i];
  }
  mean /= size;

  // Calculate the variance
  for (int i = 0; i < size; i++) {
    variance += (values[i] - mean) * (values[i] - mean);
  }
  variance /= size;

  // Return the square root of the variance to get the standard deviation
  return sqrt(variance);
}

void IMUAccelStationary::addAcceleration(const Vector3D& acceleration) {
  _accelerationArray[_arrayIndex] = acceleration;
  _arrayIndex++;
  // If the array is full, reset the index to 0
  if (_arrayIndex >= 100) {
    _arrayIndex = 0;
  }
}

bool IMUAccelStationary::lastStateIsStationary() {
  return (_lastStationaryState);
}
