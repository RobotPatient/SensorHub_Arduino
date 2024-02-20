///
// Created by J.A. Korten on 19/02/2024.
//

#ifndef IMUAccelStationary_h
#define IMUAccelStationary_h

#include <Arduino.h>
#include "Vector3D.h"

class IMUAccelStationary {
public:
    IMUAccelStationary(float threshold, unsigned long duration);
    bool lastStateIsStationary();
    bool isStationaryAndUpdate();
    bool stationaryStateChanged(); // Check if stationary state has changed since last check
    void addAcceleration(const Vector3D& acceleration); // Add a Vector3D object to the array

private:
    float _threshold;
    unsigned long _duration;
    unsigned long _lastStationaryTime;

    bool _lastStationaryState;
    bool _initialStateChecked;
    float calculateStandardDeviation(float* values, int size);
    Vector3D _accelerationArray[100]; // Array to store accelerometer readings
    int _arrayIndex; // Index to keep track of the current position in the array
};

#endif
