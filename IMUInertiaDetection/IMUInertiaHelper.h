///
// Created by J.A. Korten on 19/02/2024.
//

#ifndef IMUInertiaHelper_h
#define IMUInertiaHelper_h

#include <Arduino.h>
#include "Vector3D.h"

class IMUInertiaHelper {
public:
    IMUInertiaHelper();
    IMUInertiaHelper(Vector3D thresholds);

private:
    Vector3D _thresholds = Vector3D(0.01, 0.01, 0.01);
    
};

#endif
