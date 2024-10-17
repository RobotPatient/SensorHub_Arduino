#ifndef PINMANAGER_H
#define PINMANAGER_H

#include "pinTools.h"
#include <samd.h> // Include the SAMD core header

class PinManager {
public:
  // Define your pin numbers here (remove from SensorHub_Settings.h)
  static const uint32_t W0_SCL = 27;  // PA22
  static const uint32_t W0_SDA = 26;  // PA23
  static const uint32_t W1_SCL = 39;  // PA13
  static const uint32_t W1_SDA = 28;  // PA12
  static const uint32_t W2_SCL = 13;  // PA17
  static const uint32_t W2_SDA = 11;  // PA16

  static void setupI2CPins() {
    TwiPinPair portBackbone = TwiPinPair(W0_SCL, W0_SDA);
    TwiPinPair portSensorsA = TwiPinPair(W1_SCL, W1_SDA);
    TwiPinPair portSensorsB = TwiPinPair(W2_SCL, W2_SDA);

    portBackbone.setPinPeripheralStates();
    portSensorsA.setPinPeripheralAltStates();
    portSensorsB.setPinPeripheralStates();
  }
};

#endif // PINMANAGER_H