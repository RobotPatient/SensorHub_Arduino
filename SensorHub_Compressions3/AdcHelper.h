// adcHelper.h

#ifndef ADCHELPER_H
#define ADCHELPER_H

#include <Wire.h>
#include "ads7138_definitions.h"

class AdcHelper {
public:
  AdcHelper(byte address) :  
    address_(address)
  {}

  void readSensorValues(TwoWire &wire) {  
    int sensorData[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    for (int channel = 0; channel < 8; ++channel) {
      writeToRegister(wire, CHANNEL_SEL, channel); 
      delay(1);

      wire.requestFrom(address_, 2);
      if (wire.available() >= 2) {
        int val = wire.read() << 8 | wire.read();
        sensorData[channel] = val;
      }
      delay(1);
    }

    for (int index = 0; index < 8; ++index) {
      Serial.print(sensorData[index]);
      Serial.print(" ");
    }
    Serial.println();
  }

private:
  void writeToRegister(TwoWire &wire, byte registerAddress, byte command) { 
    wire.beginTransmission(address_);
    wire.write(SINGLE_WRITE);
    wire.write(registerAddress);
    wire.write(command);
    wire.endTransmission();
  }

  const byte address_;
};

#endif // ADCHELPER_H