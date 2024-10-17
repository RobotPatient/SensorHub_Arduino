// DeviceManager.h

#ifndef DEVICEMANAGER_H
#define DEVICEMANAGER_H

#include "deviceHelper.h"
#include "wireHelper.h"
#include "adcHelper.h"

class DeviceManager {
public:
  DeviceManager(WireHelper &wireHelper, AdcHelper &adcHelper, byte deviceAddress)
    : wireHelper_(wireHelper),
      adcHelper_(adcHelper),
      deviceAddress_(deviceAddress) {}

  void discoverAndInit() {
    if (wireHelper_.discoverDevice(deviceAddress_, sensorPort, deviceState)) {
      deviceState = STATE_DISCOVERED;
      // ... you can add more initialization code here if needed ...
    }
  }

  void readSensorData() {
    if (deviceState == STATE_DISCOVERED) {
      adcHelper_.readSensorValues();
    }
  }

  

private:
  WireHelper &wireHelper_;
  AdcHelper &adcHelper_;
  const byte deviceAddress_;
  State deviceState = STATE_UNKNOWN;
  SensorPort sensorPort = PORT_UNKNOWN;
};

#endif  // DEVICEMANAGER_H