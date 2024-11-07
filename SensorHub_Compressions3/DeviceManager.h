// DeviceManager.h

#ifndef DEVICEMANAGER_H
#define DEVICEMANAGER_H

#include "deviceHelper.h"
#include "wireHelper.h"
#include "adcHelper.h"

class DeviceManager {
public:
  DeviceManager(AdcHelper &adcHelper, byte deviceAddress) :
    adcHelper_(adcHelper),
    deviceAddress_(deviceAddress),
    deviceState_(STATE_UNKNOWN),
    sensorPort_(PORT_UNKNOWN),
    wireHelper_(nullptr) 
  {}

  void discoverAndInit(WireHelper &wireHelper) { 
    if (wireHelper.isDeviceConnected(deviceAddress_, sensorPort_)) {
      wireHelper_ = &wireHelper;  
      deviceState_ = STATE_DISCOVERED;
    } else {
      deviceState_ = STATE_NOT_FOUND; 
    }
  }

  State getDeviceState() const { 
    return deviceState_; 
  }

  void readSensorData() {
    if (deviceState_ == STATE_DISCOVERED && wireHelper_ != nullptr) {
      adcHelper_.readSensorValues(*wireHelper_->getWire()); 
    }
  }

private:
  AdcHelper &adcHelper_;
  const byte deviceAddress_;
  State deviceState_;
  SensorPort sensorPort_;
  WireHelper *wireHelper_; 
};

#endif // DEVICEMANAGER_H