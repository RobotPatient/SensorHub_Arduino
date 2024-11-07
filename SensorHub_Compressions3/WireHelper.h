// wireHelper.h

#ifndef WIREHELPER_H
#define WIREHELPER_H

#include <Wire.h>
#include "deviceHelper.h"

class WireHelper {
public:
  WireHelper(TwoWire &wire, const char *label) :
    wire_(wire),
    label_(label)
  {
    wire_.begin();
  }

  bool reportDevicesWithAddress(byte deviceAddress, bool report = false) {
    wire_.beginTransmission(deviceAddress);
    byte error = wire_.endTransmission();
    bool result = (error == 0);

    if (report) {
      if (result) {
        Serial.printf("Device was found on address 0x%02X on bus %s\n", deviceAddress, label_);
      } else {
        Serial.printf("No device was found on address 0x%02X on bus %s\n", deviceAddress, label_);
      }
    }
    return result;
  }

  void reportDevices() {
    Serial.printf("Scanning bus %s\n", label_);
    int nDevices = 0;
    for (byte address = 1; address < 127; address++) {
      if (reportDevicesWithAddress(address, true)) {
        printDeviceName(address);
        nDevices++;
      }
    }
    if (nDevices == 0) {
      Serial.printf("No I2C devices found on bus %s\n\n", label_);
    } else {
      Serial.println("done\n");
    }
  }

  bool isDeviceConnected(byte address, SensorPort &port) {
    if (reportDevicesWithAddress(address)) {
      port = (label_ == "A") ? SensorPort::SENSOR_PORT_A : SensorPort::SENSOR_PORT_B;
      return true;
    }
    return false; 
  }

  TwoWire* getWire() { 
    return &wire_; 
  }

private:
  void printDeviceName(int address) {
    switch (address) {
      case 0x10:
        Serial.println("ADS7138 for Touch Sensors found...");
        break;
      case 0x25:
        Serial.println("Sensirion SDP800-500Pa found...");
        break;
      case 0x26:
        Serial.println("Sensirion SDP800-501Pa found...");
        break;
      case 0x29:
        Serial.println("DLC-L01G-U2 or VL6180 found...");
        break;
      case 0x40:
        Serial.println("Sensirion SDP610-500Pa found...");
        break;
      case 0x50:
        Serial.println("FRAM/EEPROM found...");
        break;
      case 0x51:
        Serial.println("More memory found? Could be 1M FRAM.");
        break;
      case 0x7C:
        Serial.println("RESERVED...");
        break;
    }
  }

  TwoWire &wire_;
  const char *label_;
};

#endif // WIREHELPER_H