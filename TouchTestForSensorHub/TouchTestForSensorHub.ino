// --------------------------------------
// i2c_scanner for SAMDxx microcontrollers with Arduino bootloader
// (e.g. Feather M0 Express by Adafruit)
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
// Changes for RobotPatient Simulators and HAN University of Applied Sciences
// Nov 5, 2021 J.A. Korten
// Scanner for 3 i2c channels:
// Escpecially for DevBoard for Ventilations / Soft Robotics actuators.
//
// Based on earlier work by Krodal and Nick Gammon
// See also: https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-wire
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
//
// Changes for RobotPatient Simulators and HAN University of Applied Sciences
// Nov 5, 2021 J.A. Korten
// Scanner for 3 i2c channels:
// Escpecially for DevBoard for Ventilations / Soft Robotics actuators.
//

#include <Wire.h>            // SERCOM3?
#include "deviceHelper.h"

State deviceState = STATE_UNKNOWN;
SensorPort sensorPort = PORT_UNKNOWN;

#include "wiring_private.h"  // pinPeripheral() function
#include "pinTools.h"

#include "wireHelper.h"
#include "serialHelper.h"
#include "ledHelper.h"
#include "adcHelper.h"

#define ADS7138_Address 0x10

void setup() {
  setupSerial();
  setupWireBegins();
  setupLEDs();
  setupWireSetStates();
  waitForSerial();
  heartBeatOff();
  discoverADS7138();
}

void loop() {
  updateHeartBeat();
  if (deviceState == STATE_DISCOVERED) {
    readSensorValues(sensorPort, ADS7138_Address);
    delay(10);
  }
}

void discoverADS7138() {
  if (discoverDevice(ADS7138_Address)) {
    deviceState = STATE_DISCOVERED;
    resetStatus(ADS7138_Address);
  }
}
