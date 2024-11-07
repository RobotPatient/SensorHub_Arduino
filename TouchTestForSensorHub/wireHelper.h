// SensorHub Sensor i2c interfaces

// i2c system bus
#define W0_SCL 27  // PA22
#define W0_SDA 26  // PA23

#define W1_SCL 39  // PA13
#define W1_SDA 28  // PA12

#define W2_SCL 13  // PA17
#define W2_SDA 11  // PA16

TwiPinPair portBackbone = TwiPinPair(W0_SCL, W0_SDA);
TwiPinPair portSensorsA = TwiPinPair(W1_SCL, W1_SDA);
TwiPinPair portSensorsB = TwiPinPair(W2_SCL, W2_SDA);

TwoWire WireSensorA(&sercom1, W1_SDA, W1_SCL);  // Sensor Port A
TwoWire WireSensorB(&sercom4, W2_SDA, W2_SCL);  // Sensor Port B

boolean reportDevicesWithAddressOn(TwoWire *wire, byte deviceAddress);
void printDeviceName(int address);


void setupWireBegins() {
  Wire.begin();
  WireSensorA.begin();
  WireSensorB.begin();
}

void setupWireSetStates() {
  portSensorsA.setPinPeripheralAltStates();
  portSensorsB.setPinPeripheralStates();
}

void reportDevicesOn(TwoWire *wire, String label) {
  byte error, address;
  int nDevices;
  Serial.print("Scanning bus ");
  Serial.println(label);
  for (address = 1; address < 127; address++) {
    if (reportDevicesWithAddressOn(wire, address)) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      printDeviceName(address);

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found on bus 1\n");
  else
    Serial.println("done\n");
}

void printDeviceName(int address) {
  // Print recognized i2c device names:
  if (address == 0x10) {
    Serial.println("ADS7138 for Touch Sensors found...");
  }
  if (address == 0x25) {
    Serial.println("Sensirion SDP800-500Pa found...");
  }
  if (address == 0x26) {
    Serial.println("Sensirion SDP800-501Pa found...");
  }
  if (address == 0x29) {
    Serial.println("DLC-L01G-U2 or VL6180 found...");
  }
  if (address == 0x40) {
    Serial.println("Sensirion SDP610-500Pa found...");
  }
  if (address == 0x50) {
    Serial.println("FRAM/EEPROM found...");
  }
  if (address == 0x51) {
    Serial.println("More memory found? Could be 1M FRAM.");
  }
  if (address == 0x7C) {
    Serial.println("RESERVED...");
  }
}

boolean reportDevicesWithAddressOn(TwoWire *wire, byte deviceAddress) {
  wire->beginTransmission(deviceAddress);
  byte error = wire->endTransmission();
  boolean result = (error == 0);
  return result;
}

boolean reportDevicesWithAddressOn(TwoWire *wire, byte deviceAddress, boolean report) {
  wire->beginTransmission(deviceAddress);
  byte error = wire->endTransmission();
  boolean result = (error == 0);
  char buffer[40];
  if (report) {
    if (result) {
      sprintf(buffer, "Device was found on address 0x%02X", deviceAddress);
    } else {
      sprintf(buffer, "No device was found on address 0x%02X", deviceAddress);
    }
    Serial.println(buffer);
  }
  return result;
}

boolean discoverDevice(byte address) {
  boolean result = reportDevicesWithAddressOn(&WireSensorA, address, true);
  if (result) {
    Serial.println("Found on port A");
    sensorPort = SENSOR_PORT_A;
  } else {
    result = reportDevicesWithAddressOn(&WireSensorB, address, true);
    if (result) {
      Serial.println("Found on port B");
      sensorPort = SENSOR_PORT_B;
    } else {
      Serial.println("Sensor not found, recheck your connections!");
      deviceState = STATE_NOT_FOUND;
    }
  }
  return result;
}