#include "deviceHelper.h"
#include "ads7138_definitions.h"



void readValues(TwoWire *wire, byte address);

void readSensorValues(SensorPort port, byte deviceAddress) {
  if (port == SENSOR_PORT_A) {
    readValues(&WireSensorA, deviceAddress);
  }
  if (port == SENSOR_PORT_B) {
    readValues(&WireSensorB, deviceAddress);
  }
}

TwoWire *wireInterfaceFromPort(SensorPort port) {
  if (port == SENSOR_PORT_A) {
    return &WireSensorA;
  } else {
    return &WireSensorB;
  }
}

void readFromRegister(TwoWire *wire, byte registerAddress, byte address) {
  wire->beginTransmission(address);
  wire->write(SINGLE_READ);

  wire->write(registerAddress);
  wire->endTransmission();

  wire->requestFrom(address, 1);

  if (wire->available() <= 1) {
    int val = wire->read();
    Serial.print("Register ");
    Serial.print(registerAddress);
    Serial.print(" ");
    Serial.println(val, BIN);
  }

  /* 
      To read a single register from the device, the I2C master must provide an I2C command with three frames to set the register address for reading data. 
      Table 9 lists the opcodes for different commands. 
      
      After this command is provided, the I2C master must provide another I2C frame (as shown in Figure 33) 
      containing the device address and the read bit. 
      
      After this frame, the device provides the register data. 
      
      The device provides the same register data even if the host provides more clocks. 
      To end the register read command, the master must provide a STOP or a RESTART condition in the I2C frame.
  */
}


void writeToRegister(TwoWire *wire, byte registerAddress, byte address, byte command) {
  // 8.5.2.1 Single Register Write
  wire->beginTransmission(address);
  wire->write(SINGLE_WRITE);
  wire->write(registerAddress);
  wire->write(command);
  wire->endTransmission();
}

void resetStatus(byte ADS7138_Address) {
  TwoWire *wireInterface = wireInterfaceFromPort(sensorPort);
  writeToRegister(wireInterface, SYSTEM_STATUS, ADS7138_Address, RESET_SYS_STAT);
}

void readChannel(TwoWire *wire, byte address, byte channel) {
  // 8.6.12 CHANNEL_SEL Register (Address = 0x11) [reset = 0x0]
  // 0b = AIN0
  // 1b = AIN1
  // 10b = AIN2
  // 11b = AIN3
  // 100b = AIN4
  // 101b = AIN5
  // 110b = AIN6
  // 111b = AIN7

  writeToRegister(wire, CHANNEL_SEL, address, channel);
  delay(10);
  wire->requestFrom(address, 2);

  if (wire->available() <= 2) {
    int val = wire->read();           // Receive high byte
    val = (val << 8) | wire->read();  // Receive low byte and combine with high byte
    
    Serial.print("Channel ");
    Serial.print(channel);
    Serial.print(": ");
    Serial.print(val);
    Serial.println();
  }
}

void readValues(TwoWire *wire, byte address) {
  int sensorData[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

  for (int channel = 0; channel < 8; ++channel) {
    // Start transmission to ADS7138
    writeToRegister(wire, CHANNEL_SEL, address, channel);
    delay(1);  // Delay for conversion to complete

    // Request the conversion result
    wire->requestFrom(address, 2);

    if (wire->available() <= 2) {
      int val = wire->read();           // Receive high byte
      val = (val << 8) | wire->read();  // Receive low byte and combine with high byte
      sensorData[channel] = val;


      // Convert to appropriate value based on your sensor and setup
      // E.g., if reading a voltage with a gain of 1, you might do:
      // float voltage = val * 3.3 / 1024;

      // Print the result
    }

    delay(1);  // Delay before next reading
  }

  for (int index = 0; index < 8; ++index) {
    Serial.print(sensorData[index]);
    Serial.print(" ");
  }
  Serial.println();
}