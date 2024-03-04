#ifndef SENSOR_TOOLS_HPP
#define SENSOR_TOOLS_HPP

// I2C address selection:
// uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;  // 0x68
// uint8_t i2cAddress = BMI2_I2C_SEC_ADDR; // 0x69


bool detectIMU(BMI270 *imu, uint8_t addr, TwoWire *wire, String label) {
  if (reportDevicesWithAddressOn(wire, addr)) {
    if (imu->beginI2C(addr, *wire) == BMI2_OK) {
      Serial.print("Bosch IMU found on i2c (0x");
      Serial.print(addr, HEX);
      Serial.print(") ");
      Serial.println(label);
      return true;
    }
  } else {
    return false;
  }
  return false;
}

#endif /* SENSOR_TOOLS_HPP */