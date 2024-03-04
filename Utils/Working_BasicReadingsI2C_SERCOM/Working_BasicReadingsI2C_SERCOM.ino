#include <Wire.h>  // Backbone
#include "SensorHub_Settings.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "wiring_private.h"  // pinPeripheral() function
#include "pinTools.h"
#include "WireTools.h"


// Create a new sensor object
BMI270 imu;

// I2C address selection
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;  // 0x68
//uint8_t i2cAddress = BMI2_I2C_SEC_ADDR; // 0x69


TwoWire WireBackbone(&sercom3, W0_SDA, W0_SCL);  // Main
TwoWire WireSensorA(&sercom1, W1_SDA, W1_SCL);   // Sensor A
TwoWire WireSensorB(&sercom4, W2_SDA, W2_SCL);   // Sensor B

void setupSensorHub() {
  delay(1500);
  Serial.begin(115200);
  delay(1500);

  Wire.begin();
  WireBackbone.begin();
  WireSensorA.begin();
  WireSensorB.begin();

  pinMode(ledHb, OUTPUT);
  digitalWrite(ledHb, HIGH);

  portBackbone.setPinPeripheralStates();
  portSensorsA.setPinPeripheralAltStates();
  portSensorsB.setPinPeripheralStates();

  while (!Serial) // remove if working without serial.
    ;
}

bool detectIMU() {
  // We might have multiple IMU's, for now we assume one.
  if (reportDevicesWithAddressOn(&WireSensorA, BMI2_I2C_PRIM_ADDR)) {
    if (imu.beginI2C(i2cAddress, WireSensorA) == BMI2_OK) {
      Serial.println("Bosch IMU found on Wire Sensor Port A...");
      return true;
    }
  }
  if (reportDevicesWithAddressOn(&WireSensorB, BMI2_I2C_PRIM_ADDR)) {
    if (imu.beginI2C(i2cAddress, WireSensorB) == BMI2_OK) {
      Serial.println("Bosch IMU found on Wire Sensor Port B...");
      return true;
    }
  }
  return false;
}

void setup() {


  setupSensorHub();
  if (detectIMU()) {
    delay(1000);
  } else {
    Serial.println("Check sensor connection and/or reset board...");
    while (1)
      ;
  }
}

void loop() {
  // Get measurements from the sensor. This must be called before accessing
  // the sensor data, otherwise it will never update
  imu.getSensorData();

  // Print acceleration data
  Serial.print("Acceleration in g's");
  Serial.print("\t");
  Serial.print("X: ");
  Serial.print(imu.data.accelX, 3);
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(imu.data.accelY, 3);
  Serial.print("\t");
  Serial.print("Z: ");
  Serial.print(imu.data.accelZ, 3);

  Serial.print("\t");

  // Print rotation data
  Serial.print("Rotation in deg/sec");
  Serial.print("\t");
  Serial.print("X: ");
  Serial.print(imu.data.gyroX, 3);
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(imu.data.gyroY, 3);
  Serial.print("\t");
  Serial.print("Z: ");
  Serial.println(imu.data.gyroZ, 3);

  // Print 50x per second
  delay(20);
}