#include <Wire.h>  // Backbone
#include "SensorHub_Settings.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "wiring_private.h"  // pinPeripheral() function
#include "pinTools.h"
#include "WireTools.h"


// Create a new sensor object, make sure you connect the body sensor on Port B and head sensor on Port A
BMI270 imuBody;
BMI270 imuHead;

uint8_t IMUs_detected = 0;

// I2C address selection
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;  // 0x68
//uint8_t i2cAddress = BMI2_I2C_SEC_ADDR; // 0x69

const int NUM_SENSORS_CONNECTED = 1;
const int TRY_ADDRESSES = 2;
const int TRY_WIRES = 2;
const int MAX_TRIES = TRY_WIRES * TRY_ADDRESSES;

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

  while (!Serial)  // remove if working without serial.
    ;
}

bool tryToDetectIMUonAddress(BMI270 *imu) {
  /*
  00 BMI2_I2C_PRIM_ADDR portSensorsA
  01  BMI2_I2C_SEC_ADDR portSensorsA
  10 BMI2_I2C_PRIM_ADDR portSensorsB
  11  BMI2_I2C_SEC_ADDR portSensorsB
 */
  int tries = 0;
  TwoWire *wire = &WireSensorA;
  String _label = "Sensor Port A";

  while (tries < MAX_TRIES) {
    if (tries % 2 == 2) {
      if (detectIMU(imu, BMI2_I2C_SEC_ADDR, wire, _label)) {
        IMUs_detected++;
        return true;
      }
    } else {
      if (detectIMU(imu, BMI2_I2C_PRIM_ADDR, wire, _label)) {
        IMUs_detected++;
        return true;
      }
    }
    tries++;
    if (tries >= 2) {
      wire = &WireSensorB;
      _label = "Sensor Port B";
    }
    if (tries >= 4) {
      wire = &Wire;
      _label = "Main Port";
    }
  }
  return false;
}


bool detectIMU(BMI270 *imu, uint8_t addr, TwoWire *wire, String label) {
  if (reportDevicesWithAddressOn(wire, addr)) {
    if (imu->beginI2C(addr, *wire) == BMI2_OK) {
      Serial.print("Bosch IMU found on i2c ");
      Serial.println(label);
      return true;
    }
  } else {
    return false;
  }
  return false;
}

void setup() {
  setupSensorHub();

  if (NUM_SENSORS_CONNECTED == 1) {
    tryToDetectIMUonAddress(&imuHead);
  }
  if (NUM_SENSORS_CONNECTED == 2) {
    tryToDetectIMUonAddress(&imuBody);
    tryToDetectIMUonAddress(&imuHead);
  }

  if (IMUs_detected > 0) {
    delay(1000);
  } else {
    Serial.println("Check sensor connection and/or reset board...");
    if (NUM_SENSORS_CONNECTED == 0) {
      Serial.println("Do not forget to increase NUM_SENSORS_CONNECTED...");
    }
    while (1)
      ;
  }
}

void checkSensor(BMI270 *imu) {

  // Print 50x per second
  delay(20);
  // Get measurements from the sensor. This must be called before accessing
  // the sensor data, otherwise it will never update
  imu->getSensorData();

  // Print acceleration data
  Serial.print("Acceleration in g's");
  Serial.print("\t");
  Serial.print("X: ");
  Serial.print(imu->data.accelX, 3);
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(imu->data.accelY, 3);
  Serial.print("\t");
  Serial.print("Z: ");
  Serial.print(imu->data.accelZ, 3);

  Serial.print("\t");

  // Print rotation data
  Serial.print("Rotation in deg/sec");
  Serial.print("\t");
  Serial.print("X: ");
  Serial.print(imu->data.gyroX, 3);
  Serial.print("\t");
  Serial.print("Y: ");
  Serial.print(imu->data.gyroY, 3);
  Serial.print("\t");
  Serial.print("Z: ");
  Serial.println(imu->data.gyroZ, 3);
}

void loop() {
  checkSensor(&imuHead);
  if (NUM_SENSORS_CONNECTED > 1) {
    checkSensor(&imuBody);
  }
}
