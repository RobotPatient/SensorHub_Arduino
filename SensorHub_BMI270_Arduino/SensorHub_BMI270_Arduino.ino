/*
*
*
*
*/


#include <Wire.h>  // Backbone
#include "SensorHub_Settings.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "wiring_private.h"  // pinPeripheral() function
#include "pinTools.h"
#include "WireTools.h"
#include "SensorTools.h"


// Create a new sensor object, make sure you connect the body sensor on Port B and head sensor on Port A
BMI270 imuBody;
BMI270 imuHead;

uint8_t IMUs_detected = 0;
const int NUM_SENSORS_CONNECTED = 2;
int num_sensors_detected = 0;


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
}



void setup() {
  setupSensorHub();

  if (detectIMU(&imuBody, BMI2_I2C_PRIM_ADDR, &WireSensorA, "WireSensorA")) num_sensors_detected++;
  if (detectIMU(&imuHead, BMI2_I2C_SEC_ADDR, &WireSensorB, "WireSensorB")) num_sensors_detected++;

  if (NUM_SENSORS_CONNECTED == num_sensors_detected) {
    Serial.println("Detection complete...");
  } else {
    Serial.println("Detection incomplete recheck settings / sensors / cables ...");
    while (!Serial)  // remove if working without serial.
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
  //checkSensor(&imuHead);
  if (NUM_SENSORS_CONNECTED > 1) {
    //checkSensor(&imuBody);
  }
}
