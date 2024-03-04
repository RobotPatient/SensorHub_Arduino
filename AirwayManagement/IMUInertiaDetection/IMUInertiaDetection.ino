/*
*. J.A. Korten Feb 19, 2024
* Basic implementation for statis / movement detection of the accelerometers.
*
* This can be used to (re)calibrate the sensor fusion process.
* 
*/

#include <Wire.h>  // Backbone
#include "SensorHub_Settings.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "pinTools.h"
#include "WireTools.h"
#include "SensorTools.h"
#include "Quaternion.h"
#include "Vector3D.h"
#include "IMUInertiaController.h"
#include "IMUInertiaHelper.h"

// Create a new sensor object, make sure you connect the body sensor on Port B and head sensor on Port A
BMI270 imuBody;
BMI270 imuHead;

uint8_t IMUs_detected = 0;
const int NUM_SENSORS_CONNECTED = 2;
int num_sensors_detected = 0;

TwoWire WireBackbone(&sercom3, W0_SDA, W0_SCL);  // Main
TwoWire WireSensorA(&sercom1, W1_SDA, W1_SCL);   // Sensor A
TwoWire WireSensorB(&sercom4, W2_SDA, W2_SCL);   // Sensor B


const Vector3D thresholds(0.055, 0.05, 0.05);
Vector3D thBaseForHeadSensor(0.0, 0.0, 0.0);
Vector3D thBaseForBodySensor(0.0, 0.0, 0.0);

IMUInertiaHelper stationaryDetectorHead(thresholds);
IMUInertiaHelper stationaryDetectorBody(thresholds);


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

  delay(1500);  // note: this is critical.
  updateSensor3DVector(&imuBody, &stationaryDetectorBody);
  updateSensor3DVector(&imuHead, &stationaryDetectorHead);
}

void loop() {
  updateSensor(&imuBody, &stationaryDetectorBody, "body");
  updateSensor(&imuHead, &stationaryDetectorHead, "head");
  delay(50);
}
