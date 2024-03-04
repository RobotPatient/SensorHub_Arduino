/*
*. J.A. Korten Feb 19, 2024
* Basic implementation for statis / movement detection of the accelerometers.
*
* Head positioning detection which includes recalibration when sensors are
* 
*/

#include <Wire.h>  // Backbone
#include "SparkFun_BMI270_Arduino_Library.h"
#include "SensorHub_Settings.h"
#include "SensorFusionHelper.h"
#include "pinTools.h"
#include "WireTools.h"
#include "SensorTools.h"
#include "Quaternion.h"
#include "Vector3D.h"
#include "IMUInertiaController.h"
#include "IMUInertiaHelper.h"
#include <BlinkWithoutDelay.h>


const int HBLedPin = 14;  // Change this to the desired pin

DigitalOutput hbLEDOutput(HBLedPin);
BlinkWithoutDelay hbLEDBlinker(&hbLEDOutput, 500);  // Blink every half a second

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
IMUInertiaHelper stationaryDetectorHead(thresholds);
IMUInertiaHelper stationaryDetectorBody(thresholds);

void printQuaternionsAsEulerAngles(Quaternion qBody, Quaternion qHead); // prototype

const Sensor_fusion_method fusion_method = SF_Mahony;  // Alt: SF_Madgwick, but seems that our Madgwick implementation has still issues (or is just not performing as well as Mahony (understatement))
SensorFusionHelper sensorFusionHelperHead(fusion_method);
SensorFusionHelper sensorFusionHelperBody(fusion_method);

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

  // Note: When using two IMU's on the same port please solder the jumper to set BMI2_I2C_PRIM_ADDR and BMI2_I2C_SEC_ADDR
  if (detectIMU(&imuBody, BMI2_I2C_PRIM_ADDR, &WireSensorB, "WireSensorB")) num_sensors_detected++;
  if (detectIMU(&imuHead, BMI2_I2C_SEC_ADDR, &WireSensorB, "WireSensorB")) num_sensors_detected++;

  if (NUM_SENSORS_CONNECTED == num_sensors_detected) {
    Serial.println("Detection complete...");
  } else {
    Serial.println("Detection incomplete recheck settings / sensors / cables ...");
    while (!Serial)  // remove if working without serial.
      ;
  }

  sensorFusionHelperHead.printAlgorithm();

  delay(1500);  // note: this is critical.
  updateSensor3DVector(&imuBody, &stationaryDetectorBody, fusion_method);
  updateSensor3DVector(&imuHead, &stationaryDetectorHead, fusion_method);
}

// Note: In case you get NaN issues it might be your sensor is not detected 
// ToDo: resolve this in the library (i.e. ignore the particular sensor).

void loop() {
  hbLEDBlinker.update();

  Quaternion qBody = updateSensor(&imuBody, &stationaryDetectorBody, &sensorFusionHelperBody, "body");
  Quaternion qHead = updateSensor(&imuHead, &stationaryDetectorHead, &sensorFusionHelperHead, "head");
  printQuaternionsAsEulerAngles(qBody, qHead);
}

void printQuaternionsAsEulerAngles(Quaternion qBody, Quaternion qHead) {
  float roll, pitch, yaw;
  qBody.toEulerAngles(roll, pitch, yaw);
  qBody.printEulerAngles(roll, pitch, yaw, "body");  // just for debugging!
  Serial.print("  ");

  qHead.toEulerAngles(roll, pitch, yaw);
  qBody.printEulerAngles(roll, pitch, yaw, "head");  // just for debugging!

  Serial.print("  ");
  Quaternion qBodyHead = qBody - qHead;
  qBodyHead.toEulerAngles(roll, pitch, yaw);
  qBodyHead.printEulerAngles(roll, pitch, yaw, "head-body");  // just for debugging!

  Serial.println();
}