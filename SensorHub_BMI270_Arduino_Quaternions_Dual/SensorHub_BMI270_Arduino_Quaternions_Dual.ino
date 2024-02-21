/*
*. J.A. Korten Feb 19, 2024
* Basic implementation of head positioning sensor with sensor Fusion (gyro + accelerometer)
* using a Quaternion-based Sensor Fusion approach.
* 
* This version includes readings of the body sensor as a reference for the head positioning sensor.
*
* Note: while this code works in essence, it suffers from drift so is not a definitive solution. 
*/

#include <Wire.h>  // Backbone
#include "SensorHub_Settings.h"
#include "SparkFun_BMI270_Arduino_Library.h"
#include "pinTools.h"
#include "WireTools.h"
#include "SensorTools.h"
#include "Quaternion.h"
#include "Vector3D.h"


// Create a new sensor object, make sure you connect the body sensor on Port B and head sensor on Port A
BMI270 imuBody;
BMI270 imuHead;

uint8_t IMUs_detected = 0;
const int NUM_SENSORS_CONNECTED = 2;
int num_sensors_detected = 0;

TwoWire WireBackbone(&sercom3, W0_SDA, W0_SCL);  // Main
TwoWire WireSensorA(&sercom1, W1_SDA, W1_SCL);   // Sensor A
TwoWire WireSensorB(&sercom4, W2_SDA, W2_SCL);   // Sensor B

//Quaternion q(1, 0, 0, 0);  // Initial orientation quaternion
Quaternion bodyQuaternion(1, 0, 0, 0);
Quaternion headQuaternion(1, 0, 0, 0);

#define beta 0.1               // Filter gain
#define sampleRate 1000        // Sample rate in Hz
#define dt (1.0 / sampleRate)  // Time interval

bool _sensor_initialized = false;

const int interval = 1000 / sampleRate;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated

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

Quaternion updateSensor(BMI270 *imu, Quaternion *q) {

  imu->getSensorData();
  //delay(25);
  //delay(20);

  Vector3D accel = Vector3D(imu->data.accelX, imu->data.accelY, imu->data.accelZ);
  Vector3D gyro = Vector3D(imu->data.gyroX, imu->data.gyroY, imu->data.gyroZ);

  /* To check inside the objects (i.e. if the sensor gave any output): */
  // accel.printToSerial();
  // gyro.printToSerial();

  Quaternion deltaQ = Quaternion(1.0, gyro.x * dt / 2.0, gyro.y * dt / 2.0, gyro.z * dt / 2.0);
  *q = *q * deltaQ;

  // Normalize quaternion
  q->normalize();

  // Update orientation with accelerometer data
  Vector3D gravity = Vector3D(0, 0, -1);                          // Define gravity vector in sensor frame
  Vector3D estimatedGravity = q->rotate(gravity);                 // Rotate gravity vector using current orientation
  Vector3D error = accel - estimatedGravity;                      // Calculate error between measured and estimated gravity vectors
  Vector3D correction = error * (beta / sampleRate);              // Apply correction
  *q += Quaternion(0, correction.x, correction.y, correction.z);  // Update quaternion


  // Normalize quaternion again
  q->normalize();

  return *q;
}

void printQuaternion(Quaternion *q) {
   q->printToSerial();
   //eulerAngleFrom(q);
}

void eulerAngleFrom(Quaternion *q) {
    // Convert quaternion to Euler angles (optional)
  float roll, pitch, yaw;
  q->toEulerAngles(roll, pitch, yaw);

  // Output orientation (for example, print to serial monitor)

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Yaw: ");
  Serial.println(yaw);
  
}

void loop() {
  static Quaternion baseQuaternion(1, 0, 0, 0);
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) { // Delay to maintain sample rate
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    bodyQuaternion = updateSensor(&imuBody, &bodyQuaternion);
    headQuaternion = updateSensor(&imuHead, &headQuaternion);

    //Quaternion q1 =  headQuaternion - bodyQuaternion;
    printQuaternion(&headQuaternion);
    printQuaternion(&bodyQuaternion);
    
  }
}
