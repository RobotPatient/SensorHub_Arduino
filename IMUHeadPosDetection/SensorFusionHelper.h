//Quaternion q(1, 0, 0, 0);  // Initial orientation quaternion
Quaternion bodyQuaternion(1, 0, 0, 0);
Quaternion headQuaternion(1, 0, 0, 0);


#define beta 0.1               // Filter gain
#define sampleRate 1000        // Sample rate in Hz
#define dt (1.0 / sampleRate)  // Time interval



const int interval = 1000 / sampleRate;



Quaternion updateSensorData(BMI270 *imu, Quaternion *q) {
  if (lastSensorDataUpdate < millis() + min_update_interval) {
    delay(25);
    lastSensorDataUpdate = millis();
  }
  imu->getSensorData(); // needs to be integrated in inertia detection
  // delay(25);
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