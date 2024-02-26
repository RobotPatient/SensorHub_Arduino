/*
  Small helper for dealing with the IMU Sensor(s)
  J.A. Korten Feb 21, 2024
*/

#ifndef IMUHelper_h
#define IMUHelper_h

#define min_update_interval 25

bool isDelayNeeded() {
  static unsigned long lastSensorDataUpdate = 0;
  bool delayNeeded = (lastSensorDataUpdate < millis() + min_update_interval);
  if (!delayNeeded) {
    lastSensorDataUpdate = millis();
  }
  return delayNeeded;
}

void checkIfDelayIsNeeded() {
  if (isDelayNeeded) {
    delay(25);
  }
}

void updateSensorData(BMI270 *imu) {
  checkIfDelayIsNeeded();
  imu->getSensorData();
}

#endif /* IMUHelper */