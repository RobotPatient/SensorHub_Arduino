#ifndef DEVICE_HELPER_H
#define DEVICE_HELPER_H

enum State {
  STATE_UNKNOWN,
  STATE_DISCOVERED,
  STATE_NOT_FOUND,
  STATE_WORKING
};

enum SensorPort {
  PORT_UNKNOWN,
  SENSOR_PORT_A,
  SENSOR_PORT_B
};

enum QRESensors {
  S1,
  S2,
  S3,
  S4,
  S5,
  S6,
  S7,
  S8
};

QRESensors sensorOrder[] = {S6, S5, S3, S8, S4, S1, S2, S7};


#endif