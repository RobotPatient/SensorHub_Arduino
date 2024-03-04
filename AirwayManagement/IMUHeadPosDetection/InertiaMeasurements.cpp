/*
  Library for recording the inertia status with the IMU Sensor(s)
  J.A. Korten Feb 21, 2024

  .cpp file
*/

#include "InertiaMeasurements.h"

InertiaMeasurements::InertiaMeasurements() : head(0), tail(0) {}

void InertiaMeasurements::addRecord(bool data, unsigned long timestamp, int counter) {
    buffer[tail].data = data;
    buffer[tail].timestamp = timestamp;
    buffer[tail].counter = counter;
    advanceTail();
}

InertiaRecord* InertiaMeasurements::getRecords() {
    return buffer;
}

int InertiaMeasurements::size() {
    return BUFFER_SIZE;
}

void InertiaMeasurements::advanceTail() {
    tail = (tail + 1) % BUFFER_SIZE;
    if (tail == head) {
        head = (head + 1) % BUFFER_SIZE;
    }
}
