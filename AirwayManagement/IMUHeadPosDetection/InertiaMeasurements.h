/*
  Library for recording the inertia status with the IMU Sensor(s)
  J.A. Korten Feb 21, 2024

  .h file
*/

#ifndef INERTIA_RECORD_CIRCULAR_BUFFER_H
#define INERTIA_RECORD_CIRCULAR_BUFFER_H

struct InertiaRecord {
    bool data;
    unsigned long timestamp;
    int counter;
};

class InertiaMeasurements {
public:
    InertiaMeasurements();
    void addRecord(bool data, unsigned long timestamp, int counter);
    InertiaRecord* getRecords();
    int size();

private:
    static const int BUFFER_SIZE = 10;
    InertiaRecord buffer[BUFFER_SIZE];
    int head;
    int tail;
    void advanceTail();
};

#endif /* INERTIA_RECORD_CIRCULAR_BUFFER_H */
