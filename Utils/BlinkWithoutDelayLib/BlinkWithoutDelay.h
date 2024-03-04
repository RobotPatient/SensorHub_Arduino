/*
 * J.A. Korten Feb 19, 2024
 * BlinkWithoutDelay Library
 *
 * This can be used to (re)calibrate the sensor fusion process.
 * 
 */

#ifndef BlinkWithoutDelay_h
#define BlinkWithoutDelay_h

#include <Arduino.h>

class OutputInterface {
public:
  virtual void setOutput(bool state) = 0;
};

class DigitalOutput : public OutputInterface {
private:
  int pin;

public:
  DigitalOutput(int pin)
    : pin(pin) {
    pinMode(pin, OUTPUT);
  }

  void setOutput(bool state) override {
    digitalWrite(pin, state);
  }
};

class BlinkWithoutDelay {
private:
  OutputInterface *output;
  unsigned long previousMillis;
  unsigned long interval;
  bool ledState;

public:
  void update();
  BlinkWithoutDelay(OutputInterface *output, unsigned long interval)
    : output(output), interval(interval), previousMillis(0), ledState(false) {}
};

#endif
