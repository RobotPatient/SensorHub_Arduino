/*
 * J.A. Korten Feb 19, 2024
 * BlinkWithoutDelay Library
 *
 * This can be used to (re)calibrate the sensor fusion process.
 * 
 */

#include "BlinkWithoutDelay.h"


void BlinkWithoutDelay::update() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ledState = !ledState;
    output->setOutput(ledState);
  }
}