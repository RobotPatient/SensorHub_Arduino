/*
 * J.A. Korten Feb 19, 2024
 * BlinkWithoutDelay Library
 *
 * This can be used to (re)calibrate the sensor fusion process.
 * 
 */
 
 #include "BlinkWithoutDelay.h"

const int customLedPin = 14; // Change this to the desired pin

DigitalOutput customOutput(customLedPin);
BlinkWithoutDelay customBlink(&customOutput, 1000); // Blink every 1 second

void setup() {
    // No need to initialize anything in setup() for this library
}

void loop() {
    customBlink.update(); // Call update() method in loop()
}
