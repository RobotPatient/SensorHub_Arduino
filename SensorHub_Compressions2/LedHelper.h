#ifndef LEDHELPER_H
#define LEDHELPER_H

#include <Arduino.h>

class LedHelper {
public:
  LedHelper(uint32_t ledPin, unsigned long onInterval = 50, unsigned long offInterval = 500) :
    ledPin_(ledPin),
    onInterval_(onInterval),
    offInterval_(offInterval) 
  {
    pinMode(ledPin_, OUTPUT);
    digitalWrite(ledPin_, HIGH);
    previousMillis_ = millis(); 
  }

  void update() {
    unsigned long currentMillis = millis();

    if (ledState_ == HIGH && (currentMillis - previousMillis_ >= onInterval_)) {
      ledState_ = LOW;
      digitalWrite(ledPin_, ledState_);
      previousMillis_ = currentMillis; 
    } else if (ledState_ == LOW && (currentMillis - previousMillis_ >= offInterval_)) {
      ledState_ = HIGH;
      digitalWrite(ledPin_, ledState_);
      previousMillis_ = currentMillis;
    }
  }

  void on() {
    digitalWrite(ledPin_, HIGH);
    ledState_ = HIGH;
  }

  void off() {
    digitalWrite(ledPin_, LOW);
    ledState_ = LOW;
  }

private:
  const uint32_t ledPin_;
  unsigned long onInterval_;
  unsigned long offInterval_;
  unsigned long previousMillis_;
  int ledState_ = HIGH; 
};

#endif // LEDHELPER_H