#define ledHb 14

int hbLedState = LOW;                // ledState used to set the LED
unsigned long previousMillisHB = 0;  // will store last time LED was updated
const long intervalHB_High = 500;    // interval at which to blink (milliseconds)
const long intervalHB_Low = 50;     // interval at which to blink (milliseconds)

void heartBeatOff() {
  digitalWrite(ledHb, LOW);
}

void heartBeatOn() {
  digitalWrite(ledHb, HIGH);
}

void setupLEDs() {
  pinMode(ledHb, OUTPUT);
  heartBeatOn();
}

void updateHeartBeat() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillisHB >= intervalHB_High) {
    digitalWrite(ledHb, HIGH);
    if (currentMillis - previousMillisHB >= intervalHB_High + intervalHB_Low) {
      digitalWrite(ledHb, LOW);
      previousMillisHB = currentMillis;
    }
  }
}