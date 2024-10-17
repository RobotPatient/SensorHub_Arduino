void setupSerial() {
  delay(1500);
  Serial.begin(115200);
  delay(1500);
}

void waitForSerial() {
  while (!Serial)
    ;

  Serial.println("Ready...");
  delay(1500);
}