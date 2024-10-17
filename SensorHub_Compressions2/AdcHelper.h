
#ifndef ADCHELPER_H 
#define ADCHELPER_H

#include <Wire.h>
#include "ads7138_definitions.h"

class AdcHelper {
public:
    AdcHelper(TwoWire &wire, byte address) :
        wire_(wire),
        address_(address)
    {}

    void readSensorValues() {
        int sensorData[8] = {0, 0, 0, 0, 0, 0, 0, 0};

        for (int channel = 0; channel < 8; ++channel) {
            writeToRegister(CHANNEL_SEL, channel);
            delay(1); // Delay for conversion to complete

            wire_.requestFrom(address_, 2);
            if (wire_.available() >= 2) {
                int val = wire_.read() << 8 | wire_.read();
                sensorData[channel] = val;
            }
            delay(1); // Delay before next reading
        }

        for (int index = 0; index < 8; ++index) {
            Serial.print(sensorData[index]);
            Serial.print(" ");
        }
        Serial.println();
    }

private:
    void writeToRegister(byte registerAddress, byte command) {
        wire_.beginTransmission(address_);
        wire_.write(SINGLE_WRITE);
        wire_.write(registerAddress);
        wire_.write(command);
        wire_.endTransmission();
    }

    TwoWire &wire_;
    const byte address_;
};

#endif // ADCHELPER_H

