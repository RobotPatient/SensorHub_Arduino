# Thorax Touch Sensing

This project implements touch sensing for a thorax (chest) model using an ADS7138 16-bit ADC and an Arduino microcontroller. It is designed for [your application/project name or context].

This version adds the Compressions sensor (ToF to detect compression depths and recoils)

## Hardware

*   Arduino microcontroller (e.g., Adafruit Feather M0 Express)
*   ADS7138 16-bit ADC
*   [List any other hardware components, e.g., LEDs, resistors, connecting wires]
*   Thorax model with touch sensors connected to the ADC

## Software

*   Arduino IDE
*   Libraries:
    *   Wire (for I2C communication)
    *   [List any other custom libraries]

## Code Structure

The code is structured into the following files:

*   `SensorHub_Compressions.ino`: Main Arduino sketch file.
*   `adcHelper.h`: Functions to interact with the ADS7138 ADC.
*   `ads7138_definitions.h`: Register definitions for the ADS7138.
*   `DeviceManager.h`: Manages device discovery and data reading.
*   `deviceHelper.h`: Enums for device state, sensor port, and sensor order.
*   `ledHelper.h`: Functions to control an LED (heartbeat indicator).
*   `PinHelper.h`: Functions for printing pin information (debugging).
*   `PinManager.h`: Functions for configuring I2C pins.
*   `serialHelper.h`: Functions for serial communication setup.
*   `SensorHub_Settings.h`: Project settings (e.g., I2C address).
*   `pinTools.h`: Helper class for managing I2C pin pairs.
*   `wireHelper.h`: Helper class for I2C communication.

## Wiring Diagram

```mermaid
graph LR
    subgraph "Arduino"
        A[I2C Pins] --> B(Wire Library)
        C[Digital Pin] --> D(LedHelper)
    end
    B --> E[WireHelper]
    E --> F[DeviceManager]
    F --> G{ADS7138 ADC}
    G -- "Sensor Data" --> F
    F --> E
    E --> A
    D --> H(LED)
