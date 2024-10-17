## Class Diagram

```mermaid
classDiagram
    class AdcHelper {
        +readSensorValues()
        -writeToRegister()
    }
    class DeviceManager {
        +discoverAndInit()
        +readSensorData()
    }
    class LedHelper {
        +update()
        +on()
        +off()
    }
    class WireHelper {
        +reportDevicesWithAddress()
        +reportDevices()
        +discoverDevice()
    }
    class PinManager {
        +setupI2CPins()
    }
    class SensorHub_Compressions {
        +setup()
        +loop()
    }
    SensorHub_Compressions --> PinManager
    SensorHub_Compressions --> LedHelper
    SensorHub_Compressions --> DeviceManager
    DeviceManager --> AdcHelper
    DeviceManager --> WireHelper
