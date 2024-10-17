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
    DeviceManager --> AdcHelper
    DeviceManager --> WireHelper
