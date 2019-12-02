## Multiple VL53L0X ToF sensor for ESP32
* Simple example to drive multiple ToF sensors simultaneously from ESP32.
* Supports VL53L0X.

### Hardwares used
* ESP32 OLED board
* VL53L0X ToF sensor

### Libraries used
* SoftWire.h from [SoftI2CMaster](https://github.com/felias-fogg/SoftI2CMaster)
  * GPL v3.0
* [SlowSoftI2CMaster](https://github.com/felias-fogg/SlowSoftI2CMaster)
  * LGPL
* [Pololu VL53L0X](https://github.com/pololu/vl53l0x-arduino)
  * MIT License

### How to compile & upload the sketch
* Install [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md) board on the Arduino IDE.
* Set the board type to LOLIN D32.
* Connect the ESP32 OLED board to PC.
* Upload the sketch.
  
### Files
* ```example.ino```
  * An example code based on Arduino core for ESP32.
* ```SlowSoftI2CMaster.cpp```, ```SlowSoftI2CMaster.h```
  * Software I2C library to support I2C communications on non-HW-I2C pins of ESP32.
  * Used Object-Oriented library to support multiple I2C devices simultaneously.
* ```SoftWire.h```
  * A wrapper to use ```SlowSoftI2CMaster``` like Wire.h, with customizations to support multiple instances for each I2C connections.
* ```VL53L0X.cpp```, ```VL53L0X.h```
  * Customized VL53L0X library to support software I2C and to support multiple sensors.
