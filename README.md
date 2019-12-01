## Multiple VL53L0X ToF sensor for ESP32
* Simple example to drive multiple ToF sensors simultaneously from ESP32.
* Supports VL53L0X.

### Hardwares used
* ESP32 OLED board
* VL53L0X ToF sensor

### Libraries


### How to compile & upload the sketch
* Install [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md) board on the Arduino IDE.
* Set the board type to LOLIN D32.
* Connect the ESP32 OLED board to PC.
* Upload the sketch.
  
### Files
* ```example.ino```
  * Code based on Arduino core for ESP32.
* ```SlowSoftI2CMaster.cpp```, ```SlowSoftI2CMaster.h```
  * Customized software I2C library for ESP32.
  * Object-Oriented to support multiple I2C devices at once.
* ```SoftWire.h```
  * A wrapper to use ```SlowSoftI2CMaster``` as Wire.h.
* ```VL53L0X.cpp```, ```VL53L0X.h```
  * Customized VL53L0X library to support software I2C and to support multiple sensors.