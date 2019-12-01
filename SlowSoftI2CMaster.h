#ifndef SLOW_SOFT_I2C_MASTER_H
#define SLOW_SOFT_I2C_MASTER_H

#include <Arduino.h>
#include <inttypes.h>

#define I2C_READ 1
#define I2C_WRITE 0
#define DELAY 4
#define BUFFER_LENGTH 32
#define I2C_MAXWAIT 5000

class SlowSoftI2CMaster {
 public:
  SlowSoftI2CMaster();
  SlowSoftI2CMaster(uint8_t sda, uint8_t scl);
  SlowSoftI2CMaster(uint8_t sda, uint8_t scl, bool internal_pullup);
  bool i2c_init(void);
  bool i2c_start(uint8_t addr);
  bool i2c_start_wait(uint8_t addr);
  bool i2c_rep_start(uint8_t addr);
  void i2c_stop(void);
  bool i2c_write(uint8_t value);
  uint8_t i2c_read(bool last);
  bool error;
  
 private:
  void setHigh(uint8_t pin);
  void setLow(uint8_t pin);
  uint8_t _sda;
  uint8_t _scl;
  bool _pullup;
};

#endif