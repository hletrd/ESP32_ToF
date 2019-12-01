#include "SlowSoftI2CMaster.h"

SlowSoftI2CMaster::SlowSoftI2CMaster() {

}

SlowSoftI2CMaster::SlowSoftI2CMaster(uint8_t sda, uint8_t scl) {
  _sda = sda;
  _scl = scl;
  _pullup = false;
}

SlowSoftI2CMaster::SlowSoftI2CMaster(uint8_t sda, uint8_t scl, bool pullup) {
  _sda = sda;
  _scl = scl;
  _pullup = pullup;
}

bool SlowSoftI2CMaster::i2c_init(void) {
  digitalWrite(_sda, LOW);
  digitalWrite(_scl, LOW);
  setHigh(_sda);
  setHigh(_scl);
  if (digitalRead(_sda) == LOW || digitalRead(_scl) == LOW) return false;
  return true;
}

bool SlowSoftI2CMaster::i2c_start(uint8_t addr) {
  setLow(_sda);
  delayMicroseconds(DELAY);
  setLow(_scl);
  return i2c_write(addr);
}

bool SlowSoftI2CMaster::i2c_start_wait(uint8_t addr) {
  long retry = I2C_MAXWAIT;
  while (!i2c_start(addr)) {
    i2c_stop();
    if (--retry == 0) return false;
  }
  return true;
}

bool SlowSoftI2CMaster::i2c_rep_start(uint8_t addr) {
  setHigh(_sda);
  setHigh(_scl);
  delayMicroseconds(DELAY);
  return i2c_start(addr);
}

void SlowSoftI2CMaster::i2c_stop(void) {
  setLow(_sda);
  delayMicroseconds(DELAY);
  setHigh(_scl);
  delayMicroseconds(DELAY);
  setHigh(_sda);
  delayMicroseconds(DELAY);
}

bool SlowSoftI2CMaster::i2c_write(uint8_t value) {
  for (uint8_t curr = 0X80; curr != 0; curr >>= 1) {
    if (curr & value) setHigh(_sda); else  setLow(_sda); 
    setHigh(_scl);
    delayMicroseconds(DELAY);
    setLow(_scl);
  }

  setHigh(_sda);
  setHigh(_scl);
  delayMicroseconds(DELAY/2);
  uint8_t ack = digitalRead(_sda);
  setLow(_scl);
  delayMicroseconds(DELAY/2);  
  setLow(_sda);
  return ack == 0;
}

uint8_t SlowSoftI2CMaster::i2c_read(bool last) {
  uint8_t b = 0;
  setHigh(_sda);
  for (uint8_t i = 0; i < 8; i++) {
    b <<= 1;
    delayMicroseconds(DELAY);
    setHigh(_scl);
    if (digitalRead(_sda)) b |= 1;
    setLow(_scl);
  }
  if (last) setHigh(_sda); else setLow(_sda);
  setHigh(_scl);
  delayMicroseconds(DELAY/2);
  setLow(_scl);
  delayMicroseconds(DELAY/2);  
  setLow(_sda);
  return b;
}

void SlowSoftI2CMaster::setLow(uint8_t pin) {
    noInterrupts();
    if (_pullup) 
      digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
    interrupts();
}


void SlowSoftI2CMaster::setHigh(uint8_t pin) {
    noInterrupts();
    if (_pullup) 
      pinMode(pin, INPUT_PULLUP);
    else
      pinMode(pin, INPUT);
    interrupts();
}
