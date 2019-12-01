#ifndef _SoftWire_h
#define _SoftWire_h

#include "SlowSoftI2CMaster.h"
#include <inttypes.h>
#include "Stream.h"

#define BUFFER_LENGTH 32

#define WIRE_HAS_END 1

class SoftWire : public Stream
{
private:
  uint8_t rxBuffer[BUFFER_LENGTH];
  uint8_t rxBufferIndex;
  uint8_t rxBufferLength;
  uint8_t transmitting;
  uint8_t error;

  SlowSoftI2CMaster si;
public:
  SoftWire() {

  }
  
  SoftWire(uint8_t SCL, uint8_t SDA) {
    si = SlowSoftI2CMaster(SDA, SCL, true);
  }

  void begin(void) {
    rxBufferIndex = 0;
    rxBufferLength = 0;
    error = 0;
    transmitting = false;
    
    si.i2c_init();
  }
  
  void end(void) {
  }

  void setClock(uint32_t _) {
  }

  void beginTransmission(uint8_t address) {
    if (transmitting) {
      error = (si.i2c_rep_start((address<<1)|I2C_WRITE) ? 0 : 2);
    } else {
      error = (si.i2c_start((address<<1)|I2C_WRITE) ? 0 : 2);
    }

    transmitting = 1;
  }
  
  void beginTransmission(int address) {
    beginTransmission((uint8_t)address);
  }

  uint8_t endTransmission(uint8_t sendStop)
  {
    uint8_t transError = error;
    if (sendStop) {
      si.i2c_stop();
      transmitting = 0;
    }
    error = 0;
    return transError;
  }

  uint8_t endTransmission(void)
  {
    return endTransmission(true);
  }

  size_t write(uint8_t data) {
    if (si.i2c_write(data)) {
      return 1;
    } else {
      if (error == 0) error = 3;
      return 0;
    }
  }

  size_t write(const uint8_t *data, size_t quantity) {
    size_t trans = 0;
    for(size_t i = 0; i < quantity; ++i){
      trans += write(data[i]);
    }
    return trans;
  }

  uint8_t requestFrom(uint8_t address, uint8_t quantity,
		      uint32_t iaddress, uint8_t isize, uint8_t sendStop) {
    uint8_t localerror = 0;
    if (isize > 0) {
      beginTransmission(address);
      if (isize > 3){
	isize = 3;
      }
      while (isize-- > 0)
	write((uint8_t)(iaddress >> (isize*8)));
      endTransmission(false);
    }
    if(quantity > BUFFER_LENGTH){
      quantity = BUFFER_LENGTH;
    }
    localerror = !si.i2c_rep_start((address<<1) | I2C_READ);
    if (error == 0 && localerror) error = 2;
    for (uint8_t cnt=0; cnt < quantity; cnt++) 
      rxBuffer[cnt] = si.i2c_read(cnt == quantity-1);
    rxBufferIndex = 0;
    rxBufferLength = quantity;
    if (sendStop) {
      transmitting = 0;
      si.i2c_stop();
    }
    return quantity;
  }
  
  uint8_t requestFrom(uint8_t address, uint8_t quantity, uint8_t sendStop) {
	return requestFrom((uint8_t)address, (uint8_t)quantity, (uint32_t)0, (uint8_t)0, (uint8_t)sendStop);
  }

  uint8_t requestFrom(int address, int quantity, int sendStop) {
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)sendStop);
  }


  uint8_t requestFrom(uint8_t address, uint8_t quantity) {
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
  }

  uint8_t requestFrom(int address, int quantity) {
    return requestFrom((uint8_t)address, (uint8_t)quantity, (uint8_t)true);
  }

  int available(void) {
    return rxBufferLength - rxBufferIndex;
  }

  int read(void) {
    int value = -1;
    if(rxBufferIndex < rxBufferLength){
      value = rxBuffer[rxBufferIndex];
      ++rxBufferIndex;
    }
    return value;
  }

  int peek(void) {
    int value = -1;
    
    if(rxBufferIndex < rxBufferLength){
      value = rxBuffer[rxBufferIndex];
    }
    return value;
  }

  void flush(void) {
  }

  inline size_t write(unsigned long n) { return write((uint8_t)n); }
  
  inline size_t write(long n) { return write((uint8_t)n); }

  inline size_t write(unsigned int n) { return write((uint8_t)n); }

  inline size_t write(int n) { return write((uint8_t)n); }

  using Print::write;
};


#endif