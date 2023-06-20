#ifndef I2C_H
#define I2C_H

#include "Arduino.h"
#include "Wire.h"

#define SCL 23
#define SDA 22

void i2cInit(uint8_t slaveAddress);
void i2cWrite(uint8_t deviceAddress, uint8_t regAddress, uint8_t data);
int16_t i2cReadRegister16(uint8_t reg);
void i2cReadValueArray(uint8_t deviceAddress, uint8_t regAddress, uint8_t *array, uint8_t arraySize);

#endif