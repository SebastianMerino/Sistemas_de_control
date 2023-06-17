#include "i2c.h"

uint8_t devAddress;

void i2cInit(uint8_t slaveAddress)
{
    devAddress = slaveAddress;
    Wire.begin(SDA, SCL);
}

void i2cWrite(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.write(data);
    Wire.endTransmission();
}

int16_t i2cReadRegister16(uint8_t reg)
{
    int16_t value;
    uint8_t vha, vla;
    Wire.beginTransmission(devAddress);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(devAddress);
    Wire.requestFrom(devAddress, 2);
    while(!Wire.available());

    #if ARDUINO >= 100
        vha = Wire.read();
        vla = Wire.read();
    #else
        vha = Wire.receive();
        vla = Wire.receive();
    #endif
    Wire.endTransmission();

    value = vha << 8 | vla;


    return value;
}

void i2cReadValueArray(uint8_t deviceAddress, uint8_t regAddress, uint8_t *array, uint8_t arraySize)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, arraySize);
    for (uint8_t i = 0; i < arraySize;i++)
    {
        #if ARDUINO >= 100
            array[i] = Wire.read();
        #else
            array[i] = Wire.receive();
        #endif
    }
    Wire.endTransmission();
}