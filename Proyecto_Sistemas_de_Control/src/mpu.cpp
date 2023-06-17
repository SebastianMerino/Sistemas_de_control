#include "mpu.h"

static int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

// sensitivity scale factor respective to full scale setting provided in datasheet
// Valores de factor de sensibilidad, dependen de la hoja tecnica del dispositivo
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;
float limit_value = 0;
int counter_false_movements = 0;
int flag_set_ubication = 1;
int flag_evaluate_movement = 1;

//Aceleraciones iniciales
float Axi=0;
float Ayi=0;
float Azi=0;

void mpuInit(void)
{
    i2cInit(MPU6050_SLAVE_ADDRESS);
    delay(150);
    i2cWrite(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
    i2cWrite(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
    i2cWrite(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
    i2cWrite(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_CONFIG, 0x00);
    i2cWrite(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_GYRO_CONFIG, 0x00);  // set +/-250 degree/second full scale
    i2cWrite(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_ACCEL_CONFIG, 0x00); // set +/- 2g full scale
    i2cWrite(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_FIFO_EN, 0x00);
    i2cWrite(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_INT_ENABLE, 0x01);
    i2cWrite(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
    i2cWrite(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_USER_CTRL, 0x00);

#ifdef PRINT_ACCELEROMETER_INIT
    Serial.print(" * Accelerometer offsets: ");
    Serial.print(mpu_getAccelOffsetX());
    Serial.print(" / ");
    Serial.print(mpu_getAccelOffsetY());
    Serial.print(" / ");
    Serial.println(mpu_getAccelOffsetZ());

    Serial.println();
#endif
}

mpuStructData mpuGetLocationData(void)
{
    mpuStructData mpuMeasurements;

    mpuReadRawValue(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_ACCEL_XOUT_H);
    mpuConvertRawValues(&mpuMeasurements);
    mpuMeasurements.modulo = sqrt(pow(mpuMeasurements.Ax, 2) + pow(mpuMeasurements.Ay, 2) + pow(mpuMeasurements.Az, 2));

    return mpuMeasurements;
}

void mpuReadRawValue(uint8_t deviceAddress, uint8_t regAddress)
{
    uint8_t arraySize = 14;
    uint8_t array[arraySize];
    i2cReadValueArray(deviceAddress, regAddress, array, arraySize);

    AccelX = (((int16_t)array[0] << 8) | array[1]);
    AccelY = (((int16_t)array[2] << 8) | array[3]);
    AccelZ = (((int16_t)array[4] << 8) | array[5]);
    Temperature = (((int16_t)array[6] << 8) | array[7]);
    GyroX = (((int16_t)array[8] << 8) | array[9]);
    GyroY = (((int16_t)array[10] << 8) | array[11]);
    GyroZ = (((int16_t)array[12] << 8) | array[13]);
}

float getTemperatureData(void){

    float dataTemp = 0;
    mpuStructData mpuMeasurements;
    mpuReadRawValue(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_ACCEL_XOUT_H);
    mpuConvertRawValues(&mpuMeasurements);

    dataTemp = mpuMeasurements.T;

    return dataTemp;
}

void mpuConvertRawValues(mpuStructData *mpuMeasurments)
{
    mpuMeasurments->Ax = (double)AccelX / AccelScaleFactor;
    mpuMeasurments->Ay = (double)AccelY / AccelScaleFactor;
    mpuMeasurments->Az = (double)AccelZ / AccelScaleFactor;
    mpuMeasurments->T = (double)Temperature / 340 + 36.53; // formula de temperatura
    mpuMeasurments->Gx = (double)GyroX / GyroScaleFactor;
    mpuMeasurments->Gy = (double)GyroY / GyroScaleFactor;
    mpuMeasurments->Gz = (double)GyroZ / GyroScaleFactor;
}

int16_t mpuGetAccelOffsetX(void)
{
    return i2cReadRegister16(MPU6050_REG_ACCEL_XOFFS_H);
}

int16_t mpuGetAccelOffsetY(void)
{
    return i2cReadRegister16(MPU6050_REG_ACCEL_YOFFS_H);
}

int16_t mpuGetAccelOffsetZ(void)
{
    return i2cReadRegister16(MPU6050_REG_ACCEL_ZOFFS_H);
}

void mpuPrintReadings(mpuStructData *mpuMeasurements)
{
    Serial.print(" Ax: ");
    Serial.print(mpuMeasurements->Ax);
    Serial.print(" Ay: ");
    Serial.print(mpuMeasurements->Ay);
    Serial.print(" Az: ");
    Serial.print(mpuMeasurements->Az);
    Serial.print(" Modulo vector: ");
    Serial.print(mpuMeasurements->modulo);
    Serial.print(" T: ");
    Serial.println(mpuMeasurements->T);
}