#include "mpu.h"

static int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

// sensitivity scale factor respective to full scale setting provided in datasheet
// Valores de factor de sensibilidad, dependen de la hoja tecnica del dispositivo
const double AccelScaleFactor = 1670.1; 	// aceleracion en m/s
const double GyroScaleFactor = 131.07;	// vel angular en °/s
double ang = 0, vel_ang;

void updateAngleTask(void)
{
    const double peso_giro = 0.98;
    const double T_ang = 0.5;       // periodo
    double ang_acc;
    TickType_t PeriodAngTicks, LastTimeAng;
    PeriodAngTicks = T_ang*1000;
    while (1) {
        mpuStructData mpuData = mpuGetData();        
        ang_acc = atan(mpuData.Ax/mpuData.Az)*(180.0/M_PI);
        vel_ang = mpuData.Gy;
        ang = peso_giro*(ang + vel_ang*T_ang) + (1 - peso_giro)*ang_acc;
        vTaskDelayUntil(&LastTimeAng, PeriodAngTicks);
    }
}

double getAngle(void) { return ang; }
double getAngularVelocity(void) { return vel_ang; }

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
}

mpuStructData mpuGetData(void)
{
    mpuStructData mpuMeasurements;
    mpuReadRawValue(MPU6050_SLAVE_ADDRESS, MPU6050_REGISTER_ACCEL_XOUT_H);
    mpuConvertRawValues(&mpuMeasurements);
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

void mpuConvertRawValues(mpuStructData *mpuMeasurements)
{
    mpuMeasurements->Ax = (double)AccelX / AccelScaleFactor;
    mpuMeasurements->Ay = (double)AccelY / AccelScaleFactor;
    mpuMeasurements->Az = (double)AccelZ / AccelScaleFactor;
    mpuMeasurements->T = (double)Temperature / 340 + 36.53; // formula de temperatura
    mpuMeasurements->Gx = (double)GyroX / GyroScaleFactor;
    mpuMeasurements->Gy = (double)GyroY / GyroScaleFactor;
    mpuMeasurements->Gz = (double)GyroZ / GyroScaleFactor;
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