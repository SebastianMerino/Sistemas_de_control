#include <Arduino.h>
#include <i2c.h>
#include <math.h>
#include <motor.h>
#include <mpu.h>

mpuStructData mpuData;
Motor_VA motor;
double ang_ref = 0;

void RespuestaImpulso(void)
{
  double pos,vel,ang = 0,vel_ang = 0, voltage = 3;
  motor.tb6612fng_Wake();       // Pone pin en alta

  // Se imprimen datos de posicion y ángulo en el puerto serial
  Serial.println();
  Serial.print("y = [");

  TickType_t PeriodTicks, LastWakeTime;
  PeriodTicks = 2;
  LastWakeTime = xTaskGetTickCount();
  motor.tb6612fng_Voltage(voltage);
  for (int i = 0; i < 1000; i++)
  {
    pos = motor.GetEncoderPosition(DEGREES)*0.03;
    vel = motor.GetEncoderSpeed(DEGREES_PER_SECOND)*0.03;
    mpuData = mpuGetData();
    if (mpuData.Az != 0) {
      ang = atan2(mpuData.Ax,mpuData.Az)*(180.0/M_PI);
      vel_ang = -mpuData.Gy;
    }    
	  Serial.printf("%f, %f, %f, %f; ",pos, vel, ang, vel_ang);
    vTaskDelayUntil(&LastWakeTime, PeriodTicks);
    if (i%100 == 0) {
      voltage *= -1;
      motor.tb6612fng_Voltage(voltage);
    }
  }
  Serial.println("];");
  motor.tb6612fng_Sleep();
}

void ControlAngulo(void *pvParameters)
{
  double Ki = 0, Kpos = 1.75, Kvel = -0.18, Kang = 0.0026, Kang_vel = 0.0166, T = 0.05;
  double uk, ek, ek_1 = 0;
  double pos, vel, ang, ang_vel;

  TickType_t LastWakeTime = xTaskGetTickCount();
  TickType_t PeriodTicks = T*1000;
  motor.tb6612fng_Wake();
  while(1){
    vTaskDelayUntil(&LastWakeTime, PeriodTicks);
    
    pos = motor.GetEncoderPosition(DEGREES);
    vel = motor.GetEncoderSpeed(DEGREES_PER_SECOND);
    ang = getAngle();
    ang_vel = getAngularVelocity();
    ek = ang_ref - ang;
    ek = ek_1 + ek*T;

    uk = Ki*ek - Kpos*pos - Kvel*vel - Kang*ang - Kang_vel*ang_vel;
    motor.tb6612fng_Voltage(uk);

    ek_1 = ek;
    Serial.printf("Angulo: %+07.02f\r",ang);
  }
  //vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  motor.init(20000,374);
  mpuInit();
  Serial.println();

  //RespuestaImpulso();  
  xTaskCreate(ControlAngulo,"",2000,NULL,1,NULL);
}

void loop() {  
  // double angle, angVel;
  // angle = getAngle();
  // angVel = getAngularVelocity();
  // Serial.printf("Angulo: %+07.02f, Vel angular: %+07.02f\r",angle,angVel);
  delay(500);
}
