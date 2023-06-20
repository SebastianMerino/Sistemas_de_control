#include <Arduino.h>
#include <i2c.h>
#include <math.h>
#include <motor.h>
#include <mpu.h>

mpuStructData mpuData;
Motor_VA motor;
//Resolución del motor: 1194

// put function declarations here:
void RespuestaImpulso(void)
{
  motor.tb6612fng_Wake();       // Pone pin en alta

  // Se imprimen datos de posicion y ángulo en el puerto serial
  Serial.println();
  Serial.print("y = [");

  TickType_t PeriodTicks, LastWakeTime;
  PeriodTicks = 1;
  LastWakeTime = xTaskGetTickCount();
  motor.tb6612fng_Voltage(0.2);
  for (int i = 0; i < 2000; i++)
  {
    double pos = motor.GetEncoderPosition(DEGREES)*0.03;
	  Serial.printf("%f,",pos);
    vTaskDelayUntil(&LastWakeTime, PeriodTicks);
  }
  Serial.println("];");
  motor.tb6612fng_Sleep();
}


double ang_test = 0, ang_acc;
double peso_giro = 0.98;
double T_ang = 0.5;
TickType_t PeriodAngTicks, LastTimeAng;

void setup() {
  Serial.begin(115200);

  mpuInit();
  PeriodAngTicks = T_ang*1000;
  //motor.init(20000,374);//960, 1280
  //Serial.println("Driver configurado :)");
  //delay(1000);  
  //RespuestaImpulso();  
}


void loop() {
  mpuData = mpuGetData();
  Serial.printf("Ax: %f, Az:%f, Gy:%f, ",mpuData.Ax,mpuData.Az,mpuData.Gy);
  
  ang_acc = atan(mpuData.Ax/mpuData.Az)*(180.0/M_PI);
  ang_test = peso_giro*(ang_test + mpuData.Gy*T_ang) + (1 - peso_giro)*ang_acc;
  Serial.printf("Angulo: %f\n",ang_test);
  vTaskDelayUntil(&LastTimeAng, PeriodAngTicks);
}
