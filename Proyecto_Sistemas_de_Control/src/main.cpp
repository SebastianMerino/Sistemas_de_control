#include <Arduino.h>
#include <i2c.h>
#include <math.h>
#include <motor.h>
#include <mpu.h>
#define G 10.5



mpuStructData mpuMeasurements;
Motor_VA motor;
//Resolución del motor: 1194
float time_to_print = millis();

// put function declarations here:
void RespuestaImpulso(void) {
  motor.tb6612fng_Wake();       // Pone pin en alta

  // Se imprimen datos de posicion y ángulo en el puerto serial
  Serial.println();
  Serial.print("y = [");

  TickType_t PeriodTicks = 1;
  TickType_t LastWakeTime = xTaskGetTickCount();
  motor.tb6612fng_Voltage(0.2);
  for (int i = 0; i < 2000; i++)
  {
	double angulo = getAngle();
   // double vel = motor.GetEncoderSpeed(DEGREES_PER_SECOND)*0.03;
    double pos = motor.GetEncoderPosition(DEGREES)*0.03;
	Serial.printf("%f,%f;",pos,angulo);
    vTaskDelayUntil(&LastWakeTime, PeriodTicks);
  }
  Serial.println("];");
  motor.tb6612fng_Sleep();
//  board.DRV8837_Sleep();
}



void setup() {
  Serial.begin(115200);
  mpuInit();

  motor.init(20000,374);//960, 1280
  Serial.println("Driver configurado :)");
  delay(1000);  
  RespuestaImpulso();  
}

void loop() {
	delay(1000);
}
