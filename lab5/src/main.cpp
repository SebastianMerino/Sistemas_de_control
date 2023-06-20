#include <Arduino.h>
#include <XSpaceV2.h>
#include <math.h>

XSpaceV2Board board;
double pos_ref = 0;
const double a = 512.2;
const double b = 5.94;
const double c = 1721.9;

void RespuestaImpulso(void) {
  board.DRV8837_Wake();       // Pone pin en alta

  // Se imprimen datos de velocidad en el puerto serial
  Serial.println();
  Serial.print("y = [");

  TickType_t PeriodTicks = 1;
  TickType_t LastWakeTime = xTaskGetTickCount();
  board.DRV8837_Voltage(2);
  for (int i = 0; i < 2000; i++)
  {
    double vel = board.GetEncoderPosition(DEGREES_PER_SECOND);
    Serial.print(vel);
    Serial.print(" ");
    vTaskDelayUntil(&LastWakeTime, PeriodTicks);
  }
  Serial.println("];");

  board.DRV8837_Sleep();
}

void ControlPosicion(void *pvParameters)
{
  double Ki = 0.048, Kpos = 0.196, Kvel = 0.037, T = 0.373;
  double uk, ek, ek_1 = 0, vi;
  double pos, vel;

  board.DRV8837_Wake();

  TickType_t LastWakeTime = xTaskGetTickCount();
  TickType_t PeriodTicks = 373;
  while(1){
    vTaskDelayUntil(&LastWakeTime, PeriodTicks);
    
    pos = board.GetEncoderPosition(DEGREES_PER_SECOND) - 90;
    vel = board.GetEncoderSpeed(DEGREES_PER_SECOND);
    ek = pos_ref - pos;
    ek = ek_1 + ek*T;

    uk = Ki*ek - Kpos*pos - Kvel*vel;

    vi = (uk + c*cos(pos*PI/180))/a;
    board.DRV8837_Voltage(vi);
    Serial.printf("vi: %f, theta: %f\n",vi,pos);
    ek_1 = ek;
  }
  vTaskDelete(NULL);
}


void setup() {
  Serial.begin(115200);
  board.SerialInfo(true);
  board.init(20000,960);    // PWM frequency, 960 para el lab, 256*5 para el Xspace
  //RespuestaImpulso();
  xTaskCreate(ControlPosicion,"",2000,NULL,1,NULL);
}

void loop() {
  delay(1000);
}