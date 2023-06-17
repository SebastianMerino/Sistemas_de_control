#include <Arduino.h>
#include <XSpaceV2.h>

XSpaceV2Board board;
float vel;

void ControlVelocidad(void *pvParameters)
{
  double kp = 0.0110;
  double ki = 0.0605;
  double T = 0.02;

  double uk, ek, ek_1 = 0, uk_1 = 0;

  double vel, vel_ref = 180;

  board.DRV8837_Wake();

  while(1){
    vel=board.GetEncoderSpeed(DEGREES_PER_SECOND);
    Serial.println(vel);

    ek = vel_ref - vel;

    uk = (kp+T/2*ki)*ek + (T/2*ki -kp)*ek_1 +uk_1;

    ek_1 = ek;
    uk_1 = uk;

    board.DRV8837_Voltage(uk);
    vTaskDelay(20);
  }

  vTaskDelete(NULL);
}

void RespuestaImpulso(void)
{
  board.DRV8837_Wake();       // Pone pin en alta

  // Se imprimen datos de velocidad en el puerto serial
  Serial.println();
  Serial.print("v = [");

  board.DRV8837_Voltage(2);
  for (int i = 0; i < 2001; i++)
  {
    vel = board.GetEncoderSpeed(DEGREES_PER_SECOND);
    Serial.print(vel);
    Serial.print(" ");
    delay(1);
  }
  Serial.println("];");

  board.DRV8837_Sleep();
}

void setup() {
  Serial.begin(115200);
  board.init(20000,960);    // PWM frequency, 960 para el lab, 256*5 para el Xspace
  
  //RespuestaImpulso();
  xTaskCreate(ControlVelocidad,"",2000,NULL,1,NULL);
}

void loop() {
  delay(1000);
}