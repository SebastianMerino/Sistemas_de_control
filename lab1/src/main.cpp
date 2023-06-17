#include <Arduino.h>
#include <XSpaceV2.h>


XSpaceV2Board board;
float vel;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(1E6);

  board.init(20000,960);
  // PWM frequency
  // Resolution debe cambiarse (pulsos por revolucion)
  // Del otro motor 256*5

  board.DRV8837_Wake();       // Pone pin en alta

  // Se imprimen datos de velocidad en el puerto serial
  Serial.println();
  Serial.print("v = [");

  board.DRV8837_Voltage(3);
  for (int i = 0; i < 1001; i++)
  {
    vel = board.GetEncoderSpeed(DEGREES_PER_SECOND);
    Serial.print(vel);
    Serial.print(" ");
    delay(1);
  }
  Serial.println("];");

  board.DRV8837_Sleep();
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("aaa");
  // delay(1000);
}