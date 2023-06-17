#include <Arduino.h>
#include <XSpaceV2.h>
#define Y_OP 4.4    // Offset de % flujo
#define U_OP 25.2   // Offset de % de apertura de valvula

XSpaceV2Board board;

void ControlFlujo(void *pvParameters)
{
  // Parámetros de control hallado
  double Kp = 0.0865, Ti = 0.3953, T = 0.125;
  double Ki = Kp/Ti;

  // Inicialización
  double uk, ek, ek_1 = 0, uk_1 = 0;
  double flujo_med;
  double flujo_ref = 50; // Flujo de referencia
  double yk, y_ref;
  y_ref = flujo_ref - Y_OP; // Referencia alrededor del punto de operacion

  while(1){
    // Medicion alrededor del punto de operacion
    flujo_med = board.flujo();
    yk = flujo_med - Y_OP;
    
    // Ley de control
    ek = y_ref - yk;
    uk = (Kp+T/2*Ki)*ek + (T/2*Ki -Kp)*ek_1 + uk_1;

    // Muestras pasadas
    ek_1 = ek; uk_1 = uk;

    // Salida del control a planta real
    board.Valvula(uk + U_OP);

    // Delay por periodo de muestreo
    vTaskDelay((int)T*1000);
  }
  vTaskDelete(NULL);
}

void setup() {
  Serial.begin(115200);
  board.init(20000,256*5);
  xTaskCreate(ControlFlujo,"",2000,NULL,1,NULL);
}

void loop() {
  delay(1000);
}