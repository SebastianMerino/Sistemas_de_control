#include <Arduino.h>
#include <XSpaceV2.h>

XSpaceV2Board board;
double pos_ref = 0;
const char* ref_topic = "referencia/20181792/posicion";

void Mqtt_Callback(char* topic,byte* payload,unsigned int length){
  String incoming = "";
  if (strcmp(ref_topic,topic)==0) {
    for(int i=0; i<length; i++){
      incoming = incoming + (char)payload[i];
    }
    pos_ref = incoming.toDouble();
    Serial.printf("Angulo deseado: %lf\n",pos_ref);
  }
}

void MqttTask(void *pvParameters){
  while(1){
    //Preguntar si me he conectado
    if(!board.Mqtt_IsConnected()){
      board.Mqtt_Connect("XSpaceMerino","Merino","1234");
      board.Mqtt_Suscribe(ref_topic);
    }
    else {
      board.Mqtt_KeepAlive();
    }
    vTaskDelay(1000);
  }
  vTaskDelete(NULL);
}

void ControlPosicion(void *pvParameters)
{
  double Kp = 0.0562, Td = 0.0212, T = 0.05;
  double uk, ek, ek_1 = 0;
  double pos;

  board.DRV8837_Wake();

  TickType_t LastWakeTime = xTaskGetTickCount();
  TickType_t PeriodTicks = T*1000;
  while(1){
    vTaskDelayUntil(&LastWakeTime, PeriodTicks);
    
    pos = board.GetEncoderPosition(DEGREES_PER_SECOND);
    ek = pos_ref - pos;
    uk = Kp/T*(T+Td)*ek - Kp*Td/T*ek_1;
    ek_1 = ek;
    board.DRV8837_Voltage(uk);
  }
  vTaskDelete(NULL);
}

void RespuestaImpulso(void)
{
  board.DRV8837_Wake();       // Pone pin en alta

  // Se imprimen datos de velocidad en el puerto serial
  Serial.println();
  Serial.print("y = [");

  board.DRV8837_Voltage(2);
  for (int i = 0; i <= 1500; i++)
  {
    double vel = board.GetEncoderSpeed(DEGREES_PER_SECOND);
    Serial.print(vel);
    // double pos = board.GetEncoderPosition(DEGREES_PER_SECOND);
    // Serial.print(pos);
    Serial.print(" ");
    delay(1);
  }
  Serial.println("];");

  board.DRV8837_Sleep();
}

void setup() {
  Serial.begin(115200);
  board.SerialInfo(true);
  board.init(20000,256*5);    // PWM frequency, 960 para el lab, 256*5 para el Xspace
  
  board.Wifi_init("MOVISTAR_4B2A","mAdz1c/aRad.eper4");
  board.Mqtt_init("www.xspace.pe",1883);
  
  xTaskCreate(ControlPosicion,"",2000,NULL,1,NULL);
  xTaskCreate(MqttTask,"",4000,NULL,1,NULL);
}

void loop() {
  delay(1000);
}