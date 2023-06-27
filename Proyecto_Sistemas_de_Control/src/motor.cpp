
#include <motor.h>
#include <stdint.h>

volatile double T = 0;
volatile double Periodo = 1000000;
volatile double Tant = 0;
volatile double counter = 0;

void IRAM_ATTR ISR_encoder()
{
    T = micros();

    Periodo = T - Tant;
    Tant = T;

    if(digitalRead(encoder_CHB_D) == LOW){
        Periodo = (-1)*Periodo;
        counter--;
    } else counter++;
    
}

void Motor_VA::init(int freq, double resolution){
    pinMode(STDBY, OUTPUT);
    pinMode(D_IN1, OUTPUT);
    pinMode(D_IN2, OUTPUT);
    pinMode(I_IN1, OUTPUT);
    pinMode(I_IN2, OUTPUT);
    //Serial.println("Pines configurados");

    pinMode(PWM_D, OUTPUT);
    pinMode(PWM_I, OUTPUT);
    // ledcSetup(1, freq, 10);
    // ledcSetup(2, freq, 10);
    // ledcAttachPin(PWM_D, 1);
    // ledcAttachPin(PWM_I, 2);


    pinMode(encoder_CHA_D,INPUT_PULLDOWN);
    pinMode(encoder_CHB_D,INPUT_PULLDOWN);
    attachInterrupt(encoder_CHA_D, ISR_encoder, RISING);

    this->_resolution = resolution;

}

void Motor_VA::tb6612fng_Sleep(){
    digitalWrite(STDBY,LOW);
}

void Motor_VA::tb6612fng_Wake(){
    digitalWrite(STDBY,HIGH);
}

void Motor_VA::tb6612fng_Voltage(double vp){

    double vm = 7.4;

    if(vp>7.4) vp = 7.4;
    if(vp<-7.4) vp = -7.4;
    int Duty = (int) ( 255 - (1 - abs(vp)/vm) * 255.0);

    if(vp>0){
        digitalWrite(D_IN1,HIGH);
        digitalWrite(D_IN2,LOW);
        digitalWrite(I_IN1,HIGH);
        digitalWrite(I_IN2,LOW);
        analogWrite(PWM_D, Duty);
        analogWrite(PWM_I, Duty);
    }else{
        digitalWrite(D_IN1,LOW);
        digitalWrite(D_IN2,HIGH);
        digitalWrite(I_IN1,LOW);
        digitalWrite(I_IN2,HIGH);
        analogWrite(PWM_D, Duty);
        analogWrite(PWM_I, Duty);
    }
}

double Motor_VA::GetEncoderSpeed(int modo){
    double vel=0;

    switch (modo)
    {
    case DEGREES_PER_SECOND:
        vel = 360000000.0/(this->_resolution*Periodo);
        //if(abs(vel)>800)vel=vel_ant;
        //if(abs(vel-vel_ant)>100)vel=vel_ant;
        break;
    case RADS_PER_SECOND:
        vel = 6283185.30717/(this->_resolution*Periodo);
        break;
    
    default:
        break;
    }

    vel_ant = vel;

    return vel;
}

double Motor_VA::GetEncoderPosition(int modo){
    double pos=0;
    double rad=0;

    switch (modo)
    {
    case DEGREES:
        pos = counter/this->_resolution*360.0;
        break;
    case RADS:
        pos = counter/this->_resolution*2*PI;
        break;
    
    default:
        break;
    }

    return pos;
}