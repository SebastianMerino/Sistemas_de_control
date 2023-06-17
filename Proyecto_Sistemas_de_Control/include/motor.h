
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <stdint.h>

//Declaración de puertos

#define PWM_D 32
#define PWM_I 33

#define D_IN1 18
#define D_IN2 19
#define I_IN1 4
#define I_IN2 2

#define STDBY 25

#define encoder_CHA_D 34
#define encoder_CHB_D 35

#define encoder_CHA_I 12
#define encoder_CHB_I 13

#define DEGREES_PER_SECOND 1
#define RADS_PER_SECOND 2

#define DEGREES 1
#define RADS 2

extern volatile double Periodo;

class Motor_VA{
    private:
        double vel_ant = 0;
        bool XSpace_info = false;

        double _resolution;

    public:
        void init(int freq, double resolution);
        void tb6612fng_Sleep();
        void tb6612fng_Wake();
        void tb6612fng_Voltage(double vp);
        
        double GetEncoderSpeed(int modo);
        double GetEncoderPosition(int modo);
};

#endif