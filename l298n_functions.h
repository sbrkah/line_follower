#ifndef L298N_FUNCTIONS_H
#define L298N_FUNCTIONS_H
#include "Arduino.h"

class l298n{
    public:
        l298n(int en_a, int in1, int in2, int in3, int in4, int en_b);
        void init();
        void setMotor1(int direction, int pwm);
        void setMotor2(int direction, int pwm);
        void DRIVE();
        void STOP();

    private:
        int _pin_en_a;
        int _pin_in1;
        int _pin_in2;
        int _pin_in3;
        int _pin_in4;
        int _pin_en_b;
        int _pwm_a;
        int _state_in1;
        int _state_in2;
        int _state_in3;
        int _state_in4;
        int _pwm_b;
};

#endif
