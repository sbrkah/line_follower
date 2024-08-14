#include "l298n_functions.h"

l298n::l298n(int en_a, int in1, int in2, int in3, int in4, int en_b){
    _pin_en_a = en_a;
    _pin_in1 = in1;
    _pin_in2 = in2;
    _pin_in3 = in3;
    _pin_in4 = in4;
    _pin_en_b = en_b;
}

void l298n::init(){
    pinMode(_pin_en_a, OUTPUT);
    pinMode(_pin_in1, OUTPUT);
    pinMode(_pin_in2, OUTPUT);
    pinMode(_pin_in3, OUTPUT);
    pinMode(_pin_in4, OUTPUT);
    pinMode(_pin_en_b, OUTPUT);
}

void l298n::setMotor1(int direction, int pwm){
    _pwm_a = pwm;
    if(direction > 0){
        _state_in1 = 1;
        _state_in2 = 0;
    }
    else{
        _state_in1 = 0;
        _state_in2 = 1;
    }
    analogWrite(_pin_en_a, _pwm_a);
    digitalWrite(_pin_in1, _state_in1);
    digitalWrite(_pin_in2, _state_in2);
    delay(3);
}

void l298n::setMotor2(int direction, int pwm){
    _pwm_b = pwm;
    if(direction > 0){
        _state_in3 = 1;
        _state_in4 = 0;
    }
    else{
        _state_in3 = 0;
        _state_in4 = 1;
    }
    analogWrite(_pin_en_b, _pwm_b);
    digitalWrite(_pin_in3, _state_in3);
    digitalWrite(_pin_in4, _state_in4);
    delay(3);
}

void l298n::DRIVE(){
    analogWrite(_pin_en_a, _pwm_a);
    analogWrite(_pin_en_b, _pwm_b);
    digitalWrite(_pin_in1, _state_in1);
    digitalWrite(_pin_in2, _state_in2);
    digitalWrite(_pin_in3, _state_in3);
    digitalWrite(_pin_in4, _state_in4);
    delay(3);
}

void l298n::STOP(){
    analogWrite(_pin_en_a, 0);
    analogWrite(_pin_en_b, 0);
    digitalWrite(_pin_in1, 0);
    digitalWrite(_pin_in2, 0);
    digitalWrite(_pin_in3, 0);
    digitalWrite(_pin_in4, 0);
}
