#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor{
    private:
        uint8_t IN1;
        uint8_t IN2;
        uint8_t EN;
        uint16_t maxPWM;

    public:
        Motor(uint8_t IN1, uint8_t IN2, uint8_t EN, uint16_t maxPWM);
        void move(int pwm);

};

#endif
