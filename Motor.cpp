#include "Motor.h"

Motor::Motor(uint8_t IN1, uint8_t IN2, uint8_t EN, uint16_t maxPWM){
    this->IN1 = IN1;
    this->IN2 = IN2;
    this->EN = EN;
    this->maxPWM = maxPWM;
    
    pinMode(this->IN1, OUTPUT);
    pinMode(this->IN2, OUTPUT);
}

void Motor::move(int pwm){
    if(pwm > 0){
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        analogWrite(EN, (pwm > maxPWM) ? maxPWM : pwm);
//        Serial.print(pwm);
//        Serial.print(" ");
    } else{
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        analogWrite(EN, (-pwm > maxPWM) ? maxPWM : -pwm);
//        Serial.print(pwm);
//        Serial.print(" ");
    }
}
