#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// A simple library for the PWM control of a DC motor, only in one direction

class Motor 
{
  public:
    Motor() {};
    Motor(int pin, uint32_t frequency);
    void setSpeed(uint8_t speed);
    void stopMotor(void);
    void test(void);
  private:
    uint8_t ctr;
    int PIN_MOTOR;
};


#endif // !MOTOR_H
