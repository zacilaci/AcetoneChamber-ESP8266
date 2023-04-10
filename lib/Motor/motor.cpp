#include "motor.h"

Motor::Motor(int pin, uint32_t frequency)
{//Valid values are from 100Hz to 40000Hz.
  PIN_MOTOR = pin;
  pinMode(PIN_MOTOR, OUTPUT);
  analogWriteFreq(frequency);
  ctr = 0;
}

void Motor::setSpeed(uint8_t speed)
{//Valid values are from 0 to 255.
  analogWrite(PIN_MOTOR, speed);
}

void Motor::stopMotor()
{// stops the motor
  analogWrite(PIN_MOTOR, 0);
}

void Motor::test()
{ // A simple test for the motor
  for (uint8_t i = 0; i <= 255; i++)
  {
    analogWrite(PIN_MOTOR, i);
    delay(100);
  }
  for (uint8_t i = 255; i >= 0; i--)
  {
    analogWrite(PIN_MOTOR, i);
    delay(100);
  }
}