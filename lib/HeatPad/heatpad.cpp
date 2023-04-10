#include "heatpad.h"

HeatPad::HeatPad(int pin_heatpad) 
{ // initialize the instance, and set the pin
  PIN_HEATPAD = pin_heatpad;
  pinMode(pin_heatpad, OUTPUT);
}

void HeatPad::TurnOn(int PWMDuty)
{
  analogWrite(PIN_HEATPAD, PWMDuty); // apply a PWM duty for the pin.
}

void HeatPad::TurnOff(void)
{
  analogWrite(PIN_HEATPAD, 0); // Give a 0 duty cycle for the heatpad.
}