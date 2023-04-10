#ifndef HEATPAD_H
#define HEATPAD_H

#include <Arduino.h>

// A simple library for the heatpad. Basically it is just a way to put PWM signals on a pin.

class HeatPad
{
  public:
    HeatPad() {};
    HeatPad(int pin_heatpad);
    void TurnOn(int PWMDuty);
    void TurnOff(void);

  private:
    int PIN_HEATPAD;
};

#endif // HEATPAD_H