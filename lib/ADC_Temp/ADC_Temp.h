/*#ifndef ADC_TEMP_H
#define ADC_TEMP_H

#include <Arduino.h>

A library for an NTC temperature sensor. It can be used, however it has not been tested yet!!!

// https://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/
// Based on: https://github.com/e-tinkers/ntc-thermistor-with-arduino-and-esp32/blob/master/ntc_3950.ino
// ADC calibration: https://github.com/e-tinkers/esp32-adc-calibrate

// The formula for temp in kelvin is
//                 1
// T = ----------------------------
//     1/To + (1/beta) * ln(Rt/Ro)



#define BETA 3950.0 // Beta value of NTC3950
#define To 298.15 // Temparature in Kelvin for 25 Degrees Celsius
#define RO 100000.0 // Resistance of Thermistor at 25 Degrees Celsius
#define RESISTOR 98300.0//100000.0 // 100 kOhm, Resistor added for 

#define ADC_MAX 1023.0 // 10 bit ADC in ESP8266
#define VS 3.258 // Supply voltage

class ADC_Temp
{
  public:
    ADC_Temp(int pin);
    double getMeasurement(void);
  private:
    int PIN_NTC;
};

#endif // ADC_TEMP

*/