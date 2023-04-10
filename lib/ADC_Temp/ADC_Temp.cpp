/*#include "ADC_Temp.h"

ADC_Temp::ADC_Temp(int pin)
{
  //pinMode(PIN_NTC, INPUT);
  PIN_NTC = pin;
}

double ADC_Temp::getMeasurement()
{
  double Temperature = 0;

  double analog = analogRead(PIN_NTC);
  //Serial.print("AnalogValue:\t"); Serial.print(analog); Serial.print("\t");

  double Vout = (VS/ADC_MAX)*analog;
  //Serial.print("Vout:\t"); Serial.print(Vout); Serial.print("\t");

  double Rt = RESISTOR * Vout / (VS - Vout);
  //Serial.print("Rt:\t"); Serial.print(Rt); Serial.print("\t");

  double T = 1/(1/To - log(Rt/RO)/BETA);

  //Serial.print("T:\t"); Serial.print(T); Serial.print("\t");

  Temperature = T - 273.15;
  //Temperature = 273.15 - T;

  //Serial.print("Temp:\t"); Serial.println(Temperature);
  return Temperature;
}*/