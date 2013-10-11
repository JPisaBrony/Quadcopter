#include <C:\Users\Owner\Quadcopter\quadcopter.c>

void setup()
{
  ADMUX = 0b01000111;
  setDirection(_PIN10, _OUTPUT);
  initializePWM(50);
  int i, pi;
  while(1)
  {
    ADCSRA = 0b11000100;
    while(ADCSRA & 0b01000000);
    i = ADC;
    if(i != pi)
    {
       PWMOutput(_PIN10PWM, i / 1023.0 * 0.09 + 0.02);
       pi = i;
    }
}
}

void loop(){}
