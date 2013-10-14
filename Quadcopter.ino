//#include "quadcopter.h"

void setup()
{
  setDirection(_PIN10, _OUTPUT);
  setDirection(_A5, _INPUT);
  initializePWM(50);
  float i, pi;
  while(1)
  {
    i = analogInput(_A5);
    if(i != pi)
    {
      PWMOutput(_PIN10PWM, i / 5.0 * 0.09 + 0.02);
      pi = i;
    }
  }
}

void loop(){}
