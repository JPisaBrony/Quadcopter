#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

void setup()
{
  Serial.begin(9600);
  initializePWM(50);
  
  setDirection(_PIN5, _OUTPUT);
  setDirection(_PIN9, _OUTPUT);
  setDirection(_PIN10, _OUTPUT);
  setDirection(_PIN11, _OUTPUT);
  setDirection(_A5, _INPUT);
  
  double volts, duty;
  
  while(1)
  {
    volts = analogInput(_A5);
    
    duty = volts / 25.0;
    
    PWMOutput(_PIN5PWM, duty);
    PWMOutput(_PIN9PWM, duty);
    PWMOutput(_PIN10PWM, duty);
    PWMOutput(_PIN11PWM, duty);
    
    Serial.println(duty, 10);
    
    _delay_ms(200);
  }
}

void loop(){}
