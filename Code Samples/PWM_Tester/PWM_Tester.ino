#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

void setup()
{
  Serial.begin(9600);
  initializePWM(50);
  
  setDirection(_PIN11, _OUTPUT);
  setDirection(_A5, _INPUT);
  
  double volts, duty;
  
  while(1)
  {
    volts = analogInput(_A5);
    
    duty = 0.0034 * volts + 0.070;
    
    PWMOutput(_PIN11PWM, duty);
    
    Serial.println(duty, 10);
  }
}

void loop(){}
