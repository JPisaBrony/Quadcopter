#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

void setup()
{
  _delay_ms(5000);
  Serial.begin(9600);
  
  initializeGPS();
  
  while(1)
  {
    double times = rawUTCTime;
    updateGPSData();
    if(times != rawUTCTime)
    Serial.println(rawUTCTime);
  }
}

void loop(){}
