#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

void setup()
{
  double altitude;
  
  _delay_ms(4000);
  Serial.begin(9600);
  initializeQuadcopter();
  
  Serial.print("Initial Altitude: ");
  Serial.println(initialAltitude);
  
  readI2CBarometer(&altitude);
  
  while(1)
  {
  readI2CBarometer(&altitude);
  _delay_ms(100);
  Serial.println(altitude);
  }
}

void loop(){}
