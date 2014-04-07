#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

void setup()
{
  double altitude;
  
  _delay_ms(4000);
  Serial.begin(9600);
  Serial.println(initializeQuadcopter());
  
  unsigned char readBuffer[22];
  Serial.print("Initial Altitude: ");
  Serial.println(initialAltitude);
  
  while(1)
  {
  readI2CBarometer(&altitude);
  _delay_ms(100);
  Serial.println(altitude);
  }
}

void loop(){}
