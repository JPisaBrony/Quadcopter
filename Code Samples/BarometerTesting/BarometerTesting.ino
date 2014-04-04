#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

void setup()
{
  unsigned char readBuffer[32];
  unsigned char writeBuffer[2] = {bmpControlRegister, bmpTemperature};
  double altitude;
  
  delay(4000);
  Serial.begin(9600);
  Serial.println(initializeQuadcopter());
  
  Serial.print("Initial Altitude");
  Serial.println(initialAltitude);
  
  while(1)
  {
  readI2CBarometer(&altitude);
  _delay_ms(100);
  }
}

void loop(){}
