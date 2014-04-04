#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

void setup()
{
  unsigned char readBuffer[32];
  unsigned char writeBuffer[2] = {bmpControlRegister, bmpTemperature};
  double altitude;
  
  delay(4000);
  Serial.begin(9600);
  initializeQuadcopter();

  readI2C(bmpAddress, readBuffer, 32);  
  while(1)
  {
  readI2CBarometer(&altitude);
  Serial.println(altitude);
  _delay_ms(100);
  }
}

void loop(){}
