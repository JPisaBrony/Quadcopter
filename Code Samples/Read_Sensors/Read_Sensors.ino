#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

void setup()
{
  resetSetupRegisters();
  
  compassRegister[magCRA] = 0b01111000;
  compassRegister[magCRB] = 0b00100000;
  compassRegister[magMode] = 0b00000000;
  
  accelerometerRegister[accelBW_RATE] = 0b00001010;
  accelerometerRegister[accelPOWER_CTL] = 0b00001000;
  
  gyroscopeRegister[gyroSMPLRT_DIV] = 0b00000000;
  gyroscopeRegister[gyroDLPF_FS] = 0b00011000;
  
  initializeTWI(100, 0);
  
  int compAxes[3];
  int accAxes[3];
  int gyroAxes[3];
 
while(1)
{
    readI2CCompass(compAxes);
    readI2CAccelerometer(accAxes);
    readI2CGyroscope(gyroAxes);
 
    Serial.println("Magnetic Compass");
    Serial.print("x: ");
    Serial.println(compAxes[0]);
    Serial.print("y: ");
    Serial.println(compAxes[1]);
    Serial.print("z: ");
    Serial.println(compAxes[2]);
    Serial.println();
    Serial.println("Accelerometer");
    Serial.print("x: ");
    Serial.println(accAxes[0]);
    Serial.print("y: ");
    Serial.println(accAxes[1]);
    Serial.print("z: ");
    Serial.println(accAxes[2]);
    Serial.println();
    Serial.println("Gyroscope");
    Serial.print("x: ");
    Serial.println(gyroAxes[0]);
    Serial.print("y: ");
    Serial.println(gyroAxes[1]);
    Serial.print("z: ");
    Serial.println(gyroAxes[2]);
    Serial.println("\n");
    _delay_ms(100);
  }
}

void loop(){}
