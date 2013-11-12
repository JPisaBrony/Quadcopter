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
  
  initializePWM(50);
  
  int compAxes[3];
  int accAxes[3];
  int gyroAxes[3];
  
  double xp = 0.5, xn = 0.5, yp = 0.5, yn = 0.5, dutyP, dutyN;
  
  setDirection(_PIN10, _OUTPUT);
  setDirection(_PIN11, _OUTPUT);
  setDirection(_PIN7, _INPUT);
 
while(1)
{
    readI2CCompass(compAxes);
    readI2CAccelerometer(accAxes);
    readI2CGyroscope(gyroAxes);
    
    
    if(accAxes[0] > 20) {
      xp += 0.01;
      xn -= 0.01;
    }
    else if (accAxes[0] < -20) {
      xp -= 0.01;
      xn += 0.01;
    }
    if(accAxes[1] > 20) {
      yp += 0.01;
      yn -= 0.01;
    }
    else if(accAxes[1] < -20) {
      yp -= 0.01;
      yn += 0.01;
    }
    
    if(!digitalInput(_PIN7))
    {
      dutyP = 0.07;
      dutyN = 0.07;
      xp = 0.5;
      xn = 0.5;
    }
    else
    {
      dutyP = 0.017 * xp + 0.07;
      dutyN = 0.017 * xn + 0.07;
    }

    PWMOutput(_PIN10PWM, dutyN);    
    PWMOutput(_PIN11PWM, dutyP);
    
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
    Serial.print("Duty (X-Positive): ");
    Serial.println(dutyP, 5);
    Serial.print("Duty (X-Negative): ");
    Serial.println(dutyN, 5);
    Serial.println();
    Serial.println(xp);
    Serial.println(xn);
    Serial.println(yp);
    Serial.println(yn);
    _delay_ms(100);
  }
}

void loop(){}
