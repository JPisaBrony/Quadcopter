#include "/home/josh/Quadcopter/QuadLibrary/quadcopter.h"

#define ANGLE_ADJUSTMENT_CONSTANT 0.0591549430919
#define MIDDLE_POWER_VALUE 0.6
#define xp 0
#define xn 1
#define yp 2
#define yn 3

double lowerLimit[4] = {0.04942668628, 0.043792, 0.0375366544, 0.0387096};

double upperLimit[4] = {0.0782013654, 0.106943, 0.10694373, 0.1018572807};

double duty[4];

void setup()
{  
  initializeQuadcopter();
 
  double accel[3];
  double xAngle, yAngle;
 
  Serial.begin(9600);
 
  setDirection(_PIN5, _OUTPUT);
  setDirection(_PIN9, _OUTPUT);
  setDirection(_PIN10, _OUTPUT);
  setDirection(_PIN11, _OUTPUT);
  setDirection(_PIN7, _INPUT);
 
  initializePWM(50);
 
  while(1)
  {
    if(digitalInput(_PIN7))
    {
      _delay_ms(5000);
      while(digitalInput(_PIN7))
      {
        //readI2CAccelerometer(accel);
        accel[0] = 0;
        accel[1] = 0;
        accel[2] = 255;
        xAngle = atan2(accel[0], accel[2]);
        duty[xp] = ANGLE_ADJUSTMENT_CONSTANT * xAngle + MIDDLE_POWER_VALUE;
        duty[xn] = - ANGLE_ADJUSTMENT_CONSTANT * xAngle + MIDDLE_POWER_VALUE;
         
        yAngle = atan2(accel[1], accel[2]);
        duty[yp] = ANGLE_ADJUSTMENT_CONSTANT * yAngle + MIDDLE_POWER_VALUE;
        duty[yn] = - ANGLE_ADJUSTMENT_CONSTANT * yAngle + MIDDLE_POWER_VALUE;
        
        Serial.println(accel[0], 5);
        Serial.println(accel[1], 5);
        Serial.println(accel[2], 5);
        Serial.println(xAngle, 5);
        Serial.println(duty[xp], 5);
        
        for(int i = 0; i < 4; i++)
          duty[i] = duty[i] * (upperLimit[i] - lowerLimit[i]) + lowerLimit[i];
         
        PWMOutput(_PIN5PWM, duty[xp]);
        PWMOutput(_PIN9PWM, duty[xn]);
        PWMOutput(_PIN10PWM, duty[yp]);
        PWMOutput(_PIN11PWM, duty[yn]);
        
        
        Serial.println(duty[xp], 5);
        Serial.println();
       
      }
    }
    PWMOutput(_PIN5PWM, 0.036);
    PWMOutput(_PIN9PWM, 0.036);
    PWMOutput(_PIN10PWM, 0.036);
    PWMOutput(_PIN11PWM, 0.036);
    Serial.println("Flip the switch to start.");
  }
}

void loop(){}
