#include "\Quadcopter\QuadLibrary\quadcopter.h"

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
 
  setDirection(_PIN7, _INPUT);
 
  while(1)
  {
    if(digitalInput(_PIN7))
    {
      _delay_ms(5000);
      while(digitalInput(_PIN7))
      {
        /*readI2CAccelerometer(accel);
        
        xAngle = atan2(accel[0], accel[2]);
        duty[xp] = ANGLE_ADJUSTMENT_CONSTANT * xAngle + MIDDLE_POWER_VALUE;
        duty[xn] = - ANGLE_ADJUSTMENT_CONSTANT * xAngle + MIDDLE_POWER_VALUE;
         
        yAngle = atan2(accel[1], accel[2]);
        duty[yp] = ANGLE_ADJUSTMENT_CONSTANT * yAngle + MIDDLE_POWER_VALUE;
        duty[yn] = - ANGLE_ADJUSTMENT_CONSTANT * yAngle + MIDDLE_POWER_VALUE;
        */
        
        duty[xp] = 0.5;
        duty[xn] = 0.5;
        duty[yp] = 0.5;
        duty[yn] = 0.5;
        
        for(int i = 0; i < 4; i++)
          duty[i] = duty[i] * (upperLimit[i] - lowerLimit[i]) + lowerLimit[i];
         
        setMotorPWMDuty(duty[xp], duty[xn], duty[yp], duty[yn]);
        
        if(Serial.available() && Serial.read())
        {
          Serial.print("xp duty: ");
          Serial.println(duty[xp], 5);
          Serial.print("OCR3A: ");
          Serial.println(OCR3A);
          Serial.println();
          Serial.print("xn duty: ");
          Serial.println(duty[xn], 5);
          Serial.print("OCR1A: ");
          Serial.println(OCR1A);
          Serial.println();
          Serial.print("yp duty: ");
          Serial.println(duty[yp], 5);
          Serial.print("OCR1B: ");
          Serial.println(OCR1B);
          Serial.println();
          Serial.print("yn duty: ");
          Serial.println(duty[yn], 5);
          Serial.print("OCR1C: ");
          Serial.println(OCR1C);
          Serial.println();
          Serial.println();
        }
      }
    }
    stopMotors();
    Serial.println("Flip the switch to start.");
  }
}

void loop(){}
