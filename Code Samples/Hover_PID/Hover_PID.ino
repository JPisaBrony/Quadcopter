#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

#define ROLLING_AVERAGE_LENGTH 10

double accel[3], xRollingAverage[ROLLING_AVERAGE_LENGTH], yRollingAverage[ROLLING_AVERAGE_LENGTH];
int xAverageIndex = 0, yAverageIndex = 0;

void newXAngle(double value)
{
  xRollingAverage[xAverageIndex] = value;
  xAverageIndex = (xAverageIndex + 1) % ROLLING_AVERAGE_LENGTH;
}

double averageXAngle()
{
  double sum = 0;
  for(int i = 0; i < ROLLING_AVERAGE_LENGTH; i++)
    sum += xRollingAverage[i];
  return sum / ROLLING_AVERAGE_LENGTH;
}

void newYAngle(double value)
{
  yRollingAverage[yAverageIndex] = value;
  yAverageIndex = (yAverageIndex + 1) % ROLLING_AVERAGE_LENGTH;
}

double averageYAngle()
{
  double sum = 0;
  for(int i = 0; i < ROLLING_AVERAGE_LENGTH; i++)
    sum += yRollingAverage[i];
  return sum / ROLLING_AVERAGE_LENGTH;
}

void setup()
{
  Serial.begin(9600);
  initializeQuadcopter();
  double xAngle, yAngle;
  setDirection(_PIN7, _INPUT);
 
  while(1)
  {
    if(digitalInput(_PIN7))
    {
      _delay_ms(5000);
      
      for(int i = 0; i < ROLLING_AVERAGE_LENGTH; i++)
      {
        readI2CAccelerometer(accel);
        newXAngle(atan2(accel[0], accel[2]));
        newYAngle(atan2(accel[1], accel[2]));
      }
        
      
      while(digitalInput(_PIN7))
      {
        readI2CAccelerometer(accel);
        xAngle = atan2(accel[0], accel[2]);
        yAngle = atan2(accel[1], accel[2]);
        
        newXAngle(xAngle);
        newYAngle(yAngle);
        
        Serial.print("X Angle: ");
        Serial.println(xAngle);
        Serial.print("X Angle Average: ");
        Serial.println(averageXAngle());
        Serial.print("Y Angle: ");
        Serial.println(xAngle);
        Serial.print("Y Angle Average: ");
        Serial.println(averageYAngle());
      }
    }
    stopMotors();
    Serial.println("Flip the switch to start.");
    _delay_ms(100);
  }
}

void loop(){}
