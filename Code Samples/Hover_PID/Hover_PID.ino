#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

#define X_PROPORTIONAL_GAIN 0.01
#define X_INTEGRAL_GAIN 0.005
#define X_DERIVATIVE_GAIN 0.0075
#define Y_PROPORTIONAL_GAIN X_PROPORTIONAL_GAIN
#define Y_INTEGRAL_GAIN X_INTEGRAL_GAIN
#define Y_DERIVATIVE_GAIN X_DERIVATIVE_GAIN

double accel[3];
double xAngleSP, yAngleSP, xAnglePV, yAnglePV, lastXAngleError, lastYAngleError, integralXAngleError, integralYAngleError, xCorrection, yCorrection;
unsigned long time;

void updateCorrection(double accelerometerData[])
{
  unsigned long timeDifference = time;
  time = 0;
  xAnglePV = atan2(accelerometerData[0], accelerometerData[2]);
  yAnglePV = atan2(accelerometerData[1], accelerometerData[2]);
  double xAngleError = xAnglePV - xAngleSP;
  double yAngleError = yAnglePV - yAngleSP;
  double xP = - X_PROPORTIONAL_GAIN * (xAngleError);
  double yP = - Y_PROPORTIONAL_GAIN * (yAngleError);
  double xD = - X_DERIVATIVE_GAIN * (xAngleError - lastXAngleError)/timeDifference;
  double yD = - Y_DERIVATIVE_GAIN * (yAngleError - lastYAngleError)/timeDifference;
  integralXAngleError += (lastXAngleError + xAngleError) * (timeDifference / 2.0);
  integralYAngleError += (lastYAngleError + yAngleError) * (timeDifference / 2.0);
  double xI = - X_INTEGRAL_GAIN * integralXAngleError;
  double yI = - Y_INTEGRAL_GAIN * integralYAngleError;
  lastXAngleError = xAngleError;
  lastYAngleError = yAngleError;
  xCorrection = xP + xI + xD;
  yCorrection = yP + yI + yD;
}

void updateSetPointAndCorrection(double xAngle, double yAngle, double accelerometerData[])
{
  time = 0;
  xAngleSP = xAngle;
  yAngleSP = yAngle;
  integralXAngleError = 0;
  integralYAngleError = 0;
  xAnglePV = atan2(accelerometerData[0], accelerometerData[2]);
  yAnglePV = atan2(accelerometerData[1], accelerometerData[2]);
  double xAngleError = xAnglePV - xAngleSP;
  double yAngleError = yAnglePV - yAngleSP;
  lastXAngleError = xAngleError;
  lastYAngleError = yAngleError;
  xCorrection = - X_PROPORTIONAL_GAIN * (xAngleError);
  yCorrection = - X_PROPORTIONAL_GAIN * (yAngleError);
}

void setup()
{
  Serial.begin(9600);
  initializeQuadcopter();
  
  setDirection(_PIN7, _INPUT);
  
  readI2CAccelerometer(accel);
  updateSetPointAndCorrection(0, 0, accel);
  
  while(1)
  {
    if(digitalInput(_PIN7))
    {
      _delay_ms(5000);
      
      while(digitalInput(_PIN7))
      {
        readI2CAccelerometer(accel);
        
        time++;
        updateCorrection(accel);
        
        setMotors(0, 0, 0.5 - yCorrection, 0.5 + yCorrection);
        
        //_delay_ms(0);
      }
    }
    stopMotors();
    //Serial.println("Flip the switch to start.");
  }
}

void loop(){}
