#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

#define PROPORTIONAL_GAIN 1.0
#define INTEGRAL_GAIN 1.0
#define DERIVATIVE_GAIN 1.0

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
  double xP = - PROPORTIONAL_GAIN * (xAngleError);
  double yP = - PROPORTIONAL_GAIN * (yAngleError);
  double xD = - DERIVATIVE_GAIN * (xAngleError - lastXAngleError)/timeDifference;
  double yD = - DERIVATIVE_GAIN * (yAngleError - lastYAngleError)/timeDifference;
  integralXAngleError += (lastXAngleError + xAngleError) * (timeDifference / 2.0);
  integralYAngleError += (lastYAngleError + yAngleError) * (timeDifference / 2.0);
  double xI = - INTEGRAL_GAIN * integralXAngleError;
  double yI = - INTEGRAL_GAIN * integralYAngleError;
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
  xCorrection = - PROPORTIONAL_GAIN * (xAngleError);
  yCorrection = - PROPORTIONAL_GAIN * (yAngleError);
}

void setup()
{
  Serial.begin(9600);
  initializeQuadcopter();
  PORTB = 0;
}

void loop(){}
