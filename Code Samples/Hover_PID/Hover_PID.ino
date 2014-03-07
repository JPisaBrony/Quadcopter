#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

double X_PROPORTIONAL_GAIN;
double X_INTEGRAL_GAIN;
double X_DERIVATIVE_GAIN;
#define Y_PROPORTIONAL_GAIN X_PROPORTIONAL_GAIN
#define Y_INTEGRAL_GAIN X_INTEGRAL_GAIN
#define Y_DERIVATIVE_GAIN X_DERIVATIVE_GAIN

double accel[3];
double xAngleSP, yAngleSP, xAnglePV, yAnglePV, lastXAngleError, lastYAngleError, integralXAngleError, integralYAngleError, xCorrection, yCorrection;

void updateCorrection(double accelerometerData[])
{
  unsigned long time = timeDifferential;
//  Serial.print("Time: ");
//  Serial.println(timeDifferential);
  timeDifferential = 0;
  xAnglePV = atan2(accelerometerData[0], accelerometerData[2]);
  yAnglePV = atan2(accelerometerData[1], accelerometerData[2]);
//  Serial.print("Angle: ");
//  Serial.println(yAnglePV);
  double xAngleError = xAnglePV - xAngleSP;
  double yAngleError = yAnglePV - yAngleSP;
//  Serial.print("Angle Error: ");
//  Serial.println(yAngleError);
  double xP = - X_PROPORTIONAL_GAIN * (xAngleError);
  double yP = - Y_PROPORTIONAL_GAIN * (yAngleError);
//  Serial.print("P: ");
//  Serial.println(yP);
  double xDerivative = (xAngleError - lastXAngleError)/time;
  double yDerivative = (yAngleError - lastYAngleError)/time;
//  Serial.print("Derivative: ");
//  Serial.println(yDerivative);
  double xD = - X_DERIVATIVE_GAIN * xDerivative;
  double yD = - Y_DERIVATIVE_GAIN * yDerivative;
//  Serial.print("D: ");
//  Serial.println(yD);
  integralXAngleError += (lastXAngleError + xAngleError) * (time / 2.0);
  integralYAngleError += (lastYAngleError + yAngleError) * (time / 2.0);
//  Serial.print("Integral: ");
//  Serial.println(integralYAngleError);
  double xI = - X_INTEGRAL_GAIN * integralXAngleError;
  double yI = - Y_INTEGRAL_GAIN * integralYAngleError;
//  Serial.print("I: ");
//  Serial.println(yI);
  lastXAngleError = xAngleError;
  lastYAngleError = yAngleError;
  xCorrection = xP + xI + xD;
  yCorrection = yP + yI + yD;
//  Serial.print("Correction: ");
//  Serial.println(yCorrection);
//  Serial.println();
}

void updateSetPointAndCorrection(double xAngle, double yAngle, double accelerometerData[])
{
  timeDifferential = 0;
  xAngleSP = xAngle;
  yAngleSP = yAngle;
  xAnglePV = atan2(accelerometerData[0], accelerometerData[2]);
  yAnglePV = atan2(accelerometerData[1], accelerometerData[2]);
  double xAngleError = xAnglePV - xAngleSP;
  double yAngleError = yAnglePV - yAngleSP;
  lastXAngleError = xAngleError;
  lastYAngleError = yAngleError;
  xCorrection = - X_PROPORTIONAL_GAIN * (xAngleError);
  yCorrection = - Y_PROPORTIONAL_GAIN * (yAngleError);
}

void setup()
{
  Serial.begin(9600);
  initializeQuadcopter();
  
  setDirection(_PIN7, _INPUT);
  setDirection(_A0, _INPUT);
  
  while(1)
  {
    if(digitalInput(_PIN7))
    {
      _delay_ms(5000);
      
      while(digitalInput(_PIN7))
      {
        //Potentiometers to addjust constants; Not for final quadcopter.
        X_PROPORTIONAL_GAIN = (analogInput(_A0)/ 5.0) / 4.0;
        X_INTEGRAL_GAIN = 0;//((analogInput(_A0) / 2.5)) / 10000.0;
        X_DERIVATIVE_GAIN = 0;//((analogInput(_A0) / 2.5));
        
        readI2CAccelerometer(accel);
        
        updateCorrection(accel);
        
        setMotors(0, 0, 0.25 - yCorrection, 0.25 + yCorrection);
        //Serial.println(0.75 + yCorrection);
        
      }
      Serial.println(X_PROPORTIONAL_GAIN, 10);
      Serial.println(X_INTEGRAL_GAIN, 10);
      Serial.println(X_DERIVATIVE_GAIN, 10);
    }
    readI2CAccelerometer(accel);
    updateSetPointAndCorrection(0, 0, accel);
    stopMotors();
    //Serial.println("Flip the switch to start.");
  }
}

void loop(){}
