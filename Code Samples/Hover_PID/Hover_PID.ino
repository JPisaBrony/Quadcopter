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
  timeDifferential = 0;
  xAnglePV = atan2(accelerometerData[0], accelerometerData[2]);
  yAnglePV = atan2(accelerometerData[1], accelerometerData[2]);
  double xAngleError = xAnglePV - xAngleSP;
  double yAngleError = yAnglePV - yAngleSP;
  double xP = X_PROPORTIONAL_GAIN * (xAngleError);
  double yP = Y_PROPORTIONAL_GAIN * (yAngleError);
  double xDerivative = (xAngleError - lastXAngleError)/time;
  double yDerivative = (yAngleError - lastYAngleError)/time;
  double xD = X_DERIVATIVE_GAIN * xDerivative;
  double yD = Y_DERIVATIVE_GAIN * yDerivative;
  integralXAngleError += (lastXAngleError + xAngleError) * (time / 2.0); //trapizoid rule
  integralYAngleError += (lastYAngleError + yAngleError) * (time / 2.0);
  double xI = X_INTEGRAL_GAIN * integralXAngleError;
  double yI = Y_INTEGRAL_GAIN * integralYAngleError;
  lastXAngleError = xAngleError;
  lastYAngleError = yAngleError;
  xCorrection = xP + xI + xD;
  yCorrection = yP + yI + yD;

  Serial.print(time);
  Serial.print(',');
  Serial.print(yAnglePV, 5);
  Serial.print(',');
  Serial.print(yAngleError, 5);
  Serial.print(',');
  Serial.print(integralYAngleError, 5);
  Serial.print(',');
  Serial.print(yDerivative, 5);
  Serial.print(',');
  Serial.print(yP, 5);
  Serial.print(',');
  Serial.print(yI, 5);
  Serial.print(',');
  Serial.print(yD, 5);
  Serial.print(',');
  Serial.print(yCorrection, 5);
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
  xCorrection = X_PROPORTIONAL_GAIN * (xAngleError);
  yCorrection = Y_PROPORTIONAL_GAIN * (yAngleError);
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
      readI2CAccelerometer(accel);
      updateSetPointAndCorrection(0, 0, accel);//set the starting point
      
      while(digitalInput(_PIN7))
      {
        //Potentiometers to addjust constants; Not for final quadcopter.
        Y_PROPORTIONAL_GAIN = 0.0374103943;//((analogInput(_A0)/ 2.5)-1) / 16.0;
        Y_INTEGRAL_GAIN = 4.765655329E-5;//((analogInput(_A0) / 5.0)) / 10000.0;
        Y_DERIVATIVE_GAIN = 2.561;//analogInput(_A0) * 4;
        
        readI2CAccelerometer(accel);
        
        updateCorrection(accel);
        Serial.print(',');
        Serial.print(Y_PROPORTIONAL_GAIN, 10);
        Serial.print(',');
        Serial.print(Y_INTEGRAL_GAIN, 10);
        Serial.print(',');
        Serial.println(Y_DERIVATIVE_GAIN, 10);
        
        setMotors(0, 0, 0.5 + yCorrection, 0.5 - yCorrection);
        
      }
    }
    stopMotors();
    //Serial.println("Flip the switch to start.");
  }
}

void loop(){}
