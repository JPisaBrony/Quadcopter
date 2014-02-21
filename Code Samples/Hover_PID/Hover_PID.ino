#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

double X_PROPORTIONAL_GAIN;//0.5151515007
double X_INTEGRAL_GAIN;//0.0000537634
double X_DERIVATIVE_GAIN;
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
  double xDerivative = (xAngleError - lastXAngleError)/timeDifference;
  double yDerivative = (yAngleError - lastYAngleError)/timeDifference;
  double xD = - X_DERIVATIVE_GAIN * xDerivative;
  double yD = - Y_DERIVATIVE_GAIN * yDerivative;
  integralXAngleError += (lastXAngleError + xAngleError) * (timeDifference / 2.0);
  integralYAngleError += (lastYAngleError + yAngleError) * (timeDifference / 2.0);
  double xI = - X_INTEGRAL_GAIN * integralXAngleError;
  double yI = - Y_INTEGRAL_GAIN * integralYAngleError;
  lastXAngleError = xAngleError;
  lastYAngleError = yAngleError;
  xCorrection = xP + xI + xD;
  yCorrection = yP + yI + yD;
  Serial.print(Y_PROPORTIONAL_GAIN, 10);  
  Serial.print(",");
  Serial.print(Y_INTEGRAL_GAIN, 10);  
  Serial.print(",");
  Serial.print(Y_DERIVATIVE_GAIN, 10);  
  Serial.print(",");
  Serial.print(yAngleError);
  Serial.print(",");
  Serial.print(yDerivative);
  Serial.print(",");
  Serial.print(integralYAngleError);
  Serial.print(",");
  Serial.print(yP);
  Serial.print(",");
  Serial.print(yI);
  Serial.print(",");
  Serial.print(yD);
  Serial.print(",");
  Serial.print(yCorrection);
  Serial.println();
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
  setDirection(_A0, _INPUT);
  setDirection(_A1, _INPUT);
  setDirection(_A2, _INPUT);
  
  while(1)
  {
    if(digitalInput(_PIN7))
    {
      _delay_ms(5000);
      
      while(digitalInput(_PIN7))
      {
        X_PROPORTIONAL_GAIN = analogInput(_A0) / 5.0; //0.53 was good
        X_INTEGRAL_GAIN = analogInput(_A1) / 50000.0;
        X_DERIVATIVE_GAIN = 0;//analogInput(_A2) / 5.0;
        
        
        readI2CAccelerometer(accel);
        
        time++;
        updateCorrection(accel);
        
        setMotors(0, 0, 0.75 - yCorrection, 0.75 + yCorrection);
      }
    }
    readI2CAccelerometer(accel);
    updateSetPointAndCorrection(0, 0, accel);
    stopMotors();
    //Serial.println("Flip the switch to start.");
  }
}

void loop(){}
