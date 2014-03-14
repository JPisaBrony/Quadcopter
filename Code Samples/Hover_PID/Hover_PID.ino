#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

double X_PROPORTIONAL_GAIN;
double X_INTEGRAL_GAIN;
double X_DERIVATIVE_GAIN;
#define Y_PROPORTIONAL_GAIN 0.0374103943
#define Y_INTEGRAL_GAIN 2.3828276645436E-5
#define Y_DERIVATIVE_GAIN 25.1026401519

double accel[3], gyro[3];
double xAngleSP, yAngleSP, xAnglePV, yAnglePV, lastXAngleError, lastYAngleError, integralXAngleError, integralYAngleError, xCorrection, yCorrection;

void updateCorrection(double accelerometerData[], double gyroData[])
{
  unsigned long time = timeDifferential;
  timeDifferential = 0;
  xAnglePV = atan2(accelerometerData[0], accelerometerData[2]);
  yAnglePV = atan2(accelerometerData[1], accelerometerData[2]);
  double xAngleError = xAnglePV - xAngleSP;
  double yAngleError = yAnglePV - yAngleSP;
  double xP = X_PROPORTIONAL_GAIN * (xAngleError);
  double yP = Y_PROPORTIONAL_GAIN * (yAngleError);
  double xDerivative = 1.5E-6 * gyro[1];
  double yDerivative = 1.5E-6 * gyro[0];
  double xD = X_DERIVATIVE_GAIN * xDerivative;
  double yD = Y_DERIVATIVE_GAIN * yDerivative;  
  integralXAngleError += xAngleError * time;
  integralYAngleError += yAngleError * time;
  double xI = X_INTEGRAL_GAIN * integralXAngleError;
  double yI = Y_INTEGRAL_GAIN * integralYAngleError;
  xCorrection = xP + xI + xD;
  yCorrection = yP + yI + yD;

  Serial.print(time);
  Serial.print(',');
  Serial.print(xAnglePV, 5);
  Serial.print(',');
  Serial.print(xAngleError, 5);
  Serial.print(',');
  Serial.print(integralXAngleError, 5);
  Serial.print(',');
  Serial.print(xDerivative, 5);
  Serial.print(',');
  Serial.print(xP, 5);
  Serial.print(',');
  Serial.print(xI, 5);
  Serial.print(',');
  Serial.print(xD, 5);
  Serial.print(',');
  Serial.print(xCorrection, 5);
  Serial.print(',');
  Serial.print(X_PROPORTIONAL_GAIN, 5);
  Serial.print(',');
  Serial.print(X_INTEGRAL_GAIN, 5);
  Serial.print(',');
  Serial.println(X_DERIVATIVE_GAIN, 5);
}

void updateSetPointAndCorrection(double xAngle, double yAngle, double accelerometerData[], double gyroData[])
{
  xAngleSP = xAngle;
  yAngleSP = yAngle;
  updateCorrection(accelerometerData, gyroData);
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
      timeDifferential = 0;//to prevent bad things from happening
      readI2CAccelerometer(accel);
      readI2CGyroscope(gyro);
      updateSetPointAndCorrection(0, 0, accel, gyro);
      
      while(digitalInput(_PIN7))
      {
        
        X_PROPORTIONAL_GAIN = analogInput(_A0) / 10;
        X_INTEGRAL_GAIN = 0;
        X_DERIVATIVE_GAIN = 0;

        
        readI2CAccelerometer(accel);
        readI2CGyroscope(gyro);
        updateCorrection(accel, gyro);
                
        setMotors(0.5 - xCorrection, 0.5 + xCorrection, 0.5 + yCorrection, 0.5 - yCorrection);
        
      }
    }
    stopMotors();
  }
}

void loop(){}
