#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

double X_PROPORTIONAL_GAIN = 0;
double X_INTEGRAL_GAIN = 0;
double X_DERIVATIVE_GAIN = 0;
double Y_PROPORTIONAL_GAIN = 0;//0.0374103943;
double Y_INTEGRAL_GAIN = 0;//2.3828276645436E-5;
double Y_DERIVATIVE_GAIN = 0;//25.1026401519;
double HEADING_PROPORTIONAL_GAIN = 0;
double HEADING_INTEGRAL_GAIN = 0;
double HEADING_DERIVATIVE_GAIN = 0;
double VELOCITY_PROPORTIONAL_GAIN = 0;
double VELOCITY_INTEGRAL_GAIN = 0;
double VELOCITY_DERIVATIVE_GAIN = 0;

double ACCELEROMETER_LOW_PASS_FILTER_CONSTANT = 30;
double GYROSCOPE_LOW_PASS_FILTER_CONSTANT = 20;
double MAGNOMETER_LOW_PASS_FILTER_CONSTANT = 18;
double BAROMETER_LOW_PASS_FILTER_CONSTANT = 30;

double declination = 0;

double accel[3], gyro[3], mag[3], altitude, oldAccel[3], oldGyro[3], oldMag[3], oldAltitude;

double xAngleSP, yAngleSP, headingSP, altitudeSP;
double xCorrection, yCorrection, hCorrection, vCorrection;

void updateCorrection(double accelData[], double gyroData[], double magData[], double *altitude)
{
  unsigned long time = timeDifferential;
  timeDifferential = 0;
  static double xAnglePV, integralXAngleError;
  static double yAnglePV, integralYAngleError;
  static double headingPV, integralHeadingError;
  static double integralVelocityError, previousIntegralVelocityError = 0;
  
  xAnglePV = atan2(accelData[0], accelData[2]);
  yAnglePV = atan2(accelData[1], accelData[2]);
  headingPV = atan2(-magData[1], magData[0]);
  
  double xAngleError = xAnglePV - xAngleSP;
  double yAngleError = yAnglePV - yAngleSP;
  double headingError = headingPV - headingSP;
  
  double xP = X_PROPORTIONAL_GAIN * xAngleError;
  double yP = Y_PROPORTIONAL_GAIN * yAngleError;
  double hP = HEADING_PROPORTIONAL_GAIN * headingError;
  
  double xDerivative = 1.5E-6 * gyroData[1];
  double yDerivative = 1.5E-6 * gyroData[0];
  double headingDerivative = 1.5E-6 * gyroData[2];
  double velocityDerivative = ((accel[2] > 0) - (accel[2] < 0)) * sqrt(accelData[0] * accelData[0] + accelData[1] * accelData[1] + accelData[2] * accelData[2]) - accelerationDueToGravity;
  
  double xD = X_DERIVATIVE_GAIN * xDerivative;
  double yD = Y_DERIVATIVE_GAIN * yDerivative;
  double hD = HEADING_DERIVATIVE_GAIN * headingDerivative;
  double vD = VELOCITY_DERIVATIVE_GAIN * velocityDerivative;
  
  integralXAngleError += xAngleError * time;
  integralYAngleError += yAngleError * time;
  integralHeadingError += headingError * time;
  integralVelocityError = *altitude - altitudeSP;
  
  double velocityError = (integralVelocityError - previousIntegralVelocityError) / time;
  double vP = VELOCITY_PROPORTIONAL_GAIN * velocityError;
  previousIntegralVelocityError = integralVelocityError;
  
  Serial.print(velocityDerivative, 10);
  Serial.print(',');
  Serial.print(velocityError, 10);
  Serial.print(',');
  Serial.println(integralVelocityError, 10);
  
  double xI = X_INTEGRAL_GAIN * integralXAngleError;
  double yI = Y_INTEGRAL_GAIN * integralYAngleError;
  double hI = HEADING_INTEGRAL_GAIN * integralHeadingError;
  double vI = VELOCITY_INTEGRAL_GAIN * integralVelocityError;
  
  xCorrection = xP + xI + xD;
  yCorrection = yP + yI + yD;
  hCorrection = hP + hI + hD;
  vCorrection = vP + vI + vD;

  /*Serial.print(time);
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
  Serial.println(X_DERIVATIVE_GAIN, 5);*/
}

void updateSetPointAndCorrection(double xAngle, double yAngle,  double heading, double targetAltitude, double accelData[], double gyroData[], double magData[], double *altitude)
{
  xAngleSP = xAngle;
  yAngleSP = yAngle;
  headingSP = heading + declination;
  updateCorrection(accelData, gyroData, magData, altitude);
}

void lowPassFilter(double newData[], double oldData[], double lowPassConstant, int length)
{
  for(int i = 0; i < length; i++)
  {
    oldData[i] = newData[i] = (newData[i] + lowPassConstant * oldData[i]) / (lowPassConstant + 1);
  }
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
      readI2CCompass(mag);
      updateSetPointAndCorrection(0, 0, 0, 0, accel, gyro, mag, &altitude);
      
      while(digitalInput(_PIN7))
      {
        
        X_PROPORTIONAL_GAIN = analogInput(_A0) / 50;
        X_INTEGRAL_GAIN = 0;
        X_DERIVATIVE_GAIN = 0;

        
        readI2CAccelerometer(accel);
        readI2CGyroscope(gyro);
        readI2CCompass(mag);
        readI2CBarometer(&altitude);
        
        lowPassFilter(accel, oldAccel, ACCELEROMETER_LOW_PASS_FILTER_CONSTANT, 3);
        lowPassFilter(gyro, oldGyro, GYROSCOPE_LOW_PASS_FILTER_CONSTANT, 3);
        lowPassFilter(mag, oldMag, MAGNOMETER_LOW_PASS_FILTER_CONSTANT, 3);
        lowPassFilter(&altitude, &oldAltitude, BAROMETER_LOW_PASS_FILTER_CONSTANT, 3);
        
        updateCorrection(accel, gyro, mag, &altitude);

        vCorrection = 0;
        hCorrection = 0;
        yCorrection = 0;
        
        setMotors(vCorrection + hCorrection - xCorrection, vCorrection + hCorrection + xCorrection, vCorrection - hCorrection + yCorrection, vCorrection - hCorrection - yCorrection);
      }
    }
    stopMotors();
  }
}

void loop(){}
