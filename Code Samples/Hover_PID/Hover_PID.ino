#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

double X_PROPORTIONAL_GAIN;
double X_INTEGRAL_GAIN;
double X_DERIVATIVE_GAIN;
double Y_PROPORTIONAL_GAIN = 0.0374103943;
double Y_INTEGRAL_GAIN = 2.3828276645436E-5;
double Y_DERIVATIVE_GAIN = 25.1026401519;
double HEADING_PROPORTIONAL_GAIN;
double HEADING_INTEGRAL_GAIN;
double HEADING_DERIVATIVE_GAIN;
double ACCELERATION_PROPORTIONAL_GAIN;
double ACCELERATION_INTEGRAL_GAIN;
double ACCELERATION_DERIVATIVE_GAIN;

double ACCELEROMETER_LOW_PASS_FILTER_CONSTANT;
double GYROSCOPE_LOW_PASS_FILTER_CONSTANT;
double MAGNOMETER_LOW_PASS_FILTER_CONSTANT;


double declination;
double accelerationDueToGravity;

double accel[3], gyro[3], mag[3], oldAccel[3], oldGyro[3], oldMag[3];

double xAngleSP, yAngleSP, headingSP, accelerationSP;
double xCorrection, yCorrection, hCorrection, aCorrection;

void updateCorrection(double accelData[], double gyroData[], double magData[])
{
  unsigned long time = timeDifferential;
  timeDifferential = 0;
  static double xAnglePV, integralXAngleError;
  static double yAnglePV, integralYAngleError;
  static double headingPV, integralHeadingError;
  static double accelerationPV, integralAccelerationError, previousAccelerationPV = 0;
  
  xAnglePV = atan2(accelData[0], accelData[2]);
  yAnglePV = atan2(accelData[1], accelData[2]);
  headingPV = atan2(-magData[1], magData[0]);
  accelerationPV = sqrt(accelData[0] * accelData[0] + accelData[1] * accelData[1] + accelData[2] * accelData[2]);
  
  double xAngleError = xAnglePV - xAngleSP;
  double yAngleError = yAnglePV - yAngleSP;
  double headingError = headingPV - headingSP;
  double accelerationError = accelerationPV - accelerationSP;
  
  double xP = X_PROPORTIONAL_GAIN * xAngleError;
  double yP = Y_PROPORTIONAL_GAIN * yAngleError;
  double hP = HEADING_PROPORTIONAL_GAIN * headingError;
  double aP = ACCELERATION_PROPORTIONAL_GAIN * accelerationError;
  
  double xDerivative = 1.5E-6 * gyroData[1];
  double yDerivative = 1.5E-6 * gyroData[0];
  double headingDerivative = 1.5E-6 * gyroData[2];
  double accelerationDerivative;
  if(time > 0)
  {
    accelerationDerivative = (accelerationPV - previousAccelerationPV) / time;
    previousAccelerationPV = accelerationPV;
  }
  
  double xD = X_DERIVATIVE_GAIN * xDerivative;
  double yD = Y_DERIVATIVE_GAIN * yDerivative;
  double hD = HEADING_DERIVATIVE_GAIN * headingDerivative;
  double aD = ACCELERATION_DERIVATIVE_GAIN * accelerationDerivative;
  
  integralXAngleError += xAngleError * time;
  integralYAngleError += yAngleError * time;
  integralHeadingError += headingError * time;
  integralAccelerationError += accelerationError * time;
  
  double xI = X_INTEGRAL_GAIN * integralXAngleError;
  double yI = Y_INTEGRAL_GAIN * integralYAngleError;
  double hI = HEADING_INTEGRAL_GAIN * integralHeadingError;
  double aI = ACCELERATION_INTEGRAL_GAIN;
  
  xCorrection = xP + xI + xD;
  yCorrection = yP + yI + yD;
  hCorrection = hP + hI + hD;
  aCorrection = aP + aI + aD;

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

void updateSetPointAndCorrection(double xAngle, double yAngle,  double heading, double acceleration, double accelData[], double gyroData[], double magData[])
{
  xAngleSP = xAngle;
  yAngleSP = yAngle;
  headingSP = heading + declination;
  accelerationSP = acceleration + accelerationDueToGravity;
  updateCorrection(accelData, gyroData, magData);
}

void lowPassFilter(double newData[], double oldData[], double lowPassConstant)
{
  oldData[0] = newData[0] = (newData[0] + lowPassConstant * oldData[0]) / (lowPassConstant + 1);
  oldData[1] = newData[1] = (newData[1] + lowPassConstant * oldData[1]) / (lowPassConstant + 1);
  oldData[2] = newData[2] = (newData[2] + lowPassConstant * oldData[2]) / (lowPassConstant + 1);
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
      updateSetPointAndCorrection(0, 0, 0, 0, accel, gyro, mag);
      
      while(digitalInput(_PIN7))
      {
        
        X_PROPORTIONAL_GAIN = analogInput(_A0) / 10;
        X_INTEGRAL_GAIN = 0;
        X_DERIVATIVE_GAIN = 0;

        
        readI2CAccelerometer(accel);
        readI2CGyroscope(gyro);
        readI2CCompass(mag);
        lowPassFilter(accel, oldAccel, ACCELEROMETER_LOW_PASS_FILTER_CONSTANT);
        lowPassFilter(gyro, oldGyro, GYROSCOPE_LOW_PASS_FILTER_CONSTANT);
        lowPassFilter(mag, oldMag, MAGNOMETER_LOW_PASS_FILTER_CONSTANT);
        
        updateCorrection(accel, gyro, mag);
                
        setMotors(0.5 - xCorrection, 0.5 + xCorrection, 0.5 + yCorrection, 0.5 - yCorrection);
        
      }
    }
    stopMotors();
  }
}

void loop(){}
