#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

double X_PROPORTIONAL_GAIN = 0.0635386117;
double X_INTEGRAL_GAIN = 0.00006989247287;
double X_DERIVATIVE_GAIN = 38.50824951515113006902;
double Y_PROPORTIONAL_GAIN = 0.0374103943;
double Y_INTEGRAL_GAIN = 2.3828276645436E-5;
double Y_DERIVATIVE_GAIN = 25.1026401519;
double HEADING_PROPORTIONAL_GAIN = 0;
double HEADING_INTEGRAL_GAIN = 0;
double HEADING_DERIVATIVE_GAIN = 0;
double ALTITUDE_PROPORTIONAL_GAIN = 0;
double ALTITUDE_INTEGRAL_GAIN = 0;
double ALTITUDE_DERIVATIVE_GAIN = 0;

double ACCELEROMETER_LOW_PASS_FILTER_CONSTANT = 0.1;
double GYROSCOPE_LOW_PASS_FILTER_CONSTANT = 0.5;
double MAGNOMETER_LOW_PASS_FILTER_CONSTANT = 0.25;
double BAROMETER_LOW_PASS_FILTER_CONSTANT = 3;

double declination = 0;

double accel[3], gyro[3], mag[3], altitude, oldAccel[3], oldGyro[3], oldMag[3], oldAltitude;

double xAngleSP, yAngleSP, headingSP, altitudeSP;
double xCorrection, yCorrection, hCorrection, aCorrection;

void updateCorrection(double accelData[], double gyroData[], double magData[], double *altitude)
{
  unsigned long time = timeDifferential;
  timeDifferential = 0;
  static double xAnglePV, integralXAngleError;
  static double yAnglePV, integralYAngleError;
  static double headingPV, integralHeadingError;
  static double altitudePV, previousAltitudePV = *altitude, integralAltitudeError;

  xAnglePV = atan2(accelData[0], accelData[2]);
  yAnglePV = atan2(accelData[1], accelData[2]);
  headingPV = (PI / 2) - atan2(accelData[2] * magData[0] - accelData[0] * magData[2], accelData[1] * magData[2] - accelData[2] * magData[1]);
  if(headingPV > PI)
    headingPV -= 2 * PI;
  altitudePV = *altitude;

  double xAngleError = xAnglePV - xAngleSP;
  double yAngleError = yAnglePV - yAngleSP;
  double headingError = headingPV - headingSP;
  double altitudeError = altitudePV - altitudeSP;

  double xP = X_PROPORTIONAL_GAIN * xAngleError;
  double yP = Y_PROPORTIONAL_GAIN * yAngleError;
  double hP = HEADING_PROPORTIONAL_GAIN * headingError;
  double aP = ALTITUDE_PROPORTIONAL_GAIN * altitudeError;

  double xDerivative = -1.5E-6 * gyroData[1];
  double yDerivative = 1.5E-6 * gyroData[0];
  double headingDerivative = 1.5E-6 * gyroData[2];
  double altitudeDerivative = (altitudeError - previousAltitudePV)/time;

  double xD = X_DERIVATIVE_GAIN * xDerivative;
  double yD = Y_DERIVATIVE_GAIN * yDerivative;
  double hD = HEADING_DERIVATIVE_GAIN * headingDerivative;
  double aD = ALTITUDE_DERIVATIVE_GAIN * altitudeDerivative;

  integralXAngleError += xAngleError * time;
  integralYAngleError += yAngleError * time;
  integralHeadingError += headingError * time;
  integralAltitudeError += altitudeError * time;

  double xI = X_INTEGRAL_GAIN * integralXAngleError;
  double yI = Y_INTEGRAL_GAIN * integralYAngleError;
  double hI = HEADING_INTEGRAL_GAIN * integralHeadingError;
  double aI = ALTITUDE_INTEGRAL_GAIN * integralAltitudeError;
  
  xCorrection = xP + xI + xD;
  yCorrection = yP + yI + yD;
  hCorrection = hP + hI + hD;
  aCorrection = aP + aI + aD;
}

void updateSetPointAndCorrection(double xAngle, double yAngle, double heading, double targetAltitude, double accelData[], double gyroData[], double magData[], double *altitude)
{
  xAngleSP = xAngle;
  yAngleSP = yAngle;
  headingSP = heading + declination;
  altitudeSP = targetAltitude;
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

        HEADING_PROPORTIONAL_GAIN = analogInput(_A0) / 10;
        HEADING_INTEGRAL_GAIN = 0;
        HEADING_DERIVATIVE_GAIN = 0;


        readI2CAccelerometer(accel);
        readI2CGyroscope(gyro);
        readI2CCompass(mag);
        readI2CBarometer(&altitude);

        lowPassFilter(accel, oldAccel, ACCELEROMETER_LOW_PASS_FILTER_CONSTANT, 3);
        lowPassFilter(gyro, oldGyro, GYROSCOPE_LOW_PASS_FILTER_CONSTANT, 3);
        lowPassFilter(mag, oldMag, MAGNOMETER_LOW_PASS_FILTER_CONSTANT, 3);
        lowPassFilter(&altitude, &oldAltitude, BAROMETER_LOW_PASS_FILTER_CONSTANT, 1);

        updateCorrection(accel, gyro, mag, &altitude);

        aCorrection = 0.5;
        hCorrection = 0;
        yCorrection = 0;

        setMotors(aCorrection - hCorrection - xCorrection, aCorrection - hCorrection + xCorrection, aCorrection + hCorrection - yCorrection, aCorrection + hCorrection + yCorrection);
      }
    }
    stopMotors();
  }
}

void loop(){}
