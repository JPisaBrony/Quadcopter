#ifndef QUADCOPTER_H_INCLUDED
#define QUADCOPTER_H_INCLUDED

#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "senStick.h"

//The highest value initializePWM will take as a value
#define MAX_PWM_FREQUENCY 1000000l

#define NUM_COMPASS_REGISTERS 13
#define NUM_ACCELEROMETER_REGISTERS 58
#define NUM_GYROSCOPE_REGISTERS 63

#define UPPER_LIMIT 0.075
#define LOWER_LIMIT 0.035

#define BAUD_RATE_4800_VALUE_H (F_CPU/76800 - 1) / 256  
#define BAUD_RATE_4800_VALUE_L (F_CPU/76800 - 1) % 256

//These enums' values correspond to the bit in the DDRX, PORTX, and PINX registers that is associated with that pin
typedef enum BPin {_SS = 0b00000001, _SCK = 0b00000010, _MOSI = 0b00000100, _MI = 0b00001000, _PIN8 = 0b00010000, _PIN9 = 0b00100000, _PIN10 = 0b01000000, _PIN11 = 0b10000000} BPin;
typedef enum CPin {_PIN5 = 0b01000000, _PIN13 = 0b10000000} CPin;
typedef enum DPin {_PIN3 = 0b00000001, _PIN2 = 0b00000010, _RX = 0b00000100, _TX = 0b00001000, _PIN4 = 0b00010000, _PIN12 = 0b01000000, _PIN6 = 0b10000000} DPin;
typedef enum EPin {_PIN7 = 0b01000000} EPin;
typedef enum FPin {_A5 = 0, _A4 = 1, _A3 = 4, _A2 = 5, _A1 = 6, _A0 = 7} FPin;//Different because it's more convenient for analog to digital conversion

//self explanatory; used so there's need to do error checking
typedef enum Voltage {_LOW, _HIGH} Voltage;
typedef enum Direction {_INPUT, _OUTPUT} Direction;

//corresponds to the bits needed to change in TCCRnA.
typedef enum PWM_TC1 {_PIN9PWM = 0b10000000, _PIN10PWM = 0b00100000, _PIN11PWM = 0b00001000} PWM_TC1;
typedef enum PWM_TC3 {_PIN5PWM = 0b10000000} PWM_TC3;

//used in calculating PWM output, set in initializePWM
unsigned int maxDuty1;
unsigned int maxDuty3;

//used in initializeTWI; Set values before calling
unsigned int compassRegister[NUM_COMPASS_REGISTERS];
unsigned int accelerometerRegister[NUM_ACCELEROMETER_REGISTERS];
unsigned int gyroscopeRegister[NUM_GYROSCOPE_REGISTERS];

//retrived in initializeTWI. Represents the individual coefficients of the sensor being used. Used in the calculation of altitude in readI2CBarometer
long barometerCoefficient[11];
#define bcAC1 0
#define bcAC2 1
#define bcAC3 2
#define bcAC4 3
#define bcAC5 4
#define bcAC6 5
#define bcB1 6
#define bcB2 7
#define bcMB 8
#define bcMC 9
#define bcMD 10
#define STANDARD_SEA_LEVEL_PRESSURE 101325.0
double initialAltitude;
double accelerationDueToGravity;

//used in the timer0 interupt. counts the number of units time since it was last reset.
unsigned long timeDifferential;

//Stores the packet of information currently being received from the serial port
char currentGPSPacket[255];//packets are guaranteed not to exceed 255 bytes
unsigned char currentGPSPacketPosition;

//stores the last data received from the GPS; It is unprocessed
double rawUTCTime;
char rawStatus;
double rawLatitude;
char rawNSIndicator;
double rawLongitude;
char rawEWIndicator;
double rawSpeedOverGround;
double rawCourseOverGround;
int rawDate;

double utcTime;
double latitude;
double longitude;
double speedOverGround;
double courseOverGround;
double northSpeedOverGround;
double eastSpeedOverGround;
int date;

/*
The setDirection methods set each pin as either input or output.

set up so that constants like _PIN4 or _PIN5 can be used without needing to know what port they are on (B, C, D, etc.).
*/
void setDirection(BPin p, Direction d);
void setDirection(CPin p, Direction d);
void setDirection(DPin p, Direction d);
void setDirection(EPin p, Direction d);
void setDirection(FPin p, Direction d);

/*
The digitalOutput methods set each pin as either high or low. The pin must be configured to output (using the setDirection method or otherwise) before there is any physical change.

set up so that constants like _PIN4 or _PIN5 can be used without needing to know what port they are on (B, C, D, etc.).
*/
void digitalOutput(BPin p, Voltage v);
void digitalOutput(CPin p, Voltage v);
void digitalOutput(DPin p, Voltage v);
void digitalOutput(EPin p, Voltage v);
void digitalOutput(FPin p, Voltage v);

/*
The digitalInput methods check if each pin is either high or low. The pin must be configured to input (using the setDirection method or otherwise) before accurate readings can be made.

returns a 1 if high and a 0 if low

set up so that constants like _PIN4 or _PIN5 can be used without needing to know what port they are on (B, C, D, etc.).
*/
unsigned int digitalInput(BPin p);
unsigned int digitalInput(CPin p);
unsigned int digitalInput(DPin p);
unsigned int digitalInput(EPin p);
unsigned int digitalInput(FPin p);

/*
Sets up the four 16-bit timers to output PWM
Takes a frequency in Hz (may not be 100% accurate for some frequencies. error worsens with higher frequencies)

initializePWM1 sets timer 1,
initializePWM3 sets timer 3,
initializePWM sets both

returns 0 if it completed successfully, -1 if the frequency is out of range. (0 to MAX_PWM_FREQUENCY)
*/
int initializePWM(double frequency);
int initializePWM1(double frequency);
int initializePWM3(double frequency);

/*
Starts sending PWM output

sets a number between 0.0 and 1.0 as the duty cycle

requires the pin to be set as output

_PIN5 and _PIN5PWM are not interchangeable even though they correspond to the same pin. same for the other PWM pins.

set up so that constants like _PIN5PWM or _PIN9PWM can be used without needing to know what timer they are on (1 or 3).
*/
void PWMOutput(PWM_TC1 p, double duty);
void PWMOutput(PWM_TC3 p, double duty);

/*
Stops sending PWM output from the specified pin.

_PIN5 and _PIN5PWM are not interchangeable even though they correspond to the same pin. same for the other PWM pins.

set up so that constants like _PIN5PWM or _PIN9PWM can be used without needing to know what timer they are on (1 or 3).
*/
void stopPWM(PWM_TC1 p);
void stopPWM(PWM_TC3 p);

/*
Reads the voltage on one of the analog input pins

requires the pin to be set as input
*/
double analogInput(FPin p);
double analogInput(FPin p, double AREFVoltage);

/*
Converts two chars into a signed int.
*/
int twoCharToInt(unsigned char high, unsigned char low);

/*
Configures the TWI bit rate. values less than 256 in compassRegister, accelerometerRegister, and gyroscopeRegister will be written to their corresponding register in the sensor stick

Example for initialization:

resetSetupRegisters();
compassRegister[magCRA] = 0x10;
initializeTWI(100, 0);

Sets the TWBR register to 100 and Configuration Register A in the magnetic compass to 0x10.

If resetSetupRegisters() is not called, zeros will be written to all the registers. (or previous values in the arrays);
*/
int initializeTWI(unsigned char TWIBitRate, unsigned char TWIBitRatePrescaler);

/*
sets all the values in compassRegister, accelerometerRegister, and gyroscopeRegister to 256
*/
void resetSetupRegisters();

/*
Writes to a slave device connected by an I2C connection. Also referred to as a two wire interface.
The address is the 7 bit number that identifies the slave. Separate from the read/write bit (0x1E becomes 0x3C and 0x3D with the read or write bit included. address wants 0x1E). (address << 1) | readWrite
buffer is the array containing the values that will be written to the slave device.
length is the number of values from buffer to write to the slave device. Undefined results if length is greater than the actual length of the array.

returns 1 if successful, a status code if unsuccessful. Corresponds to the status codes from TWSR in the ATmega32u4 datasheet.
*/
int writeI2C(unsigned char address, unsigned char buffer[], unsigned int length);

/*
Reads from a slave device connected by an I2C connection. Also referred to as a two wire interface.
The address is the 7 bit number that identifies the slave. Separate from the read/write bit (0x1E becomes 0x3C and 0x3D with the read or write bit included. address wants 0x1E). (address << 1) | readWrite
buffer is the array that read values will be written into.
length is the number of values from buffer to read from the slave device. Undefined results if length is greater than the actual length of the array.

returns 1 if successful, a status code if unsuccessful. Corresponds to the status codes from TWSR in the ATmega32u4 datasheet.
*/
int readI2C(unsigned char address, unsigned char buffer[], unsigned int length);

/*
Reads from the compass. Writes x, y, and z values into the array passed. axes[0] == x, axes[1] == y, axes[2] == z.
Important that the array passed has at least 3 elements or else undefined things may occur in the program.

returns 1 if successful, a status code if unsuccessful. Corresponds to the status codes from TWSR in the ATmega32u4 datasheet.
*/
int readI2CCompass(double axes[]);

/*
Reads from the accelerometer. Writes x, y, and z values into the array passed. axes[0] == x, axes[1] == y, axes[2] == z.
Important that the array passed has at least 3 elements or else undefined things may occur in the program.

returns 1 if successful, a status code if unsuccessful. Corresponds to the status codes from TWSR in the ATmega32u4 datasheet.
*/
int readI2CAccelerometer(double axes[]);

/*
Reads from the gyroscope. Writes x, y, and z values into the array passed. axes[0] == x, axes[1] == y, axes[2] == z.
Important that the array passed has at least 3 elements or else undefined things may occur in the program.
Compensates for temperature using experimental data.

returns 1 if successful, a status code if unsuccessful. Corresponds to the status codes from TWSR in the ATmega32u4 datasheet.
*/
int readI2CGyroscope(double axes[]);

/*
Reads the temperature and pressure from the Barometer. Calculates the approximate altitude and writes it to the address of altitude.
*/
int readI2CBarometer(double* elevation);

/*
Configures the registers in the USART to receive data from a GPS at 4800 bps
*/
void initializeGPS();

/*
Called to read data from the serial buffer and put it into the raw___ variables
*/
void updateGPSData();

/*
Called by updateGPSData
No need to directly call this function
*/
void parseGPSPacket();

/*
initializes everything that the quadcopter needs to start flying
*/
int initializeQuadcopter();

/*
sets the duty cycle on all four motors at once
*/
void setMotors(double xp, double xn, double yp, double yn);

/*
sends a signal that is below the threshold for the motors so they don't move but they're still receiving signal
*/
void stopMotors();

#include "quadcopter.cpp"

#endif
