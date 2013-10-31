#ifndef QUADCOPTER_H_INCLUDED
#define QUADCOPTER_H_INCLUDED

#include <avr/delay.h>
#include <avr/interrupt.h>
#include "senStick.h"

//Defined based on the processor's frequency
#define F_CPU 16000000l

//The highest value initializePWM will take as a value
#define MAX_PWM_FREQUENCY 1000000l

#define NUM_COMPASS_REGISTERS 13
#define NUM_ACCELEROMETER_REGISTERS 58
#define NUM_GYROSCOPE_REGISTERS 63

//These enums' values correspond to the bit in the DDRX, PORTX, and PINX registers that is associated with that pin
enum BPin {_SS = 0b00000001, _SCK = 0b00000010, _MOSI = 0b00000100, _MI = 0b00001000, _PIN8 = 0b00010000, _PIN9 = 0b00100000, _PIN10 = 0b01000000, _PIN11 = 0b10000000};
enum CPin {_PIN5 = 0b01000000, _PIN13 = 0b10000000};
enum DPin {_PIN3 = 0b00000001, _PIN2 = 0b00000010, _RX = 0b00000100, _TX = 0b00001000, _PIN4 = 0b00010000, _PIN12 = 0b01000000, _PIN6 = 0b10000000};
enum EPin {_PIN7 = 0b01000000};

//Different because it's more convenient for analog to digital conversion
enum FPin {_A5 = 0, _A4 = 1, _A3 = 4, _A2 = 5, _A1 = 6, _A0 = 7};

//self explanatory; used so there's need to do error checking
enum Voltage {_LOW, _HIGH};
enum Direction {_INPUT, _OUTPUT};

//corresponds to the bits needed to change in TCCRnA.
enum PWM_TC1 {_PIN9PWM = 0b10000000, _PIN10PWM = 0b00100000, _PIN11PWM = 0b00001000};
enum PWM_TC3 {_PIN5PWM = 0b10000000};

//used in calculating PWM output, set in initializePWM
unsigned int maxDuty;

//used in initializePWM; Set values before calling
unsigned int compassRegister[NUM_COMPASS_REGISTERS];
unsigned int accelerometerRegister[NUM_ACCELEROMETER_REGISTERS];
unsigned int gyroscopeRegister[NUM_GYROSCOPE_REGISTERS];

/*
The setDirection methods set each pin as either input or output.

set up so that constants like _PIN4 or _PIN5 can be used without needing to know what port they are on (B, C, D, etc.).
*/
void setDirection(enum BPin p, enum Direction d);
void setDirection(enum CPin p, enum Direction d);
void setDirection(enum DPin p, enum Direction d);
void setDirection(enum EPin p, enum Direction d);
void setDirection(enum FPin p, enum Direction d);

/*
The digitalOutput methods set each pin as either high or low. The pin must be configured to output (using the setDirection method or otherwise) before there is any physical change.

set up so that constants like _PIN4 or _PIN5 can be used without needing to know what port they are on (B, C, D, etc.).
*/
void digitalOutput(enum BPin p, enum Voltage v);
void digitalOutput(enum CPin p, enum Voltage v);
void digitalOutput(enum DPin p, enum Voltage v);
void digitalOutput(enum EPin p, enum Voltage v);
void digitalOutput(enum FPin p, enum Voltage v);

/*
The digitalInput methods check if each pin is either high or low. The pin must be configured to input (using the setDirection method or otherwise) before accurate readings can be made.

returns a 1 if high and a 0 if low

set up so that constants like _PIN4 or _PIN5 can be used without needing to know what port they are on (B, C, D, etc.).
*/
unsigned int digitalInput(enum BPin p);
unsigned int digitalInput(enum CPin p);
unsigned int digitalInput(enum DPin p);
unsigned int digitalInput(enum EPin p);
unsigned int digitalInput(enum FPin p);

/*
Sets up the four 16-bit timers to output PWM
Takes a frequency in Hz (may not be 100% accurate for some frequencies. error worsens with higher frequencies)

returns 0 if it completed successfully, -1 if the frequency is out of range. (0 to MAX_PWM_FREQUENCY)
*/
unsigned int initializePWM(double frequency);

/*
Starts sending PWM output

sets a number between 0.0 and 1.0 as the duty cycle

requires the pin to be set as output

_PIN5 and _PIN5PWM are not interchangeable even though they correspond to the same pin. same for the other PWM pins.

set up so that constants like _PIN5PWM or _PIN9PWM can be used without needing to know what timer they are on (1 or 3).
*/
void PWMOutput(enum PWM_TC1 p, double duty);
void PWMOutput(enum PWM_TC3 p, double duty);

/*
Stops sending PWM output from the specified pin.

_PIN5 and _PIN5PWM are not interchangeable even though they correspond to the same pin. same for the other PWM pins.

set up so that constants like _PIN5PWM or _PIN9PWM can be used without needing to know what timer they are on (1 or 3).
*/
void stopPWM(enum PWM_TC1 p);
void stopPWM(enum PWM_TC3 p);

/*
Reads the voltage on one of the analog input pins

requires the pin to be set as input
*/
float analogInput(enum FPin p);
float analogInput(enum FPin p, float AREFVoltage);

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

If resetSetupRegisters() is not called, zeros will be written to all the registers.
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
int readI2CCompass(int axes[]);

/*
Reads from the accelerometer. Writes x, y, and z values into the array passed. axes[0] == x, axes[1] == y, axes[2] == z.
Important that the array passed has at least 3 elements or else undefined things may occur in the program.

returns 1 if successful, a status code if unsuccessful. Corresponds to the status codes from TWSR in the ATmega32u4 datasheet.
*/
int readI2CAccelerometer(int axes[]);

/*
Reads from the gyroscope. Writes x, y, and z values into the array passed. axes[0] == x, axes[1] == y, axes[2] == z.
Important that the array passed has at least 3 elements or else undefined things may occur in the program.
Compensates for temperature using experimental data.
(real x gyro value) = (x gyro value) + 0.00266313 * temperature + 142.52507234
(real y gyro value) = (y gyro value) - 0.00100571 * temperature - 4.60342794
(real z gyro value) = (z gyro value) - 0.00111324 * temperature - 42.44208742

returns 1 if successful, a status code if unsuccessful. Corresponds to the status codes from TWSR in the ATmega32u4 datasheet.
*/
int readI2CGyroscope(int axes[]);

#include "quadcopter.cpp"

#endif
