#ifndef QUADCOPTER_H_INCLUDED
#define QUADCOPTER_H_INCLUDED

#include <avr/delay.h>
#include <avr/interrupt.h>
#include <Arduino.h>

//Defined based on the processor's frequency
#define F_CPU 16000000l

//The highest value initializePWM will take as a value
#define MAX_PWM_FREQUENCY 1000000l

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
enum PWM_TC3 {_PIN5PWM = 0b00000000};

//used in calculating PWM output, set in initializePWM
unsigned int maxDuty;

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

#include "quadcopter.cpp"

#endif