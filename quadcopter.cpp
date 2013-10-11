#include "quadcopter.h"
#include <avr/delay.h>
#include <avr/interrupt.h>
#include <Arduino.h>

void setDirection(enum BPin p, enum Direction d)
{
	if(d == _INPUT)
		DDRB &= ~p;
	else
		DDRB |= p;
}

void setDirection(enum CPin p, enum Direction d)
{
	if(d == _INPUT)
		DDRC &= ~p;
	else
		DDRC |= p;
}

void setDirection(enum DPin p, enum Direction d)
{
	if(d == _INPUT)
		DDRD &= ~p;
	else
		DDRD |= p;
}

void setDirection(enum EPin p, enum Direction d)
{
	if(d == _INPUT)
		DDRE &= ~p;
	else
		DDRE |= p;
}

void setDirection(enum FPin p, enum Direction d)
{
	if(d == _INPUT)
		DDRF &= ~p;
	else
		DDRF |= p;
}

void digitalOutput(enum BPin p, enum Voltage v)
{
	if(v == _LOW)
		PORTB &= ~p;
	else
		PORTB |= p;
}

void digitalOutput(enum CPin p, enum Voltage v)
{
	if(v == _LOW)
		PORTC &= ~p;
	else
		PORTC |= p;
}

void digitalOutput(enum DPin p, enum Voltage v)
{
	if(v == _LOW)
		PORTD &= ~p;
	else
		PORTD |= p;
}

void digitalOutput(enum EPin p, enum Voltage v)
{
	if(v == _LOW)
		PORTE &= ~p;
	else
		PORTE |= p;
}

void digitalOutput(enum FPin p, enum Voltage v)
{
	if(v == _LOW)
		PORTF &= ~p;
	else
		PORTF |= p;
}

unsigned int digitalInput(enum BPin p)
{
	if(PINB & p)
		return 1;
	else
		return 0;
}

unsigned int digitalInput(enum CPin p)
{
	if(PINC & p)
		return 1;
	else
		return 0;
}

unsigned int digitalInput(enum DPin p)
{
	if(PIND & p)
		return 1;
	else
		return 0;
}

unsigned int digitalInput(enum EPin p)
{
	if(PINE & p)
		return 1;
	else
		return 0;
}

unsigned int digitalInput(enum FPin p)
{
	if(PINF & p)
		return 1;
	else
		return 0;
}

int initializePWM(unsigned int frequency)
{
	if(frequency > 1000000 || frequency < 1)
		return -1;
		
	unsigned int prescaler;
	if(frequency > 31)
		prescaler = 8;
	else if(frequency > 4)
		prescaler = 64;
	else
		prescaler = 256;
		
	maxDuty = 16000000 / (frequency * prescaler);
	
	TCCR1A = 0b00000010;
	TCCR3A = 0b00000010;
	
	if(prescaler == 8)
	{
		TCCR1B = 0b00011010;
		TCCR3B = 0b00011010;
	}
	else if(prescaler == 64)
	{
		TCCR1B = 0b00011011;
		TCCR3B = 0b00011011;
	}
	else
	{
		TCCR1B = 0b00011100;
		TCCR3B = 0b00011100;
	}
	
	TCCR1C = 0b00000000;
	TCCR3C = 0b00000000;
	
	unsigned char sreg;
	
	sreg = SREG;
	cli();
	ICR1 = maxDuty;
	ICR3 = maxDuty;
	SREG = sreg;
	
	TIMSK1 = 0b00000000;
	TIMSK3 = 0b00000000;
	
	return 0;
}

void PWMOutput(enum PWM16Bit p, double duty)
{
	if(duty > 1.0)
		duty = 1.0;
	else if(duty < 0.0)
		duty = 0.0;
	
	unsigned int val = (unsigned int)(duty * maxDuty + 0.5);
	unsigned char sreg;
	
	if(p == _PIN9PWM)
	{
		sreg = SREG;
		cli();
		OCR1A = val;
		SREG = sreg;
		TCCR1A |= p;
		TCCR1A &= ~(p >> 1);
	}
	else if(p == _PIN10PWM)
	{
		sreg = SREG;
		cli();
		OCR1B = val;
		SREG = sreg;
		TCCR1A |= p;
		TCCR1A &= ~(p >> 1);
	}
	else if(p == _PIN11PWM)
	{
		sreg = SREG;
		cli();
		OCR1C = val;
		SREG = sreg;
		TCCR1A |= p;
		TCCR1A &= ~(p >> 1);
	}
	else if(p == _PIN5PWM)
	{
		sreg = SREG;
		cli();
		OCR3A = val;
		SREG = sreg;
		TCCR3A |= 0b10000000;
		TCCR3A &= 0b10111111;
	}
}

void stopPWM(enum PWM16Bit p)
{
	if(p == _PIN9PWM || p == _PIN10PWM || p == _PIN11PWM)
		TCCR1A &= ~p;
	else if(p == _PIN5PWM)
		TCCR3A &= 0b01111111;
}
