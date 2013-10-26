#ifndef QUADCOPTER_CPP_INCLUDED
#define QUADCOPTER_CPP_INCLUDED

//I'm only going to comment on the first function of each group. The only difference is changing the affected port.

void setDirection(enum BPin p, enum Direction d)
{
	//checks if the direction is input or output
	if(d == _INPUT)
		//if input, makes the bit corresponding to the desired pin 0 while leaving the rest unaffected.
		DDRB &= ~p; //if affecting PB5 (_PIN9), p will have a value of 0b00100000 making it convenient for this operation.
	else
		//if output, makes the bit corresponding to the desired pin 1 while leaving the rest unaffected.
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
		DDRF &= ~(1 << p);//different because of analog to digital conversion
	else
		DDRF |= (1 << p);//different because of analog to digital conversion
}

void digitalOutput(enum BPin p, enum Voltage v)
{
	//checks if the voltage should be high or low.
	if(v == _LOW)
		//if low, makes the bit corresponding to the desired pin 0 while leaving the rest unaffected.
		PORTB &= ~p;//if affecting PB5 (_PIN9), p will have a value of 0b00100000 making it convenient for this operation.
	else
		//if high, makes the bit corresponding to the desired pin 1 while leaving the rest unaffected.
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
		PORTF &= ~(1 << p);//different because of analog to digital conversion
	else
		PORTF |= (1 << p);//different because of analog to digital conversion
}

unsigned int digitalInput(enum BPin p)
{
	//anding PINB and p gives 0 if the the corresponding pin is low and the value of p if it is high. not-equalling it with zero makes it always 1 or 0. 
	return (PINB & p) != 0;
}

unsigned int digitalInput(enum CPin p)
{
	return (PINC & p) != 0;
}

unsigned int digitalInput(enum DPin p)
{
	return (PIND & p) != 0;
}

unsigned int digitalInput(enum EPin p)
{
	return (PINE & p) != 0;
}

unsigned int digitalInput(enum FPin p)
{
	return (PINF & (1 << p)) != 0;//different because of analog to digital conversion
}

unsigned int initializePWM(double frequency)
{
	//if the frequency is out of range, return a -1. Just an error check.
	if(frequency > MAX_PWM_FREQUENCY || frequency <= 0)
		return -1;
	
	//Selects the smallest prescaler for the frequency. The smallest prescaler means the highest resolution
	unsigned int prescaler;
	if(frequency > F_CPU / 524288L)
		prescaler = 8;
	else if(frequency > F_CPU / 4194304L)
		prescaler = 64;
	else if(frequency > F_CPU / 16777216L)
		prescaler = 256;
	else
		prescaler = 1024;
	
	//calculates the value the counter should reach before resetting (rounded to the nearest integer)
	maxDuty = (unsigned int)(F_CPU / (frequency * prescaler) + 0.5);
	
	//clears any interrupts from the timers
	TIMSK1 = 0b00000000;
	TIMSK3 = 0b00000000;
	
	TCCR1A = 0b00000010;//disconnects output to all Timer/Counter1 pins (Bit 7:2), sets the waveform generation mode to mode 14 (FastPWM with ICR1 as the top value) (Bit 1:0)
	TCCR3A = 0b00000010;//same as above but for Timer/Counter3
	
	if(prescaler == 8)
	{
		TCCR1B = 0b00011010;//Turns off noise cancellation and input capture edge select (only applicable to input) (Bit 7:6), Bit 5 is reserved, sets the waveform generation mode to mode 14 (FastPWM with ICR1 as the top value) (Bit 4:3), and sets the appropriate prescaler.
		TCCR3B = 0b00011010;//same as above but for Timer/Counter3
	}
	else if(prescaler == 64)//the following conditions are the same as the previous condition but with a different prescalers;
	{
		TCCR1B = 0b00011011;
		TCCR3B = 0b00011011;
	}
	else if(prescaler == 256)
	{
		TCCR1B = 0b00011100;
		TCCR3B = 0b00011100;
	}
	else
	{
		TCCR1B = 0b00011101;
		TCCR3B = 0b00011101;
	}
	
	
	//The following chunk sets 16-bit registers
	unsigned char sreg;
	sreg = SREG;//saves the interrupt state
	cli();//avr function that clears all the interrupts
	ICR1 = maxDuty;//sets the top counter value for Timer/Counter1
	ICR3 = maxDuty;//and Timer/Counter3 
	SREG = sreg;//sets the original interrupt state
	
	//success
	return 0;
}

void PWMOutput(enum PWM_TC1 p, double duty)
{
	//if duty is between 1 and 0 it is good, otherwise set it to the closest edge
	if(duty > 1.0)
		duty = 1.0;
	else if(duty < 0.0)
		duty = 0.0;
	
	//find the value that represents the closest value to the desired duty cycle (rounded to the nearest integer)
	unsigned int val = (unsigned int)(duty * maxDuty + 0.5);
	
	unsigned char sreg;
	if(p == _PIN9PWM)
	{
		sreg = SREG;//saves the interrupt state
		cli();//avr function that clears all the interrupts
		OCR1A = val; //sets the compare value
		SREG = sreg;//sets the original interrupt state
	}
	else if(p == _PIN10PWM)//Same for the following conditions
	{
		sreg = SREG;
		cli();
		OCR1B = val;
		SREG = sreg;
	}
	else if(p == _PIN11PWM)
	{
		sreg = SREG;
		cli();
		OCR1C = val;
		SREG = sreg;
	}
	
	TCCR1A |= p;//sets one bit high
	TCCR1A &= ~(p >> 1);//and one bit low (0b10 means that the pin is low until the compare value is reached, then it's high)
}

void PWMOutput(enum PWM_TC3 p, double duty)
{
	if(duty > 1.0)
		duty = 1.0;
	else if(duty < 0.0)
		duty = 0.0;
	
	unsigned int val = (unsigned int)(duty * maxDuty + 0.5);
	
	unsigned char sreg;
	if(p == _PIN5PWM)
	{
		sreg = SREG;
		cli();
		OCR3A = val;
		SREG = sreg;
	}
	
	TCCR3A |= p;
	TCCR3A &= ~(p >> 1);
}

void stopPWM(enum PWM_TC1 p)
{
	TCCR1A &= ~p;//disconnects the wave generator
}

void stopPWM(enum PWM_TC3 p)
{
	TCCR3A &= ~p;
}

float analogInput(enum FPin p)
{
	//Disables the digital input buffer on the corresponding pin; reduces power consumption and makes a cleaner reading
	DIDR0 |= 1 << p;
	
	//Writing 0 to bit 0 allows power to go to the ADC
	PRR0 &= ~ 0b1;
	
	//bits 6 & 7 indicate that the reference voltage will be the +5 from the chip. Nothing needs to be plugged into AREF
	//bit 5 indicates the result will have the two highest bits in ADCH and the rest in ADL
	//bit 4 does nothing and will always be zero
	//bits 0 to 3 select which pin the ADC will read from (determined by p)
	ADMUX = 0b01000000 | p;
	
	//bit 7 enables the ADC. Once you're done using the ADC this should theoretically be turned off (and then bit 0 on PRR should be turned on)
	//bit 6 starts the ADC. It will be reset once the operation is complete.
	//bits 3, 4, and 5 deal with interrupts and such
	//bits 0, 1, and 2 determine the clock frequency. 100 means once every 16 clock cycles. This is the fastest frequency with high resolution. 8 and 4 are okay. 2 is bad.
	ADCSRA = 0b11000100;
	
	//wait until the operation is done
	while(ADCSRA & 0b01000000);
	
	//The first one doesn't give good results
	ADCSRA = 0b11000100;
	
	//wait until the operation is done
	while(ADCSRA & 0b01000000);
	
	//Allow access to digital input again. (In case you decide to switch to digital?)
	DIDR0 &= ~ (1 << p);
	
	ADCSRA = 0b00000100;
	
	unsigned int result;
	
	//The following chunk sets 16-bit registers
	unsigned char sreg;
	sreg = SREG;//saves the interrupt state
	cli();//avr function that clears all the interrupts
	result = ADC;
	SREG = sreg;//sets the original interrupt state
	
	//ADC is divided by 1023 to make the result a percentage and multiplied by +5 volts.
	return (result * 5) / 1023.0f;
}

float analogInput(enum FPin p, float AREFVoltage)
{
	DIDR0 |= 1 << p;
	
	PRR0 &= ~ 0b1;
	
	//bits 6 & 7 indicate that the reference voltage will be plugged into AREF
	ADMUX = 0b00000000 | p;
	
	ADCSRA = 0b11000100;
	
	while(ADCSRA & 0b01000000);
	
	ADCSRA = 0b11000100;
	
	while(ADCSRA & 0b01000000);
	
	DIDR0 &= ~ (1 << p);
	
	ADCSRA = 0b00000100;
	
	unsigned int result;
	
	unsigned char sreg;
	sreg = SREG;
	cli();
	result = ADC;
	SREG = sreg;
	
	//ADC is divided by 1023 to make the result a percentage and multiplied by AREFVoltage.
	return (result * AREFVoltage) / 1023.0f;
}

void initializeTWI(unsigned char bitRateValue, unsigned char bitRatePrescaler, unsigned char compassConfigurationA, unsigned char compassConfigurationB, unsigned char compassMode)
{
	//set the bit rate
	TWBR = bitRateValue;
	TWSR = bitRatePrescaler & 0x03;
	
	unsigned char writeBuffer[4] = {0, compassConfigurationA, compassConfigurationB, compassMode};
	
	//write the configuration to the compass's first three registers
	writeI2C(magAddress, writeBuffer, 3);
	
	//more configuration when more sensors are implemented
}
	

int writeI2C(unsigned char address, unsigned char buffer[], unsigned int length)
{
	unsigned char statusCode;//contains the masked status code after each step
	TWCR = 0b11100100;//a 1 needs to be written to the interrupt flag (bit 7) in order to clear it (counter-intuitive); this is the software telling the hardware it is ready for the hardware to take action; also sets the acknowledge bit, start bit and the TWI enable bit.
	
	while(!(TWCR & 0b10000000));//wait until the hardware is done with it's actions. Bit 7 will remain low until it is done.
	statusCode = (TWSR & 0xF8);//mask the status register,
	if(statusCode != 0x08 && statusCode != 0x10)//check that it is the appropriate value for the transmission; here it checks that a START or REPEATED START was sent
		return statusCode;//if it isn't return the bad status code
	TWDR = (address << 1) & 0xFE;//If everything is okay, load the address and write bit. (masking 0xFE is the same as setting bit 0 to 0);
	TWCR = 0b11000100;//Start the hardware again (no start bit this time. otherwise it will send a repeated start when control goes to the hardware
	
	while(!(TWCR & 0b10000000));//wait for the hardware to finish
	statusCode = (TWSR & 0xF8);//mask the status
	if(statusCode != 0x18)//if address was sent and an ACK was received continue otherwise return the statusCode
		return statusCode;
		
	for(int i = 0; i < length; i++)//loop though the number of values specified by the parameter "length"
	{
		TWDR = buffer[i];//load the data into the Data Register
		TWCR = 0b11000100;//give control to the hardware
		while(!(TWCR & 0b10000000));//and wait until it is done
		statusCode = (TWSR & 0xF8);
		if(statusCode != 0x28)//if data was sent and acknowledge received, continue to the next iteration or end
			return statusCode;
	}
	
	TWCR = 0b11010100;//send a stop signal
	return 1;//if everything happened as expected, return 1
}

int readI2C(unsigned char address, unsigned char buffer[], unsigned int length)
{
	unsigned char statusCode;//contains the masked status code after each step
	TWCR = 0b11100100;//a 1 needs to be written to the interrupt flag (bit 7) in order to clear it (counter-intuitive); this is the software telling the hardware it is ready for the hardware to take action; also sets the acknowledge bit, start bit and the TWI enable bit.
	
	while(!(TWCR & 0b10000000));//wait until the hardware is done with it's actions. Bit 7 will remain low until it is done.
	statusCode = (TWSR & 0xF8);//mask the status register,
	if(statusCode != 0x08 && statusCode != 0x10)//check that it is the appropriate value for the transmission; here it checks that a START or REPEATED START was sent
		return statusCode;//if it isn't return the bad status code
	TWDR = (address << 1) | 0x01;//If everything is okay, load the address and read bit (bit 0 is 1).
	TWCR = 0b11000100;//Start the hardware again (no start bit this time. otherwise it will send a repeated start when control goes to the hardware
	
	while(!(TWCR & 0b10000000));//wait for the hardware to finish
	statusCode = (TWSR & 0xF8);//mask the status
	if(statusCode != 0x40)//if address was sent and an ACK was received continue otherwise return the statusCode
		return statusCode;
		
	for(int i = 0; i < length - 1; i++)//loop though the number of values specified by the parameter "length"
	{
		TWCR = 0b11000100;//give control to the hardware
		while(!(TWCR & 0b10000000));//and wait until it is done
		statusCode = (TWSR & 0xF8);
		if(statusCode != 0x50)//if data was received and acknowledge sent, continue to the next iteration or end
			return statusCode;
		buffer[i] = TWDR;//load the data from the Data Register into the buffer
	}
	TWCR = 0b10000100;//give control to the hardware, but this time there'll be a NACK sent instead of the ACK
	
	while(!(TWCR & 0b10000000));//and wait until it is done
	statusCode = (TWSR & 0xF8);
	if(statusCode != 0x58)//if data was received and no acknowledge sent, continue
		return statusCode;
	buffer[length - 1] = TWDR;//read the last byte
	
	TWCR = 0b11010100;//send a stop signal
	return 1;//if everything happened as expected, return 1
}

int readMagneticCompass(int axes[])
{
	unsigned char writeBuffer[1] = {magXoutH};
	unsigned char readBuffer[6];
	int statusCode = writeI2C(magAddress, writeBuffer, 1);//Write so that the following read begins from register magXoutH (the magXoutH comes from what writeBuffer is initialized to);
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	statusCode = readI2C(magAddress, readBuffer, 6);//read the 6 registers starting at magXoutH
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	axes[0] = twoCharToInt(readBuffer[0], readBuffer[1]);//Store x data
	axes[1] = twoCharToInt(readBuffer[4], readBuffer[5]);//Store y data (note that it is coming from 4 and 5)
	axes[2] = twoCharToInt(readBuffer[2], readBuffer[3]);//Store z data (note that it is coming from 2 and 3)
	return 1;
}

int twoCharToInt(unsigned char high, unsigned char low)
{
	int16_t val = high;//val is the 16 bit value used (so that the sign bit comes out right); put high in
	val <<= 8;//shift high over to the upper 8 bits
	val |= low;//or in low
	int result = val;//move the result to an int (preserving the sign bit)
	return result;//return the int
}

#endif
