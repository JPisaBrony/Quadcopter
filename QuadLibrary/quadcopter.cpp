#ifndef QUADCOPTER_CPP_INCLUDED
#define QUADCOPTER_CPP_INCLUDED

//I'm only going to comment on the first function of each group. The only difference is changing the affected port.

void setDirection(BPin p, Direction d)
{
	//checks if the direction is input or output
	if(d == _INPUT)
		//if input, makes the bit corresponding to the desired pin 0 while leaving the rest unaffected.
		DDRB &= ~p; //if affecting PB5 (_PIN9), p will have a value of 0b00100000 making it convenient for this operation.
	else
		//if output, makes the bit corresponding to the desired pin 1 while leaving the rest unaffected.
		DDRB |= p;
}

void setDirection(CPin p, Direction d)
{
	if(d == _INPUT)
		DDRC &= ~p;
	else
		DDRC |= p;
}

void setDirection(DPin p, Direction d)
{
	if(d == _INPUT)
		DDRD &= ~p;
	else
		DDRD |= p;
}

void setDirection(EPin p, Direction d)
{
	if(d == _INPUT)
		DDRE &= ~p;
	else
		DDRE |= p;
}

void setDirection(FPin p, Direction d)
{
	if(d == _INPUT)
		DDRF &= ~(1 << p);//different because of analog to digital conversion
	else
		DDRF |= (1 << p);//different because of analog to digital conversion
}

void digitalOutput(BPin p, Voltage v)
{
	//checks if the voltage should be high or low.
	if(v == _LOW)
		//if low, makes the bit corresponding to the desired pin 0 while leaving the rest unaffected.
		PORTB &= ~p;//if affecting PB5 (_PIN9), p will have a value of 0b00100000 making it convenient for this operation.
	else
		//if high, makes the bit corresponding to the desired pin 1 while leaving the rest unaffected.
		PORTB |= p;
}

void digitalOutput(CPin p, Voltage v)
{
	if(v == _LOW)
		PORTC &= ~p;
	else
		PORTC |= p;
}

void digitalOutput(DPin p, Voltage v)
{
	if(v == _LOW)
		PORTD &= ~p;
	else
		PORTD |= p;
}

void digitalOutput(EPin p, Voltage v)
{
	if(v == _LOW)
		PORTE &= ~p;
	else
		PORTE |= p;
}

void digitalOutput(FPin p, Voltage v)
{
	if(v == _LOW)
		PORTF &= ~(1 << p);//different because of analog to digital conversion
	else
		PORTF |= (1 << p);//different because of analog to digital conversion
}

unsigned int digitalInput(BPin p)
{
	//anding PINB and p gives 0 if the the corresponding pin is low and the value of p if it is high. not-equalling it with zero makes it always 1 or 0. 
	return (PINB & p) != 0;
}

unsigned int digitalInput(CPin p)
{
	return (PINC & p) != 0;
}

unsigned int digitalInput(DPin p)
{
	return (PIND & p) != 0;
}

unsigned int digitalInput(EPin p)
{
	return (PINE & p) != 0;
}

unsigned int digitalInput(FPin p)
{
	return (PINF & (1 << p)) != 0;//different because of analog to digital conversion
}

int initializePWM(double frequency)
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
	maxDuty1 = maxDuty3 = (unsigned int)(F_CPU / (frequency * prescaler) + 0.5);
	
	
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
	ICR1 = maxDuty1;//sets the top counter value for Timer/Counter1
	ICR3 = maxDuty3;//and Timer/Counter3 
	SREG = sreg;//sets the original interrupt state
	
	//success
	return 0;
}

int initializePWM1(double frequency)
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
	maxDuty1 = (unsigned int)(F_CPU / (frequency * prescaler) + 0.5);
	
	//clears any interrupts from the timers
	TIMSK1 = 0b00000000;
	
	TCCR1A = 0b00000010;//disconnects output to all Timer/Counter1 pins (Bit 7:2), sets the waveform generation mode to mode 14 (FastPWM with ICR1 as the top value) (Bit 1:0)
	
	if(prescaler == 8)
	{
		TCCR1B = 0b00011010;//Turns off noise cancellation and input capture edge select (only applicable to input) (Bit 7:6), Bit 5 is reserved, sets the waveform generation mode to mode 14 (FastPWM with ICR1 as the top value) (Bit 4:3), and sets the appropriate prescaler.
	}
	else if(prescaler == 64)//the following conditions are the same as the previous condition but with a different prescalers;
	{
		TCCR1B = 0b00011011;
	}
	else if(prescaler == 256)
	{
		TCCR1B = 0b00011100;
	}
	else
	{
		TCCR1B = 0b00011101;
	}
	
	
	//The following chunk sets 16-bit registers
	unsigned char sreg;
	sreg = SREG;//saves the interrupt state
	cli();//avr function that clears all the interrupts
	ICR1 = maxDuty1;//sets the top counter value for Timer/Counter1
	SREG = sreg;//sets the original interrupt state
	
	//success
	return 0;
}

int initializePWM3(double frequency)
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
	maxDuty3 = (unsigned int)(F_CPU / (frequency * prescaler) + 0.5);
	
	//clears any interrupts from the timers
	TIMSK3 = 0b00000000;
	
	TCCR3A = 0b00000010;//same as above but for Timer/Counter3
	
	if(prescaler == 8)
	{
		TCCR3B = 0b00011010;//same as above but for Timer/Counter3
	}
	else if(prescaler == 64)//the following conditions are the same as the previous condition but with a different prescalers;
	{
		TCCR3B = 0b00011011;
	}
	else if(prescaler == 256)
	{
		TCCR3B = 0b00011100;
	}
	else
	{
		TCCR3B = 0b00011101;
	}
	
	
	//The following chunk sets 16-bit registers
	unsigned char sreg;
	sreg = SREG;//saves the interrupt state
	cli();//avr function that clears all the interrupts
	ICR3 = maxDuty3;//and Timer/Counter3 
	SREG = sreg;//sets the original interrupt state
	
	//success
	return 0;
}

void PWMOutput(PWM_TC1 p, double duty)
{
	//if duty is between 1 and 0 it is good, otherwise set it to the closest edge
	if(duty > 1.0)
		duty = 1.0;
	else if(duty < 0.0)
		duty = 0.0;
	
	//find the value that represents the closest value to the desired duty cycle (rounded to the nearest integer)
	unsigned int val = (unsigned int)(duty * maxDuty1 + 0.5);
	
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

void PWMOutput(PWM_TC3 p, double duty)
{
	if(duty > 1.0)
		duty = 1.0;
	else if(duty < 0.0)
		duty = 0.0;
	
	unsigned int val = (unsigned int)(duty * maxDuty3 + 0.5);
	
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

void stopPWM(PWM_TC1 p)
{
	TCCR1A &= ~p;//disconnects the wave generator
}

void stopPWM(PWM_TC3 p)
{
	TCCR3A &= ~p;
}

double analogInput(FPin p)
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
	return (result * 5) / 1023.0;
}

double analogInput(FPin p, double AREFVoltage)
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
	return (result * AREFVoltage) / 1023.0;
}

int twoCharToInt(unsigned char high, unsigned char low)
{
	int16_t val = high;//val is the 16 bit value used (so that the sign bit comes out right); put high in
	val <<= 8;//shift high over to the upper 8 bits
	val |= low;//or in low
	//int result = val;//move the result to an int (preserving the sign bit)
	//return result;//return the int
	return val;
}

int initializeTWI(unsigned char TWIBitRate, unsigned char TWIBitRatePrescaler)
{
	setDirection(_PIN6, _INPUT);
	setDirection(_PIN2, _INPUT);
	setDirection(_PIN3, _INPUT);

	//set the bit rate for the processor
	TWBR = TWIBitRate;
	TWSR = TWIBitRatePrescaler & 0x03;
	
	unsigned char statusCode;//contains the masked status code after each step
	unsigned char writeBuffer[1] = {0};
		
	//Compass
	
	unsigned char index = 0;
	
	while(index < NUM_COMPASS_REGISTERS)//loop until all the registers have been checked
	{
		if(compassRegister[index] < 256)//if the value is too high, skip it
		{
			TWCR = 0b11100100;//a 1 needs to be written to the interrupt flag (bit 7) in order to clear it (counter-intuitive); this is the software telling the hardware it is ready for the hardware to take action; also sets the acknowledge bit, start bit and the TWI enable bit.
	
			while(!(TWCR & 0b10000000));//wait until the hardware is done with it's actions. Bit 7 will remain low until it is done.
			statusCode = (TWSR & 0xF8);//mask the status register,
			if(statusCode != 0x08 && statusCode != 0x10)//check that it is the appropriate value for the transmission; here it checks that a START or REPEATED START was sent
				return statusCode;//if it isn't return the bad status code
			TWDR = (magAddress << 1) & 0xFE;//If everything is okay, load the address and write bit. (masking 0xFE is the same as setting bit 0 to 0);
			TWCR = 0b11000100;//Start the hardware again (no start bit this time. otherwise it will send a repeated start when control goes to the hardware

			while(!(TWCR & 0b10000000));//wait for the hardware to finish
			statusCode = (TWSR & 0xF8);//mask the status
			if(statusCode != 0x18)//if address was sent and an ACK was received continue otherwise return the statusCode
				return statusCode;
		
			TWDR = index;//load the register address into the Data Register
			TWCR = 0b11000100;//give control to the hardware
			while(!(TWCR & 0b10000000));//and wait until it is done
			statusCode = (TWSR & 0xF8);
			if(statusCode != 0x28)//if data was sent and acknowledge received, continue to the next iteration or end
				return statusCode;
		
			for(; index < NUM_COMPASS_REGISTERS; index++)//loop through consecutive bytes that
			{
				if(compassRegister[index] < 256)//are less than 256
				{
					TWDR = compassRegister[index];//load the data into the Data Register
					TWCR = 0b11000100;//give control to the hardware
					while(!(TWCR & 0b10000000));//and wait until it is done
					statusCode = (TWSR & 0xF8);
					if(statusCode != 0x28)//if data was sent and acknowledge received, continue to the next iteration or end
						return statusCode;
				}
				else//too big start a new transmission from the correct register
				{
					index++;
					break;
				}
			}
			TWCR = 0b11010100;//send a stop signal
		}
		else
		{
			index++;
		}
	}
	
	writeI2C(magAddress, writeBuffer, 1);//write a zero to update the registers
	
	//Accelerometer
	
	index = 0;
	
	while(index < NUM_ACCELEROMETER_REGISTERS)
	{
		if(accelerometerRegister[index] < 256)
		{
			TWCR = 0b11100100;
	
			while(!(TWCR & 0b10000000));
			statusCode = (TWSR & 0xF8);
			if(statusCode != 0x08 && statusCode != 0x10)
				return statusCode;
			TWDR = (accelAddress << 1) & 0xFE;
			TWCR = 0b11000100;

			while(!(TWCR & 0b10000000));
			statusCode = (TWSR & 0xF8);
			if(statusCode != 0x18)
				return statusCode;
		
			TWDR = index;
			TWCR = 0b11000100;
			while(!(TWCR & 0b10000000));
			statusCode = (TWSR & 0xF8);
			if(statusCode != 0x28)
				return statusCode;
		
			for(; index < NUM_ACCELEROMETER_REGISTERS; index++)
			{
				if(accelerometerRegister[index] < 256)
				{
					TWDR = accelerometerRegister[index];
					TWCR = 0b11000100;
					while(!(TWCR & 0b10000000));
					statusCode = (TWSR & 0xF8);
					if(statusCode != 0x28)
						return statusCode;
				}
				else
				{
					index++;
					break;
				}
			}
			TWCR = 0b11010100;
		}
		else
		{
			index++;
		}
	}
	
	writeI2C(accelAddress, writeBuffer, 1);//not sure if necessary
	
	//Gyroscope
	
	index = 0;
	
	while(index < NUM_GYROSCOPE_REGISTERS)
	{
		if(gyroscopeRegister[index] < 256)
		{
			TWCR = 0b11100100;
	
			while(!(TWCR & 0b10000000));
			statusCode = (TWSR & 0xF8);
			if(statusCode != 0x08 && statusCode != 0x10)
				return statusCode;
			TWDR = (gyroAddress << 1) & 0xFE;
			TWCR = 0b11000100;

			while(!(TWCR & 0b10000000));
			statusCode = (TWSR & 0xF8);
			if(statusCode != 0x18)
				return statusCode;
		
			TWDR = index;
			TWCR = 0b11000100;
			while(!(TWCR & 0b10000000));
			statusCode = (TWSR & 0xF8);
			if(statusCode != 0x28)
				return statusCode;
		
			for(; index < NUM_GYROSCOPE_REGISTERS; index++)
			{
				if(gyroscopeRegister[index] < 256)
				{
					TWDR = gyroscopeRegister[index];
					TWCR = 0b11000100;
					while(!(TWCR & 0b10000000));
					statusCode = (TWSR & 0xF8);
					if(statusCode != 0x28)
						return statusCode;
				}
				else
				{
					index++;
					break;
				}
			}
			TWCR = 0b11010100;
		}
		else
		{
			index++;
		}
	}
	
	writeI2C(gyroAddress, writeBuffer, 1);//not sure if necessary
	
	unsigned char readBuffer[22];
	writeBuffer[0] = bmpCalibrationCoefficients;
	
	statusCode = writeI2C(bmpAddress, writeBuffer, 1);
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	
	statusCode = readI2C(bmpAddress, readBuffer, 22);
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	
	for(int i = 0; i < 11; i++)
	{
		if((readBuffer[2 * i] == 0x00 && readBuffer[2 * i + 1] == 0x00) ||(readBuffer[2 * i] == 0xFF && readBuffer[2 * i + 1] == 0xFF))//if the 16 bit value == 0x0000 or 0xFFFF, there was an error
			return 2;
			
		if(3 <= i && i <= 5)
			barometerCoefficient[i] = (unsigned int)twoCharToInt(readBuffer[2 * i], readBuffer[2 * i + 1]);
		else
			barometerCoefficient[i] = twoCharToInt(readBuffer[2 * i], readBuffer[2 * i + 1]);
	}
	
	return 1;//if everything happened as expected, return 1
}

void resetSetupRegisters()
{
	//set all the values to 256
	for(int i = 0; i < NUM_COMPASS_REGISTERS; i++)
		compassRegister[i] = 256;
	for(int i = 0; i < NUM_ACCELEROMETER_REGISTERS; i++)
		accelerometerRegister[i] = 256;
	for(int i = 0; i < NUM_GYROSCOPE_REGISTERS; i++)
		gyroscopeRegister[i] = 256;
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
		
	for(unsigned int i = 0; i < length; i++)//loop though the number of values specified by the parameter "length"
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
		
	for(unsigned int i = 0; i < length - 1; i++)//loop though the number of values specified by the parameter "length"
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
	if(length == 0)
		return -1;
	buffer[length - 1] = TWDR;//read the last byte
	
	TWCR = 0b11010100;//send a stop signal
	return 1;//if everything happened as expected, return 1
}

int readI2CCompass(double axes[])
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

int readI2CAccelerometer(double axes[])
{
	unsigned char writeBuffer[1] = {accelXoutL};
	unsigned char readBuffer[6];
	int statusCode = writeI2C(accelAddress, writeBuffer, 1);//Write so that the following read begins from register accelXoutL (the accelXoutL comes from what writeBuffer is initialized to);
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	statusCode = readI2C(accelAddress, readBuffer, 6);//read the 6 registers starting at accelXoutL
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	axes[0] = twoCharToInt(readBuffer[1], readBuffer[0]);//Store x data (note that the low bit is read first)
	axes[1] = twoCharToInt(readBuffer[3], readBuffer[2]);//Store y data (note that the low bit is read first)
	axes[2] = twoCharToInt(readBuffer[5], readBuffer[4]);//Store z data (note that the low bit is read first)
	return 1;
}

int readI2CGyroscope(double axes[])
{
	unsigned char writeBuffer[1] = {gyroTEMP_OUT_H};
	unsigned char readBuffer[8];
	int statusCode = writeI2C(gyroAddress, writeBuffer, 1);//Write so that the following read begins from register gyroTEMP_OUT_H (the gyroTEMP_OUT_H comes from what writeBuffer is initialized to);
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	statusCode = readI2C(gyroAddress, readBuffer, 8);//read the 8 registers starting at gyroTEMP_OUT_H
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	
	int temperature = twoCharToInt(readBuffer[0], readBuffer[1]);
	
	//TODO Recalibrate this. Values not centered at zero.
	axes[0] = twoCharToInt(readBuffer[2], readBuffer[3]) + 154.9817501895265  +  0.004000508727186314 * temperature  -  8.23851E-8 * temperature * temperature  -  9.005870015903617E-12 * temperature * temperature * temperature  -  1.656470138597602E-16 * temperature * temperature * temperature * temperature;
	axes[1] = twoCharToInt(readBuffer[4], readBuffer[5]) - 90.31205526954072  -  0.010700316729624036 * temperature  -  4.5907900000000004E-8 * temperature * temperature  +  2.1021177699990898E-11 * temperature * temperature * temperature  +  5.094010266748168E-16 * temperature * temperature * temperature * temperature;
	axes[2] = twoCharToInt(readBuffer[6], readBuffer[7]) - 35.15564001714997  +  0.00041568193715910877 * temperature  +  9.29453E-8 * temperature * temperature  +  1.8663020031915918E-12 * temperature * temperature * temperature  +  1.1493123515668173E-17 * temperature * temperature * temperature * temperature;
	
	return 1;
}

int readI2CBarometer(double* altitude)
{
	unsigned char writeBuffer[2] = {bmpControlRegister, bmpTemperature};
	unsigned char readBuffer[2];
	int statusCode = writeI2C(bmpAddress, writeBuffer, 2);//Write so that the sensor prepares the temperature data
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	writeBuffer[0] = bmpReadRegisters;
	while(!digitalInput(_PIN6));
	statusCode = writeI2C(bmpAddress, writeBuffer, 1);//Write so that the sensor will read out the temperature data
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	statusCode = readI2C(bmpAddress, readBuffer, 2);//read the 2-byte value for temperature
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	long ut = (unsigned int)twoCharToInt(readBuffer[0], readBuffer[1]);//merge the bytes into temperature
	
	writeBuffer[0] = bmpControlRegister;
	writeBuffer[1] = bmpStandardPressure;//if you change the the oversampling on the pressure, make sure you edit the pressure and altitude calculation formulas
	statusCode = writeI2C(bmpAddress, writeBuffer, 2);//Write so that the sensor prepares the pressure data
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	writeBuffer[0] = bmpReadRegisters;
	while(!digitalInput(_PIN6));
	statusCode = writeI2C(bmpAddress, writeBuffer, 1);//Write so that the sensor will read out the temperature
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	statusCode = readI2C(bmpAddress, readBuffer, 2);//read the 2-byte value for pressure
	if(statusCode != 1)//check the resulting status code
		return statusCode;
	
	long up = (unsigned int)twoCharToInt(readBuffer[0], readBuffer[1]);//merge the bytes into the pressure
	
	long x1 = ((ut - barometerCoefficient[bcAC6]) * barometerCoefficient[bcAC5])/0x8000;
	long x2 = (barometerCoefficient[bcMC] * 0x800) / (x1 + barometerCoefficient[bcMD]);
	long b5 = x1 + x2;
	long t = (b5 + 8) / 0x10;//temperature in 0.1 C (divide by 10 to get temperature in C)
	
	long b6 = b5 - 4000;
	x1 = (barometerCoefficient[bcB2] * (b6 * b6 / 0x1000)) / 0x800;
	x2 = barometerCoefficient[bcAC2] * b6 / 0x800;
	long x3 = x1 + x2;
	long b3 = ((barometerCoefficient[bcAC1] * 4 + x3) + 2)/4;
	x1 = barometerCoefficient[bcAC3] * b6 / 0x2000;
	x2 = (barometerCoefficient[bcB1] * ((b6 * b6)/0x1000))/0x10000;
	x3 = (x1 + x2 + 2) / 0x4;
	unsigned long b4 = barometerCoefficient[bcAC4] * (unsigned long)(x3 + 32768) / 0x8000;
	unsigned long b7 = ((unsigned long)up - b3) * (50000);
	long p;
	if(b7 < 0x80000000)
		p = (b7 * 2) / b4;
	else
		p = (b7 / b4) * 2;
	x1 = (p / 0x100) * (p / 0x100);
	x1 = (x1 * 3038) / 0x10000;
	x2 = -7357 * p / 0x10000;
	p += (x1 + x2 + 3791) / 0x10; //pressure in pascals
	
	*altitude = 44330 * (1 - pow(p / STANDARD_SEA_LEVEL_PRESSURE, 1.0 / 5.255)) - initialAltitude;
	
	return 1;
}

void initializeGPS()
{
	while(!(UCSR1A & 0b00100000));//verify the transfer buffer is empty before starting

	UCSR1A = 0b00000000;
	UCSR1B = 0b00010000;//enable the receiver and disable the transmitter
	UCSR1C = 0b00000110;//asynchronous USART, no parity, 1-bit stop, 8-bit character size
	UBRR1H = BAUD_RATE_4800_VALUE_H;
	UBRR1L = BAUD_RATE_4800_VALUE_L;//set to 4800 bps
}

void updateGPSData()
{
	while(UCSR1A & 0b10000000)//While there is data in the buffer
	{
		char receivedData = UDR1;//read the new data
		if(receivedData == '$')//if it is the start of a new packet...
			currentGPSPacketPosition = 0;//reset the position counter
		currentGPSPacket[currentGPSPacketPosition] = receivedData;//place the new data in a char array to keep track of it
		if(receivedData == '\n')//if it is the last char in the packet...
			parseGPSPacket();//parse it
		currentGPSPacketPosition++;//increment the position counter for the next call
	}
}

void parseGPSPacket()
{
	unsigned char checksum = 0;
	char *stringStart;
	int i;
	for(i = 1; currentGPSPacket[i] != '*'; i++)
	{
		checksum ^= currentGPSPacket[i];//the check sum xors each byte together
	}
	if(checksum == strtol(&(currentGPSPacket[i+1]), NULL, 16))//if the checksum matches
	{
		if(strncmp(currentGPSPacket, "$GPRMC", 6) == 0)//if the packet is a RMC packet
		{
			stringStart = strchr(currentGPSPacket, ',');//points to the first comma right after the message ID
			stringStart++;//points to the first character of the UTC time
			rawUTCTime = strtod(stringStart, &stringStart);//saves the UTC Time data; stringStart now points to the following comma; if no data available the result is 0f
			stringStart++;//points to the status
			if(*stringStart != ',')
			{
				rawStatus = *stringStart;//saves the status
				stringStart++;//regardless of whether there was a status or not, after the if statement it will be pointing at the following comma
			}
			else
				rawStatus = 'V';//invalid data
			stringStart++;//points to the Latitude
			rawLatitude = strtod(stringStart, &stringStart);//saves the Latitude data; stringStart now points to the following comma; if no data available the result is 0f;
			stringStart++;//points to the North/South indicator
			if(*stringStart != ',')
			{
				rawNSIndicator = *stringStart;//saves the N/S Indicator
				stringStart++;//regardless of whether there was a N/S Indicator or not, after the if statement it will be pointing at the following comma
			}
			stringStart++;//points to the longitude
			rawLongitude = strtod(stringStart, &stringStart);//saves the Longitude data; stringStart now points to the following comma; if no data available the result is 0f;
			stringStart++;//points to the East/West indicator
			if(*stringStart != ',')
			{
				rawEWIndicator = *stringStart;//saves the E/W Indicator
				stringStart++;//regardless of whether there was a E/W Indicator or not, after the if statement it will be pointing at the following comma
			}
			stringStart++;//points to the speed over ground
			rawSpeedOverGround = strtod(stringStart, &stringStart);//saves the Speed Over Ground data; stringStart now points to the following comma; if no data available the result is 0f;
			stringStart++;//points to the course over ground
			rawCourseOverGround = strtod(stringStart, &stringStart);//saves the Course Over Ground data; stringStart now points to the following comma; if no data available the result is 0f;
			stringStart++;//points to the date
			rawDate = strtol(stringStart, &stringStart, 10);//saves the date data; stringStart now points to the following comma; if no data available the result is 0f;
		}
		
		//if other messages will be received, add them here in an else if statement and follow a similar procedure.
		
	}
	if(rawStatus == 'A')
	{
		utcTime = rawUTCTime;
		latitude = (2 * (rawNSIndicator == 'N') - 1) * rawLatitude / 100.0;
		longitude = (2 * (rawEWIndicator == 'E') - 1) * rawLongitude / 100.0;
		speedOverGround = rawSpeedOverGround;
		courseOverGround = rawCourseOverGround;
		northSpeedOverGround = rawSpeedOverGround * sin(rawCourseOverGround * acos(-1.0L) / 180.0L);
		eastSpeedOverGround = rawSpeedOverGround * cos(rawCourseOverGround * acos(-1.0L) / 180.0L);
		date = rawDate;
		rawStatus = 'D';
	}
}

int initializeQuadcopter()
{
	resetSetupRegisters();//reset all the register values to 256
	
	//bits 6 and 5 represent 8 measurements per output
	//bits 4, 3, and 2 represent an output rate of 75 Hz
	//bits 1 and 0 represent no bias in the readings (default)
	compassRegister[magCRA] = 0b01111000;
	
	//bits 7, 6, and 5 represent the gain. Set to the default of 1090 LSb/Gauss
	compassRegister[magCRB] = 0b00100000;
	
	//bits 1 and 0 represent the measurement mode. Set to continuous measurement mode
	compassRegister[magMode] = 0b00000000;
	
	
	
	//bits 3 through 0 set output rate at 100 Hz
	accelerometerRegister[accelBW_RATE] = 0b00001010;
	
	//bit three sets the accelerometer into measurement mode
	accelerometerRegister[accelPOWER_CTL] = 0b00001000;
	
	
	
	//sets the output rate to match the internal sample rate (1kHz or 8kHz)
	gyroscopeRegister[gyroSMPLRT_DIV] = 0b00000000;
	
	//bits 4 and 3 allow the gyroscope to begin collecting data
	gyroscopeRegister[gyroDLPF_FS] = 0b00011000;
	
	
	int statusCode = initializeTWI(100, 0);//initialize two wire interface with a bit rate of 100 and bit rate prescaler of 0
	if(statusCode != 1)
		return statusCode;
	
	//these pins are for PWM output to the motor controllers
	setDirection(_PIN5, _OUTPUT);
	setDirection(_PIN9, _OUTPUT);
	setDirection(_PIN10, _OUTPUT);
	setDirection(_PIN11, _OUTPUT);
	
	initializePWM(50);//start the timers for PWM at 50 Hz
	
	stopMotors();//start sending output to the motors (but below the threshold of movement)
	
	//initialize the timer interrupt
	TCCR0A = 0b00000010;//No output, CTC
	TCCR0B = 0b00000011;//CTC, Prescaler of 64
	OCR0A = 124;//reset every 2000th of a second
	TIMSK0 = 0b00000010;//enable interrupts every 1000th of a second
	
	double altitudeI = 0;
	double accelerationI[3];
	
	initialAltitude = 0;
	double altitudeAverage = 0;
	accelerationDueToGravity = 0;
	for(int i = 0; i < 100; i++)
	{
		statusCode = readI2CBarometer(&altitudeI);
		if(statusCode != 1)
			return statusCode;
		statusCode = readI2CAccelerometer(accelerationI);
		if(statusCode != 1)
			return statusCode;
		altitudeAverage += altitudeI;
		accelerationDueToGravity += sqrt(accelerationI[0] * accelerationI[0] + accelerationI[1] * accelerationI[1] + accelerationI[2] * accelerationI[2]);
	}
	
	initialAltitude = altitudeAverage / 100;
	accelerationDueToGravity /= 100;
	
	//Moar Initialization!
	
	return 1;
}

void setMotors(double xp, double xn, double yp, double yn)
{
	//scale the values to the correct range
	xp *= UPPER_LIMIT - LOWER_LIMIT;
	xn *= UPPER_LIMIT - LOWER_LIMIT;
	yp *= UPPER_LIMIT - LOWER_LIMIT;
	yn *= UPPER_LIMIT - LOWER_LIMIT;
	
	//shift it up so that 0.0 corresponds to the lower threshold of the motor
	xp += LOWER_LIMIT;
	xn += LOWER_LIMIT;
	yp += LOWER_LIMIT;
	yn += LOWER_LIMIT;

	//ensure that the values are within the range
	
	if(xp < LOWER_LIMIT)
		xp = LOWER_LIMIT;
	else if(xp > UPPER_LIMIT)
		xp = UPPER_LIMIT;
	
	if(xn < LOWER_LIMIT)
		xn = LOWER_LIMIT;
	else if(xn > UPPER_LIMIT)
		xn = UPPER_LIMIT;
	
	if(yp < LOWER_LIMIT)
		yp = LOWER_LIMIT;
	else if(yp > UPPER_LIMIT)
		yp = UPPER_LIMIT;

	if(yn < LOWER_LIMIT)
		yn = LOWER_LIMIT;
	else if(yn > UPPER_LIMIT)
		yn = UPPER_LIMIT;
		
	//output the signal
		
	PWMOutput(_PIN5PWM, xp);
	PWMOutput(_PIN9PWM, xn);
	PWMOutput(_PIN10PWM, yp);
	PWMOutput(_PIN11PWM, yn);
}

void stopMotors()
{
	setMotors(0, 0, 0, 0);
}

ISR(TIMER0_COMPA_vect)
{
	timeDifferential++;
}

#endif
