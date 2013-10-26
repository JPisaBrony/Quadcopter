#include <avr/delay.h>
#include <avr/interrupt.h>

//some definitions
#define MAX_LENGTH 100
#define I2C_BIT_RATE 100;

//some prototypes
int writeI2C(unsigned char address, unsigned char buffer[], unsigned int length);
int readI2C(unsigned char address, unsigned char buffer[], unsigned int length);

void setup()
{
  Serial.begin(9600);
  _delay_ms(5000);
  
  //declare the arrays that will be used in this exampe to hold the data
  unsigned char writeBuffer[MAX_LENGTH] = {0, 0b00010000, 0b00100000, 0b00000000};
  unsigned char readBuffer[MAX_LENGTH] = {0};
  
  //wrrite to the configuration registers and the mode register and output the status (If everything works it should say "Write Status: 1")
  Serial.print("Write Status: ");
  Serial.println(writeI2C(/*Compass Address:*/0x1E, /*The array where the data will come from:*/writeBuffer, /*Number of values to write:*/4));
  

  writeBuffer[0] = 0;
  while(1){
  //start a new transmission and write a zero to update the contol registers; in the while loop to start reading from register 0
  Serial.print("Write Status: ");
  Serial.println(writeI2C(/*Compass Address:*/0x1E, /*The array where the data will come from:*/writeBuffer, /*Number of values to write:*/1));
  
  //read from all twelve registers
  Serial.print("Read Status: ");
  Serial.println(readI2C(/*Compass Address:*/0x1E, /*The array where the data will come from:*/readBuffer, /*Number of values to read:*/12));
  Serial.println("Data: ");
  for(int i = 0; i < 12; i++)
    Serial.println(readBuffer[i]);
    _delay_ms(333);
}}
void loop(){}

int writeI2C(unsigned char address, unsigned char buffer[], unsigned int length)
{
  unsigned char statusCode;//contains the masked status code after each step
  TWCR = 0b00000000;//clears the control register; don't know why exactly. bit 7 should be 0 before messing with the registers, however
  TWBR = I2C_BIT_RATE;//Set the bit rate. not really nessisary, assuming it has been set before
  TWCR = 0b11100100;//a 1 needs to be written to the interrupt flag (bit 7) in order to clear it (counterintuitive); this is the software telling the hardware it is ready for the hardware to take action; also sets the acknowlage bit, start bit and the TWI enable bit.
  while(!(TWCR & 0b10000000));//wait until the hardware is done with it's actions. Bit 7 will remain low until it is done.
  statusCode = (TWSR & 0xF8);//mask the status register,
  if(statusCode != 0x08 && statusCode != 0x10)//check that it is the appropriate value for the transmission; here it checks that a START or REPEATED START was sent
    return statusCode;//if it isn't return the bad status code
  TWDR = (address << 1) & 0xFE;//If everything is okay, load the address and write bit. (maksking 0xFE is the same as setting bit 0 to 0);
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
    if(statusCode != 0x28)//if data was sent and acknowlage received, continue to the next iteration or end
      return statusCode;
  }
  TWCR = 0b11010100;//send a stop signal
  return 1;//if everything happened as expected, return 1
}

int readI2C(unsigned char address, unsigned char buffer[], unsigned int length)
{
  unsigned char statusCode;//contains the masked status code after each step
  TWCR = 0b00000000;//clears the control register; don't know why exactly. bit 7 should be 0 before messing with the registers, however
  TWBR = I2C_BIT_RATE;//Set the bit rate. not really nessisary, assuming it has been set before
  TWCR = 0b11100100;//a 1 needs to be written to the interrupt flag (bit 7) in order to clear it (counterintuitive); this is the software telling the hardware it is ready for the hardware to take action; also sets the acknowlage bit, start bit and the TWI enable bit.
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
    if(statusCode != 0x50)//if data was recived and acknowlage sent, continue to the next iteration or end
      return statusCode;
    buffer[i] = TWDR;//load the data from the Data Register into the buffer
  }
  TWCR = 0b10000100;//give control to the hardware, but this time there'll be a NACK sent instead of the ACK
  while(!(TWCR & 0b10000000));//and wait until it is done
  statusCode = (TWSR & 0xF8);
  if(statusCode != 0x58)//if data was recived and no acknowlage sent, continue
    return statusCode;
  buffer[length - 1] = TWDR;//read the last byte
  TWCR = 0b11010100;//send a stop signal
  return 1;//if everything happened as expected, return 1
}

int readCompass(int axes[])
{
  unsigned char writeData[1] = {3};
  unsigned char readData[6];
  writeI2C(0x1E, writeData, 1);//write a 3 to the compass (registers 3 through 8 have the data about the orinetation of the magnetic field)
  readI2C(0x1E, readData, 6);//read from thos
  
}
