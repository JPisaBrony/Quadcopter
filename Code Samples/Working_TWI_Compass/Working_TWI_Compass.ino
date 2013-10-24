#include <avr/delay.h>
#include <avr/interrupt.h>

#define MAX_LENGTH 100

int writeI2C(unsigned char address, unsigned char buffer[], unsigned int length, unsigned char bitRate);
int readI2C(unsigned char address, unsigned char buffer[], unsigned int length, unsigned char bitRate);

void setup()
{
  Serial.begin(9600);
  _delay_ms(5000);
  unsigned char writeBuffer[MAX_LENGTH] = {0, 0b00010000, 0b00100000};
  unsigned char readBuffer[MAX_LENGTH] = {0};
  Serial.print("Write Status: ");
  Serial.println(writeI2C(0x1E, writeBuffer, 3, 100));
  writeBuffer[0] = 0;
  while(1){
  Serial.print("Write Status: ");
  Serial.println(writeI2C(0x1E, writeBuffer, 1, 100));
  Serial.print("Read Status: ");
  Serial.println(readI2C(0x1E, readBuffer, 12, 100));
  Serial.println("Data: ");
  for(int i = 0; i < 12; i++)
    Serial.println(readBuffer[i]);
    _delay_ms(333);
}}
void loop(){}

int writeI2C(unsigned char address, unsigned char buffer[], unsigned int length, unsigned char bitRate)
{
  unsigned char statusCode;
  TWCR = 0b00000000;
  TWBR = bitRate;
  TWCR = 0b11100100;
  while(!(TWCR & 0b10000000));
  statusCode = (TWSR & 0xF8);
  if(statusCode != 0x08 && statusCode != 0x10)
    return statusCode;
  TWDR = (address << 1) & 0xFE;
  TWCR = 0b11000100;
  while(!(TWCR & 0b10000000));
  statusCode = (TWSR & 0xF8);
  if(statusCode != 0x18)
    return statusCode;
  for(int i = 0; i < length; i++)
  {
    TWDR = buffer[i];
    TWCR = 0b11000100;
    while(!(TWCR & 0b10000000));
    statusCode = (TWSR & 0xF8);
    if(statusCode != 0x28)
      return statusCode;
  }
  TWCR = 0b11010100;
  return 1;
}

int readI2C(unsigned char address, unsigned char buffer[], unsigned int length, unsigned char bitRate)
{
  unsigned char statusCode;
  TWCR = 0b00000000;
  TWBR = bitRate;
  TWCR = 0b11100100;
  while(!(TWCR & 0b10000000));
  statusCode = (TWSR & 0xF8);
  if(statusCode != 0x08 && statusCode != 0x10)
    return statusCode;
  TWDR = (address << 1) | 0x01;
  TWCR = 0b11000100;
  while(!(TWCR & 0b10000000));
  statusCode = (TWSR & 0xF8);
  if(statusCode != 0x40)
    return statusCode;
  for(int i = 0; i < length - 1; i++)
  {
    TWCR = 0b11000100;
    while(!(TWCR & 0b10000000));
    statusCode = (TWSR & 0xF8);
    if(statusCode != 0x50)
      return statusCode;
    buffer[i] = TWDR;
  }
  TWCR = 0b10000100;
  while(!(TWCR & 0b10000000));
  statusCode = (TWSR & 0xF8);
  if(statusCode != 0x58)
    return statusCode;
  buffer[length - 1] = TWDR;
  TWCR = 0b11010100;
  return 1;
}
