#include <avr/delay.h>
#include <avr/interupt.h>

void setup()
{
  Serial.begin(9600);
  _delay_ms(2000);
  TWBR = 100;
  TWCR = 0b11100101;
  while(1);
}

void loop(){}

ISR(TWI_vect)
{
  unsigned char statusCode = TWSR & 0xF8;
  switch(statusCode)
  {
    case 0x08:
    case 0x10:
      TWDR = 0x3Du;
      TWCR &= ~0b00100000;
    case 0x40:
      break;
    case 0x50:
      Serial.print("Data: ");
      Serial.println(TWDR);
      _delay_ms(50);
    default:
      Serial.print("Status: ");
      Serial.println(statusCode);
      _delay_ms(50);
  }
  TWCR |= 0b10000000;
}

