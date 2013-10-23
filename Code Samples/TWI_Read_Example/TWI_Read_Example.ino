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
      TWDR = 0x3Du;
    default:
      Serial.println(statusCode);
      _delay_ms(50);
  }
}

