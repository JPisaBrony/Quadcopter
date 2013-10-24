#include <avr/delay.h>
#include <avr/interrupt.h>

void setup()
{
  Serial.begin(9600);
  _delay_ms(2000);
  TWBR = 100; //determines bit rate; random number > 10
  TWCR = 0b11100101; //enables TWI hardware, Sets the interrupt flag, enables interrupts, the hardware will send ACKs (as opposed to NACKs), and the start signal will be sent.
  while(1); //loop forever, doing nothing. probably not necessary since there is the loop method.
}

void loop(){}

ISR(TWI_vect)//interrupt service routine that is called when the hardware finishes an action with the TWI.
{
  unsigned char statusCode = TWSR & 0xF8;//masks the last three bits of the status code
  switch(statusCode)
  {
    case 0x08://If a START was sent
    case 0x10://If a REPEATED START was sent
      TWDR = 0x3Du;//read address of the compass
      TWCR &= ~0b00100000;//reset the start bit (so that it stops sending starts; very important)
    case 0x40://address sent, ACK received
      break;
    case 0x50://data received, ACK sent
      Serial.print("Data: ");
      Serial.println(TWDR);//print the data that was received (matches what was received with the buttons)
      _delay_ms(50);
      break;
    default://If any other status code is given
      Serial.print("Status: ");
      Serial.println(statusCode);//print it
      _delay_ms(50);
  }
  TWCR |= 0b10000000;//always set the interrupt flag to signal the code is ready for the next action.
}