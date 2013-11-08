#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
char buffer[32];
int pos = 0;

void updatePrintBuffer(char c);
void printBuffer();

void setup()
{
  lcd.begin(16, 2);
  char c = 0;
  
  UBRR1H = 0;
  UBRR1L = 207;
  UCSR1A = 0b00000000;
  UCSR1B = 0b00010000;
  UCSR1C = 0b00000110;
  
  while(1)
  {
    while(!(UCSR1A & 0b10000000));
    updatePrintBuffer(UDR1);
  }
}

void loop(){}

void updatePrintBuffer(char c)
{
  buffer[pos] = c;
  pos++;
  pos %= 32;
  printBuffer();
}

void printBuffer()
{
  for(int i = 0; i < 32; i++)
  {
    lcd.setCursor(i % 16, i / 16);
    lcd.print(buffer[i]);
  }
}
