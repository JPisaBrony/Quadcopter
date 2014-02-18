#include "C:\Users\Owner\Quadcopter\QuadLibrary\quadcopter.h"

#define PROPORTIONAL_GAIN 1.0
#define INTEGRAL_GAIN 1.0
#define DERIVATIVE_GAIN 1.0
#define ERROR_SAMPLES 5000

double accel[3];
double error[ERROR_SAMPLES];
int count;

ISR(TIMER0_COMPA_vect)
{
  count = (count + 1) % ERROR_SAMPLES;
}

void setup()
{
  Serial.begin(9600);
  initializeQuadcopter();
  
  TCCR0A = 0b00000010;
  TCCR0B = 0b00000011;
  OCR0A = 249;
  TIMSK0 = 0b00000010;
  count = 0;
  
  int prevCount = 0;
  
  while(1)
  {
    prevCount++;
  }
}

void loop(){}
