// realtimeclock1.c
// for NerdKits with ATmega168
// mrobbins@mit.edu

#define F_CPU 14745600

#include <stdio.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <inttypes.h>

#include "../libnerdkits/delay.h"
#include "../libnerdkits/lcd.h"
#include "../libnerdkits/uart.h"

// PIN DEFINITIONS:

void realtimeclock_setup() {
  // setup Timer0:
  // CTC (Clear Timer on Compare Match mode)
  // TOP set by OCR0A register
  TCCR0A |= (1<<WGM01);
  // clocked from CLK/1024
  // which is 14745600/1024, or 14400 increments per second
  TCCR0B |= (1<<CS02) | (1<<CS00);
  // set TOP to 143
  // because it counts 0, 1, 2, ... 142, 143, 0, 1, 2 ...
  // so 0 through 143 equals 144 events
  OCR0A = 143;
  // enable interrupt on compare event
  // (14400 / 144 = 100 per second)
  TIMSK0 |= (1<<OCIE0A);
}

// the_time will store the elapsed time
// in hundredths of a second.
// (100 = 1 second)
// 
// note that this will overflow in approximately 248 days!
//
// This variable is marked "volatile" because it is modified
// by an interrupt handler.  Without the "volatile" marking,
// the compiler might just assume that it doesn't change in 
// the flow of any given function (if the compiler doesn't
// see any code in that function modifying it -- sounds 
// reasonable, normally!).
//
// But with "volatile", it will always read it from memory 
// instead of making that assumption.
volatile int32_t the_time;

SIGNAL(SIG_OUTPUT_COMPARE0A) {
  // when Timer0 gets to its Output Compare value,
  // one one-hundredth of a second has elapsed (0.01 seconds).
  the_time++;
}

int main() {
  realtimeclock_setup();

  // init lcd
  lcd_init();
  FILE lcd_stream = FDEV_SETUP_STREAM(lcd_putchar, 0, _FDEV_SETUP_WRITE);
  lcd_home();
  
  // init serial port
  uart_init();
  FILE uart_stream = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);
  stdin = stdout = &uart_stream;
  

  // turn on interrupt handler
  sei();

  while(1) {
    lcd_home();
    fprintf_P(&lcd_stream, PSTR("%16.2f sec"), (double) the_time / 100.0);
  } 


  return 0;
}
