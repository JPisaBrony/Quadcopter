GCCFLAGS=-g -Os -Wall -mmcu=atmega168 
LINKFLAGS=-Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_flt -lm
AVRDUDEFLAGS=-c avr109 -p m168 -b 115200 -P com3 
LINKOBJECTS=../libnerdkits/delay.o ../libnerdkits/lcd.o ../libnerdkits/uart.o

all:	initialload-upload

initialload.hex:	initialload.c
	make -C ../libnerdkits
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o initialload.o initialload.c ${LINKOBJECTS}
	avr-objcopy -j .text -O ihex initialload.o initialload.hex
	
initialload.ass:	initialload.hex
	avr-objdump -S -d initialload.o > initialload.ass
	
initialload-upload:	initialload.hex
	avrdude ${AVRDUDEFLAGS} -U flash:w:initialload.hex:a
