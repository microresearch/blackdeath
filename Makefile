MCU=atmega128
CC=avr-gcc
CEXTRA=-Wa,-adhlns=$(<:.c=.lst)
#EXTERNAL_RAM = -Wl,--defsym=__heap_start=0x801100,--defsym=__heap_end=0x80ffff
#EXTERNAL_RAM = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff
LDFLAGS  = -mmcu=${MCU} ${EXTERNAL_RAM} -Wl,-u,vfprintf -lprintf_flt -lm
OBJCOPY=avr-objcopy
# optimize for size:
CFLAGS=-g -mmcu=$(MCU) -Wall -Wstrict-prototypes -O3 -mcall-prologues ${CEXTRA}
TARGET=blackdeath

DEVICE = m128
AVRDUDE = avrdude -c usbasp -p $(DEVICE)
FUSEH = 0x99
FUSEE = 0xff
FUSEL = 0xff


#-------------------
all: blackdeath.hex
#-------------------
help: 
	@echo "Usage: make all|load|load_pre|rdfuses|wrfuse1mhz|wrfuse4mhz|wrfusecrystal"
	@echo "Warning: you will not be able to undo wrfusecrystal unless you connect an"
	@echo "         external crystal! uC is dead after wrfusecrystal if you do not"
	@echo "         have an external crystal."
#-------------------
blackdeath.hex : blackdeath.out 
	$(OBJCOPY) -R .eeprom -O ihex blackdeath.out blackdeath.hex 
#blackdeath.out : blackdeath.o 
#	$(CC) $(CFLAGS) -o blackdeath.out -Wl,-Map,blackdeath.map blackdeath.o 
blackdeath.out : blackdeath.o 
	$(CC) ${LDFLAGS} $(CFLAGS) -o blackdeath.out  blackdeath.o -lm


blackdeath.o : blackdeath.c 
	$(CC) $(CFLAGS) -Os -c blackdeath.c

blackdeath.elf: blackdeath.o
	$(CC) ${LDFLAGS} $(CFLAGS) -o blackdeath.elf blackdeath.o -lm

disasm:	blackdeath.elf
	avr-objdump -d noise.elf

fuse:
	$(AVRDUDE) -U hfuse:w:$(FUSEH):m -U lfuse:w:$(FUSEL):m -U efuse:w:$(FUSEE):m

flash: all
	$(AVRDUDE) -U flash:w:blackdeath.hex:i


#-------------------
clean:
	rm -f *.o *.map *.out *t.hex
#-------------------
