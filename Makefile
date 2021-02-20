DEVICE     = attiny85
CLOCK      = 8000000UL

PROGRAMMER = -c avrispmkII
OBJECTS    = main.o nvm.o hx711.o twi.o
FUSES      = -U efuse:w:0xFF:m -U hfuse:w:0xDF:m -U lfuse:w:0x62:m

DEFINES    = -DF_CPU=$(CLOCK) -DNO_SERIAL #-DENABLE_CHECKPOINTS

TARGET     = weight-scale

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -std=gnu99 -g -Wall -Wno-unused-function -Os $(DEFINES) \
                  -mmcu=$(DEVICE) -Wl,-u,vfprintf -lprintf_min -fshort-enums
OBJDUMP = avr-objdump

all: $(TARGET).hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

flash: all
	$(AVRDUDE) -U flash:w:$(TARGET).hex:i

fuse:
	$(AVRDUDE) $(FUSES)

clean:
	rm -f $(TARGET).hex $(TARGET).elf $(OBJECTS) $(DBGOBJS)

$(TARGET).elf: $(OBJECTS)
	$(COMPILE) -o $(TARGET).elf $^

%.hex: %.elf
	rm -f $@
	avr-objcopy -j .text -j .data -O ihex $< $@
	avr-size --format=avr --mcu=$(DEVICE) $<

%.eep: %.elf
	rm -f $@
	avr-objcopy -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@

disasm: $(TARGET).elf
	avr-objdump -d $<

cpp:
	$(COMPILE) -E main.c

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

$(OBJECTS): nvm.h twi.h hx711.h util.h debug.h Makefile
