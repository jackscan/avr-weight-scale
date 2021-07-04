DEVICE     = attiny85
CLOCK      = 8000000UL

PROGRAMMER = -c avrispmkII
OBJECTS    = main.o nvm.o hx711.o twi.o
FUSES      = -U efuse:w:0xFF:m -U hfuse:w:0xDF:m -U lfuse:w:0x62:m

DEFINES    = -DF_CPU=$(CLOCK) -DNO_SERIAL #-DENABLE_CHECKPOINTS

TARGET     = weight-scale

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE)
COMPILE = avr-gcc -std=gnu99 -g -Wall -Wno-unused-function -Os $(DEFINES) \
                  -mmcu=$(DEVICE) -fshort-enums #-Wl,-u,vfprintf -lprintf_min
OBJDUMP = avr-objdump

GIT_TAG := $(shell git describe --tags --abbrev=0  --always)

all: $(TARGET).hex

version.h.tmp: FORCE
	@echo "#ifndef GIT_HASH" > version.h.tmp
	@echo "#define VERSION_MAJOR $(word 1,$(subst ., ,$(GIT_TAG)))" >> version.h.tmp
	@echo "#define VERSION_MINOR $(word 2,$(subst ., ,$(GIT_TAG)))" >> version.h.tmp
	@echo "#define VERSION_PATCH $(shell git log --oneline $(GIT_TAG)..HEAD| wc -l)" >> version.h.tmp
	@echo '#define GIT_HASH 0x$(shell git rev-parse --short=4 --abbrev-commit HEAD)' >> version.h.tmp
	@echo "#define GIT_DIRTY $(shell git diff --quiet; echo $$?)" >> version.h.tmp
	@echo "#endif" >> version.h.tmp

version.h: version.h.tmp
	@diff $@ $< && rm $< || mv $< $@

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

$(OBJECTS): nvm.h twi.h hx711.h util.h debug.h version.h Makefile

.PHONY: FORCE
FORCE:
