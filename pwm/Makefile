FILE    = test

CTRL    = atmega2560
STK     = stk500v2      # Arduino 2560 ADK via USB

CC      = avr-gcc
OC      = avr-objcopy
LDFLAGS = -j .text -j .data

AVRDUDE = avrdude
AVRDUDE_FLAGS = -P /dev/ttyACM0
AVRDUDE_FLAGS += -b 115200
AVRDUDE_FLAGS += -c $(STK)
AVRDUDE_FLAGS += -p $(CTRL)
AVRDUDE_FLAGS += -U f:w:$(FILE).hex:a

.PHONY: all

all: 
	@echo "compiling"
	$(CC) -c -o $(FILE).o -x assembler-with-cpp -mmcu=$(CTRL) $(FILE).S
	$(CC) -g -O -Wall -mmcu=$(CTRL) -o $(FILE).elf  $(FILE).o
	@echo "generate properly output format"
	@$(OC) $(LDFLAGS) -O ihex $(FILE).elf $(FILE).hex

program: all
	@echo "Programing the device"
	$(AVRDUDE) $(AVRDUDE_FLAGS)

asm:	all
	@echo "Generating assembler code"
	avr-objdump -S $(FILE).elf >  $(FILE).asm

clean:
	@echo "Cleaning"
	@rm -rf $(FILE).hex $(FILE).o $(FILE).elf *~