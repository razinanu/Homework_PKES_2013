TARGET  = main
ASRC    = counter_func

CTRL    = atmega2560
INCLUDE = ../include
STK     = avr911        # Seriel connection
STK     = avrispmkII    # Atmel ISP Programmer
STK     = stk500v2      # Arduino 2560 ADK via USB

CC      = avr-gcc
OC      = avr-objcopy
LDFLAGS = -j .text -j .data

AVRDUDE = avrdude
AVRDUDE_FLAGS = -P /dev/ttyUSB0
AVRDUDE_FLAGS = -P usb
AVRDUDE_FLAGS = -P /dev/ttyACM0
AVRDUDE_FLAGS += -b 115200
AVRDUDE_FLAGS += -c $(STK)
AVRDUDE_FLAGS += -p $(CTRL)
AVRDUDE_FLAGS += -U f:w:$(TARGET).hex:a

.PHONY: all

all: 
	@echo "compiling"
	$(CC) -c -o $(TARGET).o -g -O -Wall -I $(INCLUDE) -mmcu=$(CTRL) $(TARGET).c
	$(CC) -c -o $(ASRC).o -g -O -Wall -I $(INCLUDE) -mmcu=$(CTRL) $(ASRC).S
	@echo "linking"
	@$(CC) -g -Wall -O -mmcu=$(CTRL) -o $(TARGET).elf $(TARGET).o $(ASRC).o
	@echo "generate properly output format"
	@$(OC) $(LDFLAGS) -O ihex $(TARGET).elf $(TARGET).hex

program: all
	@echo "Programing the device"
	$(AVRDUDE) $(AVRDUDE_FLAGS)

asm:	all
	@echo "Generating assembler code"
	avr-objdump -S $(TARGET).elf >  $(TARGET).asm

clean:
	@echo "Cleaning"
	@rm -rf $(TARGET).hex $(TARGET).o $(TARGET).elf *~

com:    program
	@echo "Opening Serial connection"
	gtkterm -p /dev/ttyACM0 -s 9600 -r "-"