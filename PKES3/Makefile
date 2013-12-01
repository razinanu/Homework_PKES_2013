BOARD     := mega2560
SERIALDEV := /dev/ttyACM0

include arduino.mk

gtkterm:
	gtkterm -p $(SERIALDEV)
	
screen: monitor