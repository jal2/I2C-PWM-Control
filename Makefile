NAME = twi2pwm
MCU = attiny85 # if you change here, adapt the AVRDUDE -p argument below as well!
F_CPU = 8000000 # freq in Hz - we need to run at 8Mhz for 400kHz operation on twi bus
SOURCES = main.c usi_i2c_slave.c
CFLAGS += -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os
AVRDUDE = avrdude -c usbasp -p t85 -u

# calculated on http://www.engbedded.com/fusecalc/
# internal  Osc 8MHz, no divider, serial program downloading enabled
# brown out detection at 2.7V
FUSES = -U lfuse:w:0xe2:m -U hfuse:w:0xdd:m -U efuse:w:0xff:m 

all: $(NAME).hex

$(NAME).hex: $(NAME).elf
	avr-objcopy -O ihex -j .text -j .data $(NAME).elf $(NAME).hex

$(NAME).elf: $(SOURCES)
	avr-gcc $(CFLAGS) $^ -o $@
	avr-size --mcu=$(MCU) -C $@

clean:
	-@rm $(NAME).elf $(NAME).hex

flash: all
	$(AVRDUDE) -U flash:w:$(NAME).hex:i
fuse:
	$(AVRDUDE) $(FUSES)

help:
	@echo "make all - build the hex file"
	@echo "make clean - remove all generated files"
	@echo "make flash - build hex file and flash it with an USBasp into a ATtiny85"
	@echo "make fuse - program the fuses of an ATtiny85 with an USBasp"

.PHONY: clean flash fuse all help
