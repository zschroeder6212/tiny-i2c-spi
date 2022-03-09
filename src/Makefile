MCU		:= attiny84
programmer	:= usbtiny

CC	:= avr-gcc
OBJCOPY	:= avr-objcopy
SIZE	:= avr-size
AVRDUDE	:= avrdude
CFLAGS	:= -Wall -Wextra -Os -mmcu=$(MCU)

.PHONY: all
all: i2c-spi-bridge.elf

.PHONY: clean
clean:
	rm -f *.o i2c-spi-bridge.hex i2c-spi-bridge.elf

.PHONY: size
size: i2c-spi-bridge.elf
	$(SIZE) --mcu=$(MCU) --format=avr $<

.PHONY: flash
flash: i2c-spi-bridge.hex size
	$(AVRDUDE) -c $(programmer) -p $(MCU) -U flash:w:$<

.PHONY: fuses
fuses:
	$(AVRDUDE) -c $(programmer) -p $(MCU) -U lfuse:w:0xe2:m

i2c-spi-bridge.elf: main.o USI_TWI_Slave.o SoftSPI.o SoftwareSerial.o
	$(CC) $(CFLAGS) -o $@ $^

i2c-spi-bridge.hex: i2c-spi-bridge.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@
