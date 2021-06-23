MCU = attiny84
programmer = usbtiny

all: main.c
	avr-gcc -mmcu=$(MCU) -Wall -Os -o i2c-spi-bridge.elf main.c USI_TWI_Slave.c SoftSPI.c SoftwareSerial.c
	avr-objcopy -j .text -j .data -O ihex i2c-spi-bridge.elf i2c-spi-bridge.hex
	avr-size --mcu=$(MCU) --format=avr i2c-spi-bridge.elf

flash: all
	avrdude -c $(programmer) -p $(MCU) -U flash:w:i2c-spi-bridge.hex

fuses:
	avrdude -c $(programmer) -p $(MCU) -U lfuse:w:0xe2:m

# make clean - remove .o files and the executable file.
clean:
	rm -f *.o i2c-spi-bridge.hex i2c-spi-bridge.elf 