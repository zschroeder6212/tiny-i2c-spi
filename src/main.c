#define F_CPU 8000000UL

#define DEBUG

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h> 

#include "SoftSPI.h"
#include "USI_TWI_Slave.h"

#ifdef DEBUG
#include "SoftwareSerial.h"
#endif


#define I2C_ADDR 0x28

#define MOSI PA0
#define MISO PA1
#define SCK  PA2

#define CS_DDR DDRA
#define CS_PORT PORTA
#define CS PA3

#define BUFFER_SIZE 128


#define CMD_TRANSMIT    1
#define CMD_CONFIGURE   2

#define MASK_SPI_MODE       0x03
#define MASK_SPI_ORDER      0x04
#define MASK_SPI_CLOCK_DIV  0x38

uint8_t SPI_buffer[BUFFER_SIZE];

uint8_t received_index = 0;
uint8_t transmit_index = 0;

#ifdef DEBUG
void ss_puts(char* s)
{
    while(*s != '\0')
    {
        softSerialWrite(*s++);
    }
}

void ss_putn(uint8_t val, uint8_t base)
{
  char out[9];
  itoa(val, out, base);
  ss_puts(out);
}
#endif

void I2C_received(uint8_t bytes_recieved)
{
    uint8_t command = usiTwiReceiveByte();

    switch(command)
    {
        case CMD_TRANSMIT:
        {
            CS_PORT &= ~(1<<CS);

            for(int i = 1; i < bytes_recieved; i++)
            {
                uint8_t received_byte = SPI_transfer(usiTwiReceiveByte());
#ifdef DEBUG
                ss_putn(received_byte, 16);
                softSerialWrite(' ');
#endif
                SPI_buffer[received_index++] = received_byte;

                if(received_index >= BUFFER_SIZE)
                {
                    received_index = 0;
                }
            }
#ifdef DEBUG
                softSerialWrite('\n');
#endif

            CS_PORT |= (1<<CS);
            break;
        }
        case CMD_CONFIGURE:
        {

            /*
             *
             *   !!!UNTESTED!!!   
             *
             *   bit 0 is CPOL
             *   bit 1 is CPHA
             *   bit 2 is bit order 0=LSB firts 1=MSB first
             *   bit 3-6 is the clock divider
             *       SPI_CLOCK_DIV2 0x00
             *       SPI_CLOCK_DIV4 0x01
             *       SPI_CLOCK_DIV8 0x02
             *       SPI_CLOCK_DIV16 0x03
             *       SPI_CLOCK_DIV32 0x04
             *       SPI_CLOCK_DIV64 0x05
             *       SPI_CLOCK_DIV128 0x06
             *   bit 7-8 is unused
             *
             */

            uint8_t received_byte = usiTwiReceiveByte();

            SPI_setClockDivider((received_byte>>3)&MASK_SPI_CLOCK_DIV);
            SPI_setDataMode(received_byte&MASK_SPI_MODE);
            SPI_setBitOrder((received_byte>>2)&MASK_SPI_ORDER);
        }
    }
}

void I2C_requested()
{
    usiTwiTransmitByte(SPI_buffer[transmit_index++]);
    if(transmit_index >= BUFFER_SIZE)
    {
        transmit_index = 0;
    }
}

int main (void)
{   
    /* init I2C */
    usiTwiSlaveInit(I2C_ADDR);

    /* set received/requested callbacks */
    usi_onReceiverPtr = I2C_received;
    usi_onRequestPtr = I2C_requested;

    /* enable interrupts */
    sei();

    /* init SPI */
    SPI_init(MOSI, MISO, SCK);
    SPI_begin();
    SPI_setClockDivider(SPI_CLOCK_DIV2);
    SPI_setDataMode(0b00);
    SPI_setBitOrder(MSBFIRST);

    CS_DDR |= (1<<CS);
    CS_PORT |= (1<<CS);

#ifdef DEBUG
    softSerialInit(&DDRB, &PORTB, &PINB, PB1, PB2);
    softSerialBegin(9600);
#endif

	while(true);
}