/*
 * Copyright (c) 2014, Majenko Technologies
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 *  1. Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 * 
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of Majenko Technologies nor the names of its contributors may be used
 *     to endorse or promote products derived from this software without 
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _SOFTSPI_H
#define _SOFTSPI_H

#include <avr/io.h>

#define SPI_PORT PORTA
#define SPI_DDR DDRA
#define SPI_PIN PINA

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_CLOCK_DIV2 0x00
#define SPI_CLOCK_DIV4 0x01
#define SPI_CLOCK_DIV8 0x02
#define SPI_CLOCK_DIV16 0x03
#define SPI_CLOCK_DIV32 0x04
#define SPI_CLOCK_DIV64 0x05
#define SPI_CLOCK_DIV128 0x06

#define SPI_MODE0 0x00
#define SPI_MODE1 0x01
#define SPI_MODE2 0x02
#define SPI_MODE3 0x03

#define LOW 0
#define HIGH 1

void wait(uint_fast8_t del);
uint8_t _cke;
uint8_t _ckp;
uint8_t _delay;
uint8_t _miso;
uint8_t _mosi;
uint8_t _sck;
uint8_t _order;

void SPI_init(uint8_t mosi, uint8_t miso, uint8_t sck);
void SPI_begin();
void SPI_end();
void SPI_setBitOrder(uint8_t);
void SPI_setDataMode(uint8_t);
void SPI_setClockDivider(uint8_t);
uint8_t SPI_transfer(uint8_t);
uint16_t SPI_transfer16(uint16_t data);
void digitalWrite(uint8_t pin, uint8_t val);
#endif