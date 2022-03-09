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


#include "SoftSPI.h"

uint8_t _cke;
uint8_t _ckp;
uint8_t _delay;
uint8_t _miso;
uint8_t _mosi;
uint8_t _sck;
uint8_t _order;

void digitalWrite(uint8_t pin, uint8_t val)
{
    if (val == LOW) {
        SPI_PORT &= ~(1 << pin);
    } else {
        SPI_PORT |= (1 << pin);
    }
}

void SPI_init(uint8_t mosi, uint8_t miso, uint8_t sck) {
    _mosi = mosi;
    _miso = miso;
    _sck = sck;
    _delay = 2;
    _cke = 0;
    _ckp = 0;
    _order = MSBFIRST;
}

void SPI_begin() {
    SPI_DDR |= (1 << _mosi);
    SPI_DDR |= (0 << _miso);
    SPI_DDR |= (1 << _sck);
    
    /*
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    pinMode(_sck, OUTPUT);
    */
}

void SPI_end() {
    SPI_DDR |= (0 << _mosi);
    SPI_DDR |= (0 << _miso);
    SPI_DDR |= (0 << _sck);
    /*
    pinMode(_mosi, INPUT);
    pinMode(_miso, INPUT);
    pinMode(_sck, INPUT);
    */
}

void SPI_setBitOrder(uint8_t order) {
    _order = order & 1;
}

void SPI_setDataMode(uint8_t mode) {
    switch (mode) {
        case SPI_MODE0:
            _ckp = 0;
            _cke = 0;
            break;
        case SPI_MODE1:
            _ckp = 0;
            _cke = 1;
            break;
        case SPI_MODE2:
            _ckp = 1;
            _cke = 0;
            break;
        case SPI_MODE3:
            _ckp = 1;
            _cke = 1;
            break;
    }

    //SPI_PORT |= (_ckp << _sck);
    digitalWrite(_sck, _ckp ? HIGH : LOW);
}

void SPI_setClockDivider(uint8_t div) {
    switch (div) {
        case SPI_CLOCK_DIV2:
            _delay = 2;
            break;
        case SPI_CLOCK_DIV4:
            _delay = 4;
            break;
        case SPI_CLOCK_DIV8:
            _delay = 8;
            break;
        case SPI_CLOCK_DIV16:
            _delay = 16;
            break;
        case SPI_CLOCK_DIV32:
            _delay = 32;
            break;
        case SPI_CLOCK_DIV64:
            _delay = 64;
            break;
        case SPI_CLOCK_DIV128:
            _delay = 128;
            break;
        default:
            _delay = 128;
            break;
    }
}

void SPI_wait(uint_fast8_t del) {
    for (uint_fast8_t i = 0; i < del; i++) {
        asm volatile("nop");
    }
}

uint8_t SPI_transfer(uint8_t val) {
    uint8_t out = 0;
    if (_order == MSBFIRST) {
        uint8_t v2 = 
            ((val & 0x01) << 7) |
            ((val & 0x02) << 5) |
            ((val & 0x04) << 3) |
            ((val & 0x08) << 1) |
            ((val & 0x10) >> 1) |
            ((val & 0x20) >> 3) |
            ((val & 0x40) >> 5) |
            ((val & 0x80) >> 7);
        val = v2;
    }

    uint8_t del = _delay >> 1;

    uint8_t bval = 0;
    /*
     * CPOL := 0, CPHA := 0 => INIT = 0, PRE = Z|0, MID = 1, POST =  0
     * CPOL := 1, CPHA := 0 => INIT = 1, PRE = Z|1, MID = 0, POST =  1
     * CPOL := 0, CPHA := 1 => INIT = 0, PRE =  1 , MID = 0, POST = Z|0
     * CPOL := 1, CPHA := 1 => INIT = 1, PRE =  0 , MID = 1, POST = Z|1
     */

    int sck = (_ckp) ? HIGH : LOW;

    for (uint8_t bit = 0u; bit < 8u; bit++)
    {
        if (_cke) {
            sck ^= 1;
            //SPI_PORT |= (_ckp << sck);
            digitalWrite(_sck, sck);            
            SPI_wait(del);
        }

        /* ... Write bit */
        //SPI_PORT |= (((val & (1<<bit)) << _mosi));
        digitalWrite(_mosi, ((val & (1<<bit)) ? HIGH : LOW));

        SPI_wait(del);

        sck ^= 1u; 
        //SPI_PORT |= (sck << _sck);
        digitalWrite(_sck, sck);

        /* ... Read bit */
        {
            bval = ((SPI_PIN & (1 << _miso)) >> _miso);
            //bval = digitalRead(_miso);

            if (_order == MSBFIRST) {
                out <<= 1;
                out |= bval;
            } else {
                out >>= 1;
                out |= bval << 7;
            }
        }

        SPI_wait(del);

        if (!_cke) {
            sck ^= 1u;
            //SPI_PORT |= (sck << _sck);
            digitalWrite(_sck, sck);
        }
    }

    return out;
}

uint16_t SPI_transfer16(uint16_t data)
{
	union {
		uint16_t val;
		struct {
			uint8_t lsb;
			uint8_t msb;
		};
	} in, out;
  
	in.val = data;

	if ( _order == MSBFIRST ) {
		out.msb = SPI_transfer(in.msb);
		out.lsb = SPI_transfer(in.lsb);
	} else {
		out.lsb = SPI_transfer(in.lsb);
		out.msb = SPI_transfer(in.msb);
	}

	return out.val;
}