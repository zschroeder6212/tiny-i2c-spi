/*
SoftwareSerial.h (formerly NewSoftSerial.h) - 
Single-instance software serial library for ATtiny84A and ATtiny85, modified
from Arduino SoftwareSerial.
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)
-- Port to ATtiny84A / C by Michael Shimniok (http://www.bot-thoughts.com/)

Notes on the ATtiny84A port. To save space I've:
 - Converted back to C
 - Removed the ability to have mulitple serial ports,
 - Hardcoded the RX pin to PA0 and TX pin to PA1
 - Using & mask versus modulo (%)
 - A few other tweaks to get the code near 1k
More notes:
 - Converted from Arduinoish library stuff (pins etc)
 - Error is too high at 57600 (0.64%) and 115200 (2.12%)
 - Ok at 38400 and lower.
 - Still working out how to prevent missing bits when writing characters

-- Port to ATtiny85 by Stephen Ludin ( sludin@ludin.org )

Notes on attiny85 port and updates
- Added tables for 1Mhz and 8Mhz usage.  1Mhz is mostly useless
  above 4800
- Added a mode for calculating the delay table on the fly in software based
  on comment here: http://forum.arduino.cc/index.php?topic=138497.0. This
  is the default mode and can be reverted by defining
  SOFTWARE_SERIAL_TABLE_LOOKUP
- Added the ability to set the PORT DDR and PIN in a call to a new
  function softSerialInit.   It should be noted that this causes
  the compiler to move from birest addressign to indirect causing the
  instructions to take a bit more time.  This should not be a factor
  but it is worth noting.
- Added the ability to have a transmit only mode ( set rx pin to
  SOFTWARE_SERIAL_RX_DISABLED )
- Note that it is necessary to define F_CPU now

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SoftwareSerial_h
#define SoftwareSerial_h

#include <inttypes.h>
#include <stddef.h>


//
// Definitions
//
#define _SS_MAX_RX_BUFF 64 // RX buffer size, must be (1<<n)
#define _SS_RX_BUFF_MASK (_SS_MAX_RX_BUFF-1)
#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#define SOFTWARE_SERIAL_RX_DISABLED 0xFF

//
// public methods
//
void softSerialInit( volatile uint8_t *ddr,
                     volatile uint8_t *port,
                     volatile uint8_t *pin,
                     uint8_t rx,
                     uint8_t tx );
uint8_t softSerialBegin(long speed);
void softSerialEnd();
uint8_t softSerialOverflow();
int softSerialPeek();
size_t softSerialWrite(uint8_t byte);
int softSerialRead();
int softSerialAvailable();
void softSerialFlush();

#endif
