#ifndef _STM8_EEP_H
#define _STM8_EEP_H
/*==================================================================
  File Name    : eep.h
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This is the header-file for eep.c
  ------------------------------------------------------------------
  EEP is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  EEP is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with EEP.  If not, see <http://www.gnu.org/licenses/>.
  ==================================================================*/ 
#include <stdint.h>

// EEPROM base address within STM8 uC
#define EEP_BASE_ADDR (0x4000)

// Function prototypes
uint16_t eeprom_read_config(uint8_t eeprom_address);
void     eeprom_write_config(uint8_t eeprom_address,uint16_t data);

#endif
