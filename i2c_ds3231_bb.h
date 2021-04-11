/*==================================================================
  File Name    : i2c_ds3231_bb.h
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains the DS3231 related functions.
            The DS3231 is a Real-Time Clock (RTC).
  ------------------------------------------------------------------
  This is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This file is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this software. If not, see <http://www.gnu.org/licenses/>.
  ==================================================================
*/ 
#ifndef _I2C_DS3231_BB_H
#define _I2C_DS3231_BB_H

#include <stdbool.h>
#include <stdint.h>
      
// DS3231 DOW defines
#define MONDAY		(1)
#define TUESDAY		(2)
#define WEDNESDAY	(3)
#define THURSDAY	(4)
#define FRIDAY		(5)
#define SATURDAY	(6)
#define SUNDAY		(7)

#define SQW_RATE_1	(0)
#define SQW_RATE_1K	(1)
#define SQW_RATE_4K	(2)
#define SQW_RATE_8K	(3)

#define OUTPUT_SQW	(0)
#define OUTPUT_INT	(1)

typedef struct _Time
{
    uint8_t	hour;
    uint8_t	min;
    uint8_t	sec;
    uint8_t	day;
    uint8_t	mon;
    uint16_t	year;
    uint8_t	dow;
} Time;

#define DS3231_ADR  (0xD0)

#define REG_SEC		(0x00)
#define REG_MIN		(0x01)
#define REG_HOUR	(0x02)
#define REG_DOW		(0x03)
#define REG_DATE	(0x04)
#define REG_MON		(0x05)
#define REG_YEAR	(0x06)
#define REG_CON		(0x0E)
#define REG_STATUS  (0x0F)
#define REG_AGING	(0x10)
#define REG_TEMPM	(0x11)
#define REG_TEMPL	(0x12)

// Function prototypes for DS3231
bool    ds3231_gettime(Time *p);
void    ds3231_settime(uint8_t hour, uint8_t min, uint8_t sec);
uint8_t ds3231_calc_dow(uint8_t date, uint8_t mon, uint16_t year);
void    ds3231_setdate(uint8_t date, uint8_t mon, uint16_t year);
void    ds3231_setdow(uint8_t dow);
int16_t ds3231_gettemp(void);

#endif
