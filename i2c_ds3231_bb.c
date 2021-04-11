/*==================================================================
  File Name    : i2c_ds3231_bb.c
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
#include "i2c_bb.h"
#include "i2c_ds3231_bb.h"

bool ds3231_read_register(uint8_t reg, uint8_t *value)
{
    bool err;

    err = (i2c_start_bb(DS3231_ADR | I2C_WRITE) != I2C_ACK); // generate I2C start + output address to I2C bus
    if (!err) 
    { 
        i2c_write_bb(reg);      // write register address to read from
        i2c_rep_start_bb(DS3231_ADR | I2C_READ);
        *value = i2c_read_bb(I2C_NACK); // Read register + issue NACK
    } // if
    i2c_stop_bb();          // i2c_stop() condition
    return err;
} // ds3231_read_register()
 
bool ds3231_write_register(uint8_t reg, uint8_t value)
{
    bool err;

    err = (i2c_start_bb(DS3231_ADR | I2C_WRITE) != I2C_ACK); // generate I2C start + output address to I2C bus
    if (!err) 
    {
        i2c_write_bb(reg);   // write register address to write to
        i2c_write_bb(value); // write value into register
    } // if
    i2c_stop_bb();       // close I2C bus
    return err;
} // ds3231_write_register()

uint8_t	ds3231_decode(uint8_t value)
{
    uint8_t decoded = value & 0x7F;

    decoded = (decoded & 0x0F) + 10 * ((decoded & (0xF0)) >> 4);
    return decoded;
} // ds3231_decode()

uint8_t ds3231_decodeH(uint8_t value)
{
    if (value & 0x40) // 12 hour format
        value = (value & 0x0F) + ((value & 0x20) ? 10 : 0);
    else 
    {   // 24 hour format
        value = (value & 0x0F) + (5 * ((value & 0x30) >> 3));
    } // else
    return value;
} // ds3231_decodeH()

uint8_t	ds3231_decodeY(uint8_t value)
{
    uint8_t decoded = (value & 0x0F) + 10 * ((value & 0xF0) >> 4);
    return decoded;
} // ds3231_decodeY()

uint8_t ds3231_encode(uint8_t value)
{
    uint8_t encoded = ((value / 10) << 4) + (value % 10);
    return encoded;
} // ds3231_encode()

bool ds3231_gettime(Time *p)
{
	bool    err;
	uint8_t buf;

	err = (i2c_start_bb(DS3231_ADR | I2C_WRITE) != I2C_ACK); // generate I2C start + output address to I2C bus
	if (!err) 
        {
            i2c_write_bb(REG_SEC); // seconds register is first register to read
            i2c_rep_start_bb(DS3231_ADR | I2C_READ);
            
            buf     = i2c_read_bb(I2C_ACK);  // Read SECONDS register
            p->sec  = ds3231_decode(buf);     
            buf     = i2c_read_bb(I2C_ACK);  // Read MINUTES register
            p->min  = ds3231_decode(buf);     
            buf     = i2c_read_bb(I2C_ACK);  // Read HOURS register
            p->hour = ds3231_decodeH(buf);    
            buf     = i2c_read_bb(I2C_ACK);  // Read DOW register
            p->dow  = buf;    
            buf     = i2c_read_bb(I2C_ACK);  // Read DAY register
            p->day  = ds3231_decode(buf);
            buf     = i2c_read_bb(I2C_ACK);  // Read MONTH register
            p->mon  = ds3231_decode(buf);
            buf     = i2c_read_bb(I2C_NACK); // Read YEAR register
            p->year = 2000 + ds3231_decodeY(buf); 
	} // if
	i2c_stop_bb();
	return err;
} // ds3231_gettime()

void ds3231_settime(uint8_t hour, uint8_t min, uint8_t sec)
{
	if ((hour < 24) && (min < 60) && (sec < 60))
	{
		ds3231_write_register(REG_HOUR, ds3231_encode(hour));
		ds3231_write_register(REG_MIN , ds3231_encode(min));
		ds3231_write_register(REG_SEC , ds3231_encode(sec));
	} // if	
} // ds3231_settime()

// 1=Monday, 2=Tuesday, 3=Wednesday, 4=Thursday, 5=Friday, 6=Saturday, 7=Sunday
uint8_t ds3231_calc_dow(uint8_t date, uint8_t mon, uint16_t year)
{
	uint32_t JND = date + ((153L * (mon + 12 * ((14 - mon) / 12) - 3) + 2) / 5)
	+ (365L * (year + 4800L - ((14 - mon) / 12)))
	+ ((year + 4800L - ((14 - mon) / 12)) / 4)
	- ((year + 4800L - ((14 - mon) / 12)) / 100)
	+ ((year + 4800L - ((14 - mon) / 12)) / 400)
	- 32044L;
        JND = JND % 7;
        if (JND == 0) JND = 7;
	return JND;
} // ds3231_calc_dow()

void ds3231_setdate(uint8_t date, uint8_t mon, uint16_t year)
{
	uint8_t dow;
	
	if (((date > 0) && (date <= 31)) && ((mon > 0) && (mon <= 12)) && ((year >= 2000) && (year < 3000)))
	{
		dow = ds3231_calc_dow(date, mon, year);
		ds3231_write_register(REG_DOW,dow);
		year -= 2000;
		ds3231_write_register(REG_YEAR, ds3231_encode(year));
		ds3231_write_register(REG_MON , ds3231_encode(mon));
		ds3231_write_register(REG_DATE, ds3231_encode(date));
	} // if
} // ds3231_setdate()

void ds3231_setdow(uint8_t dow)
{
	if ((dow > 0) && (dow < 8))
		ds3231_write_register(REG_DOW, dow);
} // ds3231_setdow() 

// Returns the Temperature in a Q8.2 format
int16_t ds3231_gettemp(void)
{
	bool err;
	uint8_t msb,lsb = 0;
	int16_t retv;
	
	err    = ds3231_read_register(REG_TEMPM, &msb);
	retv   = msb;
	retv <<= 2; // SHL 2
	if (!err) err = ds3231_read_register(REG_TEMPL, &lsb);
	retv  |= (lsb >> 6);
	if (retv & 0x0200)
	{   // sign-bit is set
		retv &= ~0x0200; // clear sign bit
		retv = -retv;    // 2-complement
	} // if
	if (!err) return retv;
	else      return 0;
} // ds3231_gettemp()
