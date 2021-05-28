/*==================================================================
  File Name    : main.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This is the main-file for the Clock SSD STM8S105 project.
  ------------------------------------------------------------------
  This is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this software.  If not, see <http://www.gnu.org/licenses/>.
  ================================================================== */ 
#include "main.h"
#include "delay.h"
#include "scheduler.h"
#include "i2c_bb.h"
#include "i2c_ds3231_bb.h"
#include "uart.h"
#include "eep.h"

extern uint32_t t2_millis;         // Updated in TMR2 interrupt

char     rs232_inbuf[UART_BUFLEN]; // buffer for RS232 commands
uint8_t  rs232_ptr     = 0;        // index in RS232 buffer
char     ssd_clk_ver[] = "Clock SSD S105 v0.46\n";
// Bit-order: 0abcdefg. Digits: 0123456789 -bE°CPvt
uint8_t  ssd[19] = {0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B,
                    0x00,0x01,0x1F,0x4F,0x63,0x4E,0x67,0x3E,0x0F};

uint8_t led_r[NR_LEDS];           // Array with 8-bit red colour for all WS2812
uint8_t led_g[NR_LEDS];           // Array with 8-bit green colour for all WS2812
uint8_t led_b[NR_LEDS];           // Array with 8-bit blue colour for all WS2812
bool    enable_test_pattern = false; // true = enable WS2812 test-pattern
uint8_t show_date_IR = IR_SHOW_TIME; // What to display on the 7-segment displays
uint8_t set_time_IR  = IR_NO_TIME;   // Show normal time or blanking begin/end time
bool    set_color_IR = false;     // true = set color intensity via IR
uint8_t watchdog_test = 0;        // 1 = watchdog test modus
uint8_t led_intensity_r;          // Intensity of WS2812 Red LEDs [1..39]
uint8_t led_intensity_g;          // Intensity of WS2812 Green LEDs [1..39]
uint8_t led_intensity_b;          // Intensity of WS2812 Blue LEDs [1..39]
bool    dst_active  = false;      // true = Daylight Saving Time active
Time    dt;                       // Struct with time and date values, updated every sec.
bool    powerup         = true;
bool    set_col_white   = false; // true = esp8266 time update was successful
bool    blanking_invert = false; // Invert blanking-active IR-command
bool    enable_test_IR  = false; // Enable Test-pattern IR-command
bool    last_esp8266    = false; // true = last esp8266 command was successful
uint8_t  esp8266_std    = ESP8266_INIT; // update time from ESP8266 every 18 hours
uint16_t esp8266_tmr    = 0;     // timer for updating ESP8266

uint8_t blank_begin_h  = 23;     // Blanking begin-time in hours
uint8_t blank_begin_m  = 30;     // Blanking begin-time in minutes
uint8_t blank_end_h    =  8;     // Blanking end-time in hours
uint8_t blank_end_m    = 30;     // Blanking end-time in hours
uint8_t time_arr[6];             // Array for changing time or intensity with IR
uint8_t time_arr_idx;            // Index into time_arr[]

uint8_t  tmr3_std = STATE_IDLE;  // FSM for reading IR codes
uint16_t rawbuf[100];            // buffer with clock-ticks from IR-codes
uint8_t  rawlen     = 0;         // number of bits read from IR
uint16_t prev_ticks = 0;         // previous value of ticks, used for bit-length calc.
uint32_t ir_result  = 0;         // 32 bit raw bit-code from IR is stored here 
bool     ir_rdy     = false;     // flag for ir_task() that new IR code is received
uint8_t  ir_cmd_std = IR_CMD_IDLE; // FSM state in handle_ir_command()
uint8_t  ir_cmd_tmr = 0;         // No-action timer for handle_ir_command()

//----------------------------------------------------------------------------
// These values are stored directly into EEPROM
// Note: DST_ACTIVE is stored outside this array, so that it is not initialised
//       every time new firmware is loaded
//----------------------------------------------------------------------------
__root __eeprom const int eedata[] = 
{
       0,0,0,0,0,0,0,0,0,0,0,0,0,0, // Not used
       LED_INTENSITY, /* Intensity of WS2812 blue leds */
       LED_INTENSITY, /* Intensity of WS2812 green leds */
       LED_INTENSITY, /* Intensity of WS2812 red leds */
       0,             /* not in use yet */
       23,            /* Blanking begin hours */
       30,            /* Blanking begin minutes */
       8,             /* Blanking end hours */
       30,            /* Blanking end minutes */
       0,             /* not in use yet */
       0              /* not in use yet */
}; // eedata[]

/*-----------------------------------------------------------------------------
  Purpose  : This is external interrupt routine 5 for PORTC
             It is connected to the IR output of the VS1838B.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
#pragma vector = EXTI2_vector
__interrupt void PORTC_IRQHandler(void)
{
    uint16_t diff_ticks;
    uint16_t ticks = tmr3_val(); // counts at f = 31.25 kHz, T = 32 usec.
        
    if (ticks < prev_ticks)
         diff_ticks = ~prev_ticks + ticks;
    else diff_ticks = ticks  - prev_ticks;
    if (IR_RCVb) // copy IR-signal to debug output
         IRQ_LEDb = 1;
    else IRQ_LEDb = 0; 
    switch (tmr3_std)
    {
        case STATE_IDLE:
            if (!IR_RCVb)
            {   // falling edge
                rawbuf[0] = ticks;
                rawlen    = 1;
                tmr3_std  = STATE_MARK;
            } // if
            break;
        case STATE_MARK: // A mark is a 0 for the VS1838B
            if (IR_RCVb) 
            {   // rising edge, end of mark
                if (rawlen < 99)
                {
                    rawbuf[rawlen++] = diff_ticks;
                    tmr3_std         = STATE_SPACE;
                } // if
                else 
                {   // overflow
                    ir_rdy   = true;
                    tmr3_std = STATE_STOP;
                } // else
            } // if
            break;
        case STATE_SPACE:
            if (!IR_RCVb) 
            {   // falling edge, end of space
                if (diff_ticks > 625) // 625 = 20 msec. min. gap between transmissions
                {   // long space received, ready to process everything
                    ir_rdy    = true;
                    tmr3_std  = STATE_STOP;
                } // if
                else if (rawlen < 99)
                {
                    rawbuf[rawlen++] = diff_ticks;
                    tmr3_std         = STATE_MARK;
                } // if
                else 
                {   // overflow
                    ir_rdy   = true;
                    tmr3_std = STATE_STOP;
                } // else
            } // if
            break;
        case STATE_STOP:
            // remain in this state unless ir_task() resets this
            break;
    } // switch
    prev_ticks = ticks; // save ticks value
} // PORTC_IRQHandler()

/*------------------------------------------------------------------
  Purpose  : This function checks if the IR bit timing is between a
             lower and a upper limit. All values are in clock-ticks,
             a clock-tick (timer 3) is 32 usec (f = 31.25 kHz)
  Variables: -
  Returns  : true = success ; false = error
  ------------------------------------------------------------------*/
bool check_ticks(uint16_t val, uint16_t low, uint16_t high)
{
    return ((val >= low) && (val <= high));
} // check_ticks()

/*------------------------------------------------------------------
  Purpose  : This function decodes the bits from rawbuf and stores
             the result in ir_result. It is called from ir_task().
  Variables: -
  Returns  : true = success ; false = error
  ------------------------------------------------------------------*/
bool ir_decode_nec(void)
{
    int16_t  offset = 1;  // Index in to results; Skip first entry!?
    //char     s[20];
    
    ir_result = 0; // We decode in to here; Start with nothing
    // Check we have the right amount of data (68). 
    // The +4 is for initial gap, start bit mark and space + stop bit mark.
    if ((rawlen < 68) && (rawlen != 4)) 
    {
        //sprintf(s,"len (%d) err\n",rawlen);
        //uart_printf(s);
        return false;
    } // if
    
    if (!check_ticks(rawbuf[offset], HDR_MARK_LTICKS, HDR_MARK_HTICKS))  
    {   // Check header "mark" this must be done for repeat and data
        //uart_printf("hdr mark err\n");
        return false;
    } // if
    offset++;
    
    if (rawlen == 4)
    {   // Check for repeat - this has a different header space length
        //uart_printf("rpt hdr ");
        if (check_ticks(rawbuf[offset  ], RPT_SPACE_LTICKS, RPT_SPACE_HTICKS) &&
            check_ticks(rawbuf[offset+1], BIT_MARK_LTICKS , BIT_MARK_HTICKS))
        {
            //uart_printf("ok\n");
            return true;
        } // if
        //uart_printf("err\n");
        return false; // wrong repeat header
    } // if 
    
    if (!check_ticks(rawbuf[offset], HDR_SPACE_LTICKS, HDR_SPACE_HTICKS)) 
    {   // Check command header space
        //uart_printf("hdr space err\n");
        return false; // Header space length is wrong
    } // if
    offset++;
    
    // Build the data
    while (offset < rawlen-1)
    {   // Check data "mark"
        if (!check_ticks(rawbuf[offset], BIT_MARK_LTICKS, BIT_MARK_HTICKS)) 
        {
            //sprintf(s,"mark %d err\n",offset);
            //uart_printf(s);
            return false;
        } // if
        offset++;
        // Suppend this bit
        if      (check_ticks(rawbuf[offset], ONE_SPACE_LTICKS , ONE_SPACE_HTICKS ))  ir_result = (ir_result << 1) | 1 ;
        else if (check_ticks(rawbuf[offset], ZERO_SPACE_LTICKS, ZERO_SPACE_HTICKS))  ir_result = (ir_result << 1) | 0 ;
        else 
        {
            //sprintf(s,"space %d err\n",offset);
            //uart_printf(s);
            return false;
        } // else
        offset++;
    } // while
    return true; // success
} // ir_decode_nec()

/*------------------------------------------------------------------
  Purpose  : This function retrieves the proper key from the IR code
  Variables: global variable ir_result is used. It is called from ir_task().
  Returns  : code for key found
  ------------------------------------------------------------------*/
uint8_t ir_key(void)
{
    char    s[10];
    uint8_t key;
    
    switch (ir_result)
    {
        case IR_CODE_UP      : key = IR_UP      ; break;
        case IR_CODE_DOWN    : key = IR_DOWN    ; break;
        case IR_CODE_LEFT    : key = IR_LEFT    ; break;
        case IR_CODE_RIGHT   : key = IR_RIGHT   ; break;
        case IR_CODE_OK	     : key = IR_OK      ; break;
        case IR_CODE_ASTERISK: key = IR_ASTERISK; break;
        case IR_CODE_HASH    : key = IR_HASH    ; break;
        case IR_CODE_0       : key = IR_0       ; break;
        case IR_CODE_1       : key = IR_1       ; break;
        case IR_CODE_2       : key = IR_2       ; break;
        case IR_CODE_3       : key = IR_3       ; break;
        case IR_CODE_4       : key = IR_4       ; break;
        case IR_CODE_5       : key = IR_5       ; break;
        case IR_CODE_6       : key = IR_6       ; break;
        case IR_CODE_7       : key = IR_7       ; break;
        case IR_CODE_8       : key = IR_8       ; break;
        case IR_CODE_9       : key = IR_9       ; break;
        case IR_CODE_REPEAT  : key = IR_REPEAT  ; break;
        default              : key = IR_NONE    ; break;
    } // switch
    sprintf(s,"IR[%c]\n",IR_CHARS[key]);
    uart_printf(s); // output to terminal screen
    return key;
} // ir_key()

/*-----------------------------------------------------------------------------
  Purpose  : This function checks if a BCD number is allowed for a time.
  Variables: digit: [0..9], the number proposed for a certain digit.
  Globals  : time_arr_idx: index into timer_arr[], indicates which digit is about
                           to change, 0 = left-most digit, 5 = right-most digit
             time_arr[]  : array to store the proposed digits in.
  Returns  : -
  ---------------------------------------------------------------------------*/
void check_possible_digit(uint8_t digit)
{
    if (digit > DIG_9) return; // only valid digits allowed
    
    switch (time_arr_idx)
    {
        case 0: // MSB of hours
            if (digit < DIG_3) 
            {   // only 0, 1 or 2 allowed
                time_arr[0]  = digit;
            } // else
            break;
        case 1: // LSB of hours
            if ((time_arr[0] == DIG_2) && (digit < DIG_4))
            {   // only 20, 21, 22 and 23 allowed
                time_arr[1]  = digit;
            } // if
            else if (time_arr[0] < DIG_2)
            {   // time_arr[0] == 0 || time_arr[0] == 1
                time_arr[1]  = digit;
            } // else    
            break;
        case 2: // MSB of minutes
            if (digit < DIG_6) 
            {   // only 0..5 allowed
                time_arr[2]  = digit;
            } // else
            break;
        default: // time_arr_idx == 3, LSB of minutes
            time_arr[3]  = digit;
            break;
    } // switch
} // check_possible_digit()

/*-----------------------------------------------------------------------------
  Purpose  : This function checks if a BCD number is allowed for a 
             color-intensity. The color-intensity is a number between 1 and 39.
  Variables: digit: [0..9], the number proposed for a certain digit.
  Globals  : time_arr_idx: index into timer_arr[], indicates which digit is about
                           to change, 0 = left-most digit, 5 = right-most digit
             time_arr[]  : array to store the proposed digits in.
  Returns  : -
  ---------------------------------------------------------------------------*/
void check_possible_col_digit(uint8_t digit)
{
    if (digit <= DIG_9)
    {   // only valid digits allowed
        if (!(time_arr_idx & 0x01))
        {   // even number, meaning MSB of color-intensity
            if (digit < DIG_4) time_arr[time_arr_idx] = digit; // store it
        } // if
        else if ((digit > DIG_0) || (time_arr[time_arr_idx-1] > DIG_0))
        {   // odd number, any digit allowed, except 00 number
            time_arr[time_arr_idx] = digit;
        } // else
    } // if
} // check_possible_col_digit()

/*-----------------------------------------------------------------------------
  Purpose  : This function is called every 100 msec. and initiates all actions
             derived from IR remote keys
  Variables: key: the key pressed on the IR-remote
  Returns  : -
  ---------------------------------------------------------------------------*/
void handle_ir_command(uint8_t key)
{
    static uint16_t tmr_xsec; // seconds timer
    uint8_t  x;
    uint16_t temp,t2;
    
    if (key == IR_NONE)
    {   // increment no-action timer
        if (!blanking_invert && !enable_test_IR && (++ir_cmd_tmr > 200))
        {   // back to idle after 20 seconds
            set_time_IR  = IR_NO_TIME;
            set_color_IR = false;
            ir_cmd_std   = IR_CMD_IDLE; 
            return; // exit
        } // if
    } // if
    else ir_cmd_tmr = 0; // reset timer if IR key is pressed
    
    switch (ir_cmd_std)
    {
        case IR_CMD_IDLE:
            if (key == IR_0)      ir_cmd_std = IR_CMD_0;
            else if (key == IR_1) 
            {
                ir_cmd_std = IR_CMD_1; // show version number for 5 seconds
                tmr_xsec   = 0;
            } // else if
            else if (key == IR_2) 
            {
                ir_cmd_std = IR_CMD_2; // show last response status from ESP8266
                tmr_xsec = 0;
            } // else if
            else if (key == IR_3) 
            {
                uart_printf("e0\n");   // Get Date & Time from ESP8266 NTP server
                tmr_xsec = 0;
            } // else if
            else if (key == IR_4) 
            {
                ir_cmd_std = IR_CMD_4; // Show temperature for 5 seconds 
                tmr_xsec = 0;          // reset timer
            } // else if
            else if (key == IR_5) 
            {
                ir_cmd_std = IR_CMD_5; // Set color intensity
                tmr_xsec = 0;          // reset timer
            } // else if
            else if (key == IR_6) 
            {
                ir_cmd_std = IR_CMD_6; // Blanking invert
                tmr_xsec = 0;          // reset timer
            } // else if
            else if (key == IR_7) 
            {
                ir_cmd_std = IR_CMD_7; // Test mode
                tmr_xsec = 0;          // reset timer
            } // else if
            else if (key == IR_8) 
            {
                ir_cmd_std = IR_CMD_8; // Set Blanking-Begin time
                tmr_xsec = 0;          // reset timer
            } // else if
            else if (key == IR_9) 
            {
                ir_cmd_std = IR_CMD_9; // Set Blanking-End time
                tmr_xsec = 0;          // reset timer
            } // else if
            else if (key == IR_HASH) 
            {
                ir_cmd_std = IR_CMD_HASH; // Show date & year
                tmr_xsec = 0;             // reset timer
            } // else if
            break;
            
        case IR_CMD_0:
            break;
            
        case IR_CMD_1:
            if (++tmr_xsec >= 50)
            {
                show_date_IR = IR_SHOW_TIME;
                ir_cmd_std   = IR_CMD_IDLE;
            } // if
            else
            {   // no time-out yet
                time_arr[0]  = DIG_SPACE; // space
                time_arr[1]  = DIG_V;     // V
                x = strlen(ssd_clk_ver);
                time_arr[2]  = (uint8_t)(ssd_clk_ver[x-5] - '0');
                time_arr[3]  = (uint8_t)(ssd_clk_ver[x-3] - '0');
                time_arr[4]  = (uint8_t)(ssd_clk_ver[x-2] - '0');
                time_arr[5]  = DIG_SPACE; // space
                show_date_IR = IR_SHOW_VER;
            } // else
            break;
            
        case IR_CMD_2: // show last response status from ESP8266
            if (++tmr_xsec >= 30)
            {
                show_date_IR = IR_SHOW_TIME;
                ir_cmd_std   = IR_CMD_IDLE;
            } // if
            else
            {   // no time-out yet
             temp = ESP8266_MINUTES - (esp8266_tmr / 60); // minutes left until next update
             t2   = encode_to_bcd4(temp);
             time_arr[0]  = DIG_t; // t
             time_arr[1]  = (uint8_t)((t2 >> 8) & 0x0F); // MSB of minutes left
             time_arr[2]  = (uint8_t)((t2 >> 4) & 0x0F); // middle byte of minutes left
             time_arr[3]  = (uint8_t)(t2 & 0x0F);        // LSB of minutes left 
             time_arr[4]  = DIG_SPACE; // leave empty
             time_arr[5]  = (last_esp8266) ? DIG_1 : DIG_0;
             show_date_IR = IR_SHOW_ESP_STAT;
            } // else
            break;
            
        case IR_CMD_3:            
            break;
            
        case IR_CMD_4: // Show temperature for 5 seconds
            if (++tmr_xsec >= 50)
            {
                show_date_IR = IR_SHOW_TIME;
                ir_cmd_std   = IR_CMD_IDLE;
            } // if
            else
            {   // no time-out yet
                temp = ds3231_gettemp();
                x = encode_to_bcd2(temp >> 2); // overflows if temp > 255 Celsius
                time_arr[0] = (x >> 4) & 0x0F; // MSB of temp integer
                time_arr[1] = x & 0x0F;        // LSB of temp integer
                switch (temp & 0x03)
                {   // MSB and LSB of decimal fraction of temperature
                    case 0 : time_arr[2] = time_arr[3] = DIG_0;        break; // .00 Celsius
                    case 1 : time_arr[2] = DIG_2; time_arr[3] = DIG_5; break; // .25 Celsius
                    case 2 : time_arr[2] = DIG_5; time_arr[3] = DIG_0; break; // .50 Celsius
                    case 3 : time_arr[2] = DIG_7; time_arr[3] = DIG_5; break; // .75 Celsius
                    default: break;
                } // switch
                time_arr[4] = DIG_DEGR; // degree symbol
                time_arr[5] = DIG_C;    // C symbol
                show_date_IR = IR_SHOW_TEMP;
            } // else
            break;
            
        case IR_CMD_5: // Set intensity of colors
            x = encode_to_bcd2(led_intensity_b);
            time_arr[0]  = (x >> 4) & 0x0F;   // MSB of blue intensity
            time_arr[1]  = x & 0x0F;          // LSB of blue intensity
            x = encode_to_bcd2(led_intensity_g);
            time_arr[2]  = (x >> 4) & 0x0F;   // MSB of green intensity
            time_arr[3]  = x & 0x0F;          // LSB of green intensity
            x = encode_to_bcd2(led_intensity_r);
            time_arr[4]  = (x >> 4) & 0x0F;   // MSB of red intensity
            time_arr[5]  = x & 0x0F;          // LSB of green intensity
            time_arr_idx = 0;
            set_color_IR = true;              // indicate change color intensity
            ir_cmd_std   = IR_CMD_COL_CURSOR; // use cursor keys to change time
            break;
            
        case IR_CMD_6: // Invert Blanking Active for 60 seconds
            if (++tmr_xsec >= 600)
            {
                blanking_invert = false;
                ir_cmd_std      = IR_CMD_IDLE;
            } // if
            else blanking_invert = true;
            break;
            
        case IR_CMD_7: // Set test mode for 60 seconds
            if (++tmr_xsec >= 600)
            {
                enable_test_IR = false;
                ir_cmd_std     = IR_CMD_IDLE;
            } // if
            else enable_test_IR = true;
            break;
            
        case IR_CMD_8: // Set Blanking Begin
            x = encode_to_bcd2(blank_begin_h);
            time_arr[0]  = DIG_b;           // b
            time_arr[1]  = DIG_b;           // b
            time_arr[2]  = (x >> 4) & 0x0F; // MSB of hours blanking-begin
            time_arr[3]  = x & 0x0F;        // LSB of hours blanking-begin
            x = encode_to_bcd2(blank_begin_m);
            time_arr[4]  = (x >> 4) & 0x0F; // MSB of minutes blanking-begin
            time_arr[5]  = x & 0x0F;        // LSB of minutes blanking-begin 
            time_arr_idx = 2;               // start at MSB of hours
            set_time_IR  = IR_BB_TIME;      // indicate change blanking-begin time
            ir_cmd_std   = IR_CMD_CURSOR;   // use cursor keys to change time
            break;
            
        case IR_CMD_9: // Set Blanking End
            x = encode_to_bcd2(blank_end_h);
            time_arr[0]  = DIG_b;           // b
            time_arr[1]  = DIG_E;           // E
            time_arr[2]  = (x >> 4) & 0x0F; // MSB of hours blanking-end
            time_arr[3]  = x & 0x0F;        // LSB of hours blanking-end
            x = encode_to_bcd2(blank_end_m);
            time_arr[4]  = (x >> 4) & 0x0F; // MSB of minutes blanking-end
            time_arr[5]  = x & 0x0F;        // LSB of minutes blanking-end
            time_arr_idx = 2;               // start at MSB of hours
            set_time_IR  = IR_BE_TIME;      // indicate change blanking-end time
            ir_cmd_std   = IR_CMD_CURSOR;   // use cursor keys to change time
            break;

        case IR_CMD_HASH: // Show date & year for 8 seconds
            if (++tmr_xsec >= 80)
            {
                show_date_IR = IR_SHOW_TIME;
                ir_cmd_std   = IR_CMD_IDLE;
            } // if
            else if ((tmr_xsec < 20) || ((tmr_xsec >= 40) && (tmr_xsec < 60))) 
                 show_date_IR = IR_SHOW_DATE;
            else show_date_IR = IR_SHOW_YEAR;
            break;
            
        case IR_CMD_CURSOR:  // use cursor keys to change time for blanking-begin & -end
            x = time_arr[time_arr_idx]; // get current digit
            switch (key)
            {
                case IR_0: case IR_1: case IR_2: case IR_3: case IR_4:
                case IR_5: case IR_6: case IR_7: case IR_8: case IR_9:
                    check_possible_digit(key);
                    break;
                case IR_UP: 
                    check_possible_digit(++x); 
                    break;
                case IR_DOWN: 
                    check_possible_digit(--x); 
                    break;
                case IR_LEFT: 
                    if (time_arr_idx == 2) 
                         time_arr_idx = 5;
                    else time_arr_idx--;
                    break;
                case IR_RIGHT: 
                    if (time_arr_idx == 5) 
                         time_arr_idx = 2;
                    else time_arr_idx++;
                     break;
                case IR_OK: 
                    if (set_time_IR == IR_BB_TIME)
                    {  // Blanking-time Begin
                       blank_begin_h = 10 * time_arr[2] + time_arr[3];
                       blank_begin_m = 10 * time_arr[4] + time_arr[5];
                       eeprom_write_config(EEP_ADDR_BBEGIN_H,blank_begin_h);
                       eeprom_write_config(EEP_ADDR_BBEGIN_M,blank_begin_m);
                       set_time_IR = IR_NO_TIME;
                       ir_cmd_std  = IR_CMD_IDLE;
                    } // if
                    else if (set_time_IR == IR_BE_TIME)
                    {  // Blanking-time End
                       blank_end_h = 10 * time_arr[2] + time_arr[3];
                       blank_end_m = 10 * time_arr[4] + time_arr[5];
                       eeprom_write_config(EEP_ADDR_BEND_H,blank_end_h);
                       eeprom_write_config(EEP_ADDR_BEND_M,blank_end_m);
                       set_time_IR = IR_NO_TIME;
                       ir_cmd_std  = IR_CMD_IDLE;
                    } // if
                    break;
                default: break; // ignore all other keys
            } // switch
            break;
            
        case IR_CMD_COL_CURSOR:  // use cursor keys to change color intensity
            led_intensity_b = 10 * time_arr[0] + time_arr[1];
            led_intensity_g = 10 * time_arr[2] + time_arr[3];
            led_intensity_r = 10 * time_arr[4] + time_arr[5];
            x = time_arr[time_arr_idx]; // get current digit
            switch (key)
            {
                case IR_0: case IR_1: case IR_2: case IR_3: case IR_4:
                case IR_5: case IR_6: case IR_7: case IR_8: case IR_9:
                    check_possible_col_digit(key);
                    break;
                case IR_UP: 
                    check_possible_col_digit(++x); 
                    break;
                case IR_DOWN: 
                    check_possible_col_digit(--x); 
                    break;
                case IR_LEFT: 
                    if (time_arr_idx == 0) 
                         time_arr_idx = 5;
                    else time_arr_idx--;
                    break;
                case IR_RIGHT: 
                    if (time_arr_idx == 5) 
                         time_arr_idx = 0;
                    else time_arr_idx++;
                    break;
                case IR_OK: 
                       eeprom_write_config(EEP_ADDR_INTENSITY_B,led_intensity_b);
                       eeprom_write_config(EEP_ADDR_INTENSITY_G,led_intensity_g);
                       eeprom_write_config(EEP_ADDR_INTENSITY_R,led_intensity_r);
                       set_color_IR = false; // leave color intensity change mode
                       ir_cmd_std   = IR_CMD_IDLE;
                    break;
                default: break; // ignore all other keys
            } // switch
            break;
            
        default:
            ir_cmd_std = IR_CMD_IDLE;
            break;
    } // switch   
} // handle_ir_command()

/*-----------------------------------------------------------------------------
  Purpose  : This is the 100 msec. task from the task-scheduler and it controls
             the IR remote controller. If the PORTC IRQ handler signals a new
             IR signal, it sets ir_rdy high. This function then decodes the
             IR signal into a key pressed and resets the PORTC IRQ handler for
             reception of a new IR signal.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void ir_task(void)
{
    uint8_t i, key = IR_NONE;
    
    if (ir_rdy)
    {
       if (ir_decode_nec()) 
       {
           key = ir_key(); // find the key pressed
       } // if
       for (i = 0; i < 99; i++) rawbuf[i] = 0; // clear buffer
       tmr3_std = STATE_IDLE; // reset state for next IR code
       ir_rdy   = false;      // done here
    } // if
    handle_ir_command(key);   // run this every 100 msec.
} // ir_task()

/*-----------------------------------------------------------------------------
  Purpose  : This is external interrupt routine 7 for PORTE
             It is connected to the SQW output of the DS3231, but not used now.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
#pragma vector = EXTI4_vector
__interrupt void PORTE_IRQHandler(void)
{
} // PORTE_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This is interrupt routine 13 for the Timer 2 Overflow handler.
             It runs at 1 kHz and drives the scheduler and the multiplexer.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
#pragma vector = TIM2_OVR_UIF_vector
__interrupt void TIM2_UPD_OVF_IRQHandler(void)
{
    scheduler_isr();  // Run scheduler interrupt function
    t2_millis++;      // update milliseconds timer
    TIM2_SR1_UIF = 0; // Reset the interrupt otherwise it will fire again straight away.
} // TIM2_UPD_OVF_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises the system clock to run at 16 MHz.
             It uses the internal HSI oscillator.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void initialise_system_clock(void)
{
    CLK_ICKR       = 0;           //  Reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1;           //  Enable the HSI.
    CLK_ECKR       = 0;           //  Disable the external clock.
    while (CLK_ICKR_HSIRDY == 0); //  Wait for the HSI to be ready for use.
    CLK_CKDIVR     = 0;           //  Ensure the clocks are running at full speed.
 
    // The datasheet lists that the max. ADC clock is equal to 6 MHz (4 MHz when on 3.3V).
    // Because fMASTER is now at 16 MHz, we need to set the ADC-prescaler to 4.
    ADC_CR1_SPSEL  = 0x02;        //  Set prescaler to 4, fADC = 4 MHz
    CLK_SWIMCCR    = 0;           //  Set SWIM to run at clock / 2.
    CLK_SWR        = 0xe1;        //  Use HSI as the clock source.
    CLK_SWCR       = 0;           //  Reset the clock switch control register.
    CLK_SWCR_SWEN  = 1;           //  Enable switching.
    while (CLK_SWCR_SWBSY != 0);  //  Pause while the clock switch is busy.
} // initialise_system_clock()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises Timer 2 to generate a 1 kHz interrupt.
             16 MHz/(16 * 1000) = 1000 Hz (1000 = 0x03E8), which is the main
             time-base for the scheduler.
             Furthermore, set the interrupt priority to level 1 (medium), so
             that it only can be interrupted by the IR-interrupt (Timer 3).
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_timer2(void)
{
    TIM2_PSCR    = 0x04;    //  Prescaler = 16
    TIM2_ARRH    = 0x03;    //  High byte of 1000
    TIM2_ARRL    = 0xE8;    //  Low  byte of 1000
    
    ITC_SPR4_VECT13SPR = 1; // IRQ 13 (Timer 2 update/overflow) to priority 1
    TIM2_IER_UIE       = 1; //  Enable the update interrupts
    TIM2_CR1_CEN       = 1; //  Finally enable the timer
} // setup_timer2()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises Timer 3 to generate a 31.25 kHz 32 usec.
             counter. 16 MHz/(2^9) = 31.25 kHz. This counter is used for the
             VS1838b IR remote controller.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_timer3(void)
{
    TIM3_PSCR    = 0x09;    //  Prescaler = 512, freq. = 31.25 kHz, T = 32 usec.
    //TIM3_ARRH    = 0x00;    //  High byte of 50
    //TIM3_ARRL    = 0x32;    //  Low  byte of 50
    
    //ITC_SPR4_VECT15SPR = 0; // IRQ 15 (Timer 3 update/overflow) to priority 2
    //TIM3_IER_UIE       = 1; //  Enable the update interrupts
    TIM3_CR1_CEN       = 1; //  Finally enable the timer
} // setup_timer3()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises all the GPIO pins of the STM8 uC.
             See binary_clock.h for a detailed description of all pin-functions.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_output_ports(void)
{
  PC_DDR     |= DI_3V3;     // Set as output
  PC_CR1     |= DI_3V3;     // Set to Push-Pull
  PC_ODR     &= ~(DI_3V3);  // Turn off outputs
  PC_ODR     |=  IR_RCV;
  PC_DDR     &= ~IR_RCV;    // Set as input
  PC_CR1     &= ~IR_RCV;    // Enable pull-up
  PC_CR2     |=  IR_RCV;    // Enable external interrupt
  EXTI_CR1_PCIS = 0x03;     // PORTC external interrupt to rising & falling edge
  ITC_SPR2_VECT5SPR = 0;    // PORTC external interrupt to priority 2 (higher)
  
  PD_DDR     |= TX;         // Set as output
  PD_CR1     |= TX;         // Set to Push-Pull
  PD_ODR     |= TX;         // Set TX high
  PD_DDR     &= ~RX;        // Set UART1-RX as input
  PD_CR1     &= ~RX;        // Set to floating
  
  PE_DDR     &= ~SQW;       // Set as input
  PE_CR1     |=  SQW;       // Enable pull-up
  PE_ODR     |=  (I2C_SCL | I2C_SDA);   // Must be set here, or I2C will not work
  PE_DDR     |=  (I2C_SCL | I2C_SDA);   // Set as outputs
  PE_CR2     &= ~(I2C_SCL | I2C_SDA);   // O: Set speed to 2 MHz, I: disable IRQ
  //PE_CR2     |=  SQW;       // Enable external interrupt
  //EXTI_CR2_PEIS = 0x01;     // PORTE external interrupt to rising edge only
  PE_DDR     |=  IRQ_LED;
  PE_CR1     |=  IRQ_LED;
  PE_ODR     &= ~IRQ_LED;
} // setup_output_ports()

/*-----------------------------------------------------------------------------
  Purpose  : This routine sends one byte to the WS2812B LED-string.
  Variables: bt: the byte to send
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812b_send_byte(uint8_t bt)
{
    uint8_t i,x = 0x80; // Start with MSB first
    
    for (i = 0; i < 8; i++)
    {
        if (bt & x)
        {    // Send a 1   
             ws2812b_send_1;
        } // if
        else 
        {   // Send a 0
            ws2812b_send_0;
        } // else
        x >>= 1; // Next bit
    } // for i
} // ws2812b_send_byte()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initializes the WS2812B LEDs by sending all zeros to it.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812b_init(void)
{
    for (uint16_t i = 0; i < 3*NR_LEDS; i++) ws2812b_send_byte(0x00);
} // ws2812b_init()

/*-----------------------------------------------------------------------------
  Purpose  : This routine clears all WS2812B LEDs.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void clear_all_leds(void)
{
    for (uint8_t i = 0; i < NR_LEDS; i++)
    {
        led_g[i] = led_r[i] = led_b[i] = 0x00;
    } // for i
 } // clear_all_leds()

/*-----------------------------------------------------------------------------
  Purpose  : This routine sends a test pattern to all WS2812B LEDs. It is 
             called by pattern_task() every 100 msec.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void test_pattern(void)
{
    static uint8_t cntr_b = 0, tmr_b = 0;
    uint8_t i;

    if (++tmr_b >= 20)
    {   // change colour every 2 seconds
        tmr_b = 0;
        switch (cntr_b)
        {
            case 0: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_b[i] = led_intensity_b;
                    led_g[i] = led_r[i] = 0x00;
                } // for
                cntr_b = 1; // next colour
                break;
            case 1: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_g[i] = led_intensity_g;
                    led_b[i] = led_r[i] = 0x00;
                } // for
                cntr_b = 2;
                break;
            case 2: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_r[i] = led_intensity_r;
                    led_b[i] = led_g[i] = 0x00;
                } // for
                cntr_b = 0;
                break;
        } // switch
    } // if
} // test_pattern()

/*------------------------------------------------------------------------
  Purpose  : Encode a byte into 2 BCD numbers.
  Variables: x: the byte to encode
  Returns  : the two encoded BCD numbers
  ------------------------------------------------------------------------*/
uint8_t encode_to_bcd2(uint8_t x)
{
    uint8_t temp;
    uint8_t retv = 0;
    
    temp   = x / 10;
    retv  |= (temp & 0x0F);
    retv <<= 4; // SHL 4
    temp   = x - temp * 10;
    retv  |= (temp & 0x0F);
    return retv;
} // encode_to_bcd2()

/*------------------------------------------------------------------------
  Purpose  : Encode a 16-bit integer into 4 BCD numbers.
  Variables: x: the integer to encode
  Returns  : the four encoded BCD numbers
  ------------------------------------------------------------------------*/
uint16_t encode_to_bcd4(uint16_t x)
{
    uint16_t temp, rest = x;
    uint16_t retv = 0;
    
    temp   = rest / 1000;
    retv  |= (temp & 0x0F);
    retv <<= 4; // SHL 4
    rest  -= temp * 1000;

    temp   = rest / 100;
    retv  |= (temp & 0x0F);
    retv <<= 4; // SHL 4
    rest  -= temp * 100;
    
    temp   = rest / 10;
    retv  |= (temp & 0x0F);
    retv <<= 4; // SHL 4
    rest  -= temp * 10;
    retv  |= (rest & 0x0F);
    return retv;
} // encode_to_bcd4()

/*-----------------------------------------------------------------------------
  Purpose  : This function fills one color of a 7-segment display with a digit 
             and with an intensity. The decimal-point can also be set.
  Variables: 
             board_nr: [0,NR_BOARDS-1]
             *p      : pointer to led_r[], led_g[] or led_b[] array
             digit   : digit to write into array 
             dp      : true = enable decimal-point 
  Returns  : -
  ---------------------------------------------------------------------------*/
void fill_led_color(uint8_t *p, uint8_t board_nr, uint8_t digit, uint8_t intensity, bool dp)
{
    uint8_t lednr = board_nr * NR_LEDS_PER_BOARD;
    
    if ((board_nr >= NR_BOARDS) || (digit >= sizeof(ssd))) return; // error
    
    // LED chain-order is segment E, D, C, G, B, A, F, dp
    p[lednr   ] = p[lednr+ 1] = p[lednr+ 2] = p[lednr+ 3] = (ssd[digit] & SEG_E) ? intensity : 0x00;
    p[lednr+ 4] = p[lednr+ 5] = p[lednr+ 6] = p[lednr+ 7] = (ssd[digit] & SEG_D) ? intensity : 0x00;
    p[lednr+ 8] = p[lednr+ 9] = p[lednr+10] = p[lednr+11] = (ssd[digit] & SEG_C) ? intensity : 0x00;
    p[lednr+12] = p[lednr+13] = p[lednr+14] = p[lednr+15] = (ssd[digit] & SEG_G) ? intensity : 0x00;
    p[lednr+16] = p[lednr+17] = p[lednr+18] = p[lednr+19] = (ssd[digit] & SEG_B) ? intensity : 0x00;
    p[lednr+20] = p[lednr+21] = p[lednr+22] = p[lednr+23] = (ssd[digit] & SEG_A) ? intensity : 0x00;
    p[lednr+24] = p[lednr+25] = p[lednr+26] = p[lednr+27] = (ssd[digit] & SEG_F) ? intensity : 0x00;
    p[lednr+28] = (dp ? intensity : 0x00); // decimal-point
} // fill_led_color()

/*-----------------------------------------------------------------------------
  Purpose  : This routine fills one 7-segment display with a digit in a 
             particular color and intensity. The decimal-point can also be set.
  Variables: board_nr: [0,NR_BOARDS-1]
             color   : set of defined colors
             digit   : digit to write into array 
             dp      : true = enable decimal-point
  Returns  : -
  ---------------------------------------------------------------------------*/
void fill_led_array(uint8_t board_nr, uint8_t color, uint8_t digit, bool dp)
{
    switch (color)
    {
    case COL_RED:
        fill_led_color(led_r, board_nr, digit, led_intensity_r, dp);
        break;
    case COL_GREEN:
        fill_led_color(led_g, board_nr, digit, led_intensity_g, dp);
        break;
    case COL_BLUE:
        fill_led_color(led_b, board_nr, digit, led_intensity_b, dp);
        break;
    case COL_YELLOW:
        fill_led_color(led_r, board_nr, digit, led_intensity_r>>1, dp);
        fill_led_color(led_g, board_nr, digit, led_intensity_g>>1, dp);
        break;
    case COL_MAGENTA:
        fill_led_color(led_r, board_nr, digit, led_intensity_r>>1, dp);
        fill_led_color(led_b, board_nr, digit, led_intensity_b>>1, dp);
        break;
    case COL_CYAN:
        fill_led_color(led_g, board_nr, digit, led_intensity_g>>1, dp);
        fill_led_color(led_b, board_nr, digit, led_intensity_b>>1, dp);
        break;
    default: // COL_WHITE:
        fill_led_color(led_r, board_nr, digit, led_intensity_r>>1, dp);
        fill_led_color(led_g, board_nr, digit, led_intensity_g>>1, dp);
        fill_led_color(led_b, board_nr, digit, led_intensity_b>>1, dp);
        break;
    } // switch
} // fill_led_array()

/*-----------------------------------------------------------------------------
  Purpose  : This routine creates a pattern for the LEDs and stores it in
             the arrays led_r, led_g and led_b
             It uses the global variables seconds, minutes and hours and is
             called every 100 msec. by the scheduler.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void pattern_task(void)
{
    uint8_t  x,xl,xm,cl,cm;
    uint16_t y;
    bool     dpm = false;
    bool     dpl = false;
    static bool    blink     = false;
    static uint8_t col_white_tmr = 0;
    
    if (!watchdog_test)   
    {   // only refresh when watchdog_test == 0 (X0 command)
        IWDG_KR = IWDG_KR_KEY_REFRESH; // Refresh watchdog (reset after 500 msec.)
    } // if
    if (enable_test_pattern || enable_test_IR)
    {   // WS2812 test-pattern
	test_pattern(); 
    } // if
    else
    {
        if ((blanking_active() || powerup) && (show_date_IR == IR_SHOW_TIME) && 
            (set_time_IR == IR_NO_TIME) && !set_color_IR) 
        {  // blanking leds only on power-up and no IR-commands active
            clear_all_leds();
            return;
        } // if
        // check summertime change every minute
        if (dt.sec == 0) check_and_set_summertime(); 
        
        clear_all_leds(); // Start with clearing all leds
        blink = !blink;   // toggle blinking

        //-------------------------------------
        // Fill SSD 0 and 1 (left SSDs)
        //-------------------------------------
        if (show_date_IR == IR_SHOW_DATE)
        {   // show day and month
            x  = encode_to_bcd2(dt.day);
            xm = (x >> 4) & 0x0F; // msb day
            xl = x & 0x0F;        // lsb day
            cl = cm = COL_YELLOW;
        } // else if
        else if (show_date_IR == IR_SHOW_YEAR)
        {   // show year
            y  = encode_to_bcd4(dt.year);
            xm = DIG_SPACE;                   // SSD off
            xl = (uint8_t)((y >> 12) & 0x0F); // msb year
            cl = cm = COL_YELLOW;
        } // else
        else if (show_date_IR == IR_SHOW_TEMP)
        {   // show temperature of DS3231
            xm  = time_arr[0]; // msb of temperature decimal part
            xl  = time_arr[1]; // lsb of temperature decimal part
            cl  = cm = COL_CYAN;
            dpl = true; // set decimal point
        } // else if
        else if (show_date_IR == IR_SHOW_ESP_STAT)
        {   // show status of last ESP8266 response
            xm = time_arr[0]; // esp01 text
            xl = time_arr[1]; // esp01 text
            cl = cm = COL_YELLOW;
        } // else if
        else if (show_date_IR == IR_SHOW_VER)
        {   // show version info
            xm = time_arr[0]; // V0.xx text
            xl = time_arr[1]; // V0.xx text
            cl = cm = COL_YELLOW;
        } // else if
        else if (set_time_IR != IR_NO_TIME)
        {   // show blanking-begin or -end time
            xm = time_arr[0]; // bb/bE text
            xl = time_arr[1]; // bb/bE text
            cl = cm = COL_YELLOW;
        } // else if
        else if (set_color_IR)
        {   // change color-intensity
            xm = time_arr[0]; // msb of blue color intensity
            xl = time_arr[1]; // lsb of blue color intensity
            cl = cm = COL_BLUE;
            if (blink)
            {  // blink decimal points if digits are active 
               if (time_arr_idx == 0)      dpm = true;
               else if (time_arr_idx == 1) dpl = true;
            } // if
        } // else if
        else
        {   // normal time mode (show_date_IR == IR_SHOW_TIME)
            x  = encode_to_bcd2(dt.hour);
            xm = (x >> 4) & 0x0F; // msb hours
            xl = x & 0x0F;        // lsb hours
            if (set_col_white)
                 cl = cm = COL_WHITE;
            else cl = cm = COL_BLUE;
        } // if
        fill_led_array(0, cm, xm, dpm); // MSB
        fill_led_array(1, cl, xl, dpl); // LSB

        //-------------------------------------
        // Fill SSD 2 and 3 (middle SSDs)
        //-------------------------------------
        if (show_date_IR == IR_SHOW_DATE)
        {   // show day and month
            x  = encode_to_bcd2(dt.mon);
            xm = DIG_MINUS;       // - (seg G)
            xl = (x >> 4) & 0x0F; // msb month
            cl = cm = COL_YELLOW;
        } // else if
        else if (show_date_IR == IR_SHOW_YEAR)
        {   // show year
            xm = (uint8_t)((y >> 8) & 0x0F); // year, 3rd digit from right
            xl = (uint8_t)((y >> 4) & 0x0F); // year, 2nd digit from right
            cl = cm = COL_YELLOW;
        } // else if
        else if (show_date_IR == IR_SHOW_TEMP)
        {   // show temperature of DS3231
            xm  = time_arr[2]; // msb of temperature fraction
            xl  = time_arr[3]; // lsb of temperature fraction
            cl  = cm = COL_CYAN;
            dpl = dpm = false;
        } // else if
        else if (show_date_IR == IR_SHOW_ESP_STAT)
        {   // show status of last ESP8266 response
            xm = time_arr[2]; // ESP01 text
            xl = time_arr[3]; // ESP01 text
            cl = cm = COL_YELLOW;
        } // else if
        else if (show_date_IR == IR_SHOW_VER)
        {   // show version info
            xm  = time_arr[2]; // version info
            xl  = time_arr[3]; // version info
            cl  = cm = COL_MAGENTA;
            dpm = true;
        } // else if
        else if (set_time_IR != IR_NO_TIME)
        {   // show blanking-begin or -end time
            xm = time_arr[2]; // msb of hours
            xl = time_arr[3]; // lsb of hours
            cl = cm = COL_MAGENTA; // default color
            if (blink)
            {   // blinking color
                if      (time_arr_idx == 0) cm = COL_WHITE - COL_MAGENTA;
                else if (time_arr_idx == 1) cl = COL_WHITE - COL_MAGENTA;
            } // if
        } // else if
        else if (set_color_IR)
        {   // change color-intensity
            xm  = time_arr[2]; // msb of green color intensity
            xl  = time_arr[3]; // lsb of green color intensity
            cl  = cm  = COL_GREEN;
            dpl = dpm = false;
            if (blink)
            {  // blink decimal points if digits are active 
               if (time_arr_idx == 2)      dpm = true;
               else if (time_arr_idx == 3) dpl = true;
            } // if
        } // else if
        else
        {   // normal time mode (show_date_IR == IR_SHOW_TIME)
            x  = encode_to_bcd2(dt.min);
            xm = (x >> 4) & 0x0F; // msb minutes
            xl = x & 0x0F;        // lsb minutes
            if (set_col_white)
                 cl = cm = COL_WHITE;
            else cl = cm = COL_GREEN;
        } // if
        fill_led_array(2, cm, xm, dpm); // MSB
        fill_led_array(3, cl, xl, dpl); // LSB
        
        //-------------------------------------
        // Fill SSD 4 and 5 (right SSDs)
        //-------------------------------------
        if (show_date_IR == IR_SHOW_DATE)
        {   // show day and month
            xm = encode_to_bcd2(dt.mon) & 0x0F; // lsb month
            xl = DIG_SPACE;                     // SSD off
            cl = cm = COL_YELLOW;
        } // else if
        else if (show_date_IR == IR_SHOW_YEAR)
        {   // show year
            xm = (uint8_t)(y & 0x000F); // lsb year
            xl = DIG_SPACE;             // SSD off
            cl = cm = COL_YELLOW;
        } // else if
        else if (show_date_IR == IR_SHOW_TEMP)
        {   // show temperature of DS3231
            xm = time_arr[4]; // degree symbol
            xl = time_arr[5]; // C symbol
            cl = cm = COL_YELLOW;
        } // else if
        else if (show_date_IR == IR_SHOW_ESP_STAT)
        {   // show status of last ESP8266 response
            xm  = time_arr[4]; // ESP01 msb
            xl  = time_arr[5]; // ESP01 lsb
            cm  = COL_YELLOW;
            if (xl == 1) 
                 cl = COL_GREEN; // last response ok
            else cl = COL_RED;   // last response not ok
        } // else if
        else if (show_date_IR == IR_SHOW_VER)
        {   // show version info
            xm  = time_arr[4]; // version info
            xl  = time_arr[5]; // version info
            cl  = cm  = COL_MAGENTA;
            dpm = dpl = false;
        } // else if
        else if (set_time_IR != IR_NO_TIME)
        {   // show blanking-begin or end time
            xm = time_arr[4]; // msb of minutes
            xl = time_arr[5]; // lsb of minutes
            cl = cm = COL_MAGENTA; // default color
            if (blink)
            {   // blinking color
                if      (time_arr_idx == 2) cm = COL_WHITE - COL_MAGENTA;
                else if (time_arr_idx == 3) cl = COL_WHITE - COL_MAGENTA;
            } // if
        } // else if
        else if (set_color_IR)
        {   // set color-intensity
            xm  = time_arr[4]; // msb of red color intensity
            xl  = time_arr[5]; // lsb of red color intensity
            cl  = cm  = COL_RED;
            dpl = dpm = false;
            if (blink)
            {  // blink decimal points if digits are active 
               if (time_arr_idx == 4)      dpm = true;
               else if (time_arr_idx == 5) dpl = true;
            } // if
        } // else if
        else
        {   // normal time mode (show_date_IR == IR_SHOW_TIME)
            x  = encode_to_bcd2(dt.sec);
            xm = (x >> 4) & 0x0F; // msb seconds
            xl = x & 0x0F;        // lsb seconds
            if (set_col_white)
            {
                cl = cm = COL_WHITE;
                if (++col_white_tmr > 10)
                {
                    col_white_tmr = 0;
                    set_col_white = false;
                } // if
            } // else
            else cl = cm = COL_RED;
        } // if
        fill_led_array(4, cm, xm, dpm); // MSB
        fill_led_array(5, cl, xl, dpl); // LSB
    } // else
} // pattern_task()    
        
/*-----------------------------------------------------------------------------
  Purpose  : This routine sends the RGB-bytes for every LED to the WS2812B
             LED string. It is called every 100 msec. by the scheduler.
  Variables: 
      led_g: the (global) green byte array
      led_r: the (global) red byte array
      led_b: the (global) blue byte array
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812_task(void)
{
    __disable_interrupt();   // disable IRQ for time-sensitive LED-timing
    for (uint8_t i = 0; i < NR_LEDS; i++)
    {
        ws2812b_send_byte(led_g[i]); // Send one byte of Green
        ws2812b_send_byte(led_r[i]); // Send one byte of Red
        ws2812b_send_byte(led_b[i]); // Send one byte of Blue
    } // for i
    __enable_interrupt(); // enable IRQ again
} // ws2812_task()

/*------------------------------------------------------------------------
Purpose  : This task is called every minute by pattern_task(). It checks 
           for a change from summer- to wintertime and vice-versa.
           To start DST: Find the last Sunday in March  : @2 AM advance clock to 3 AM.
           To stop DST : Find the last Sunday in October: @3 AM set clock back to 2 AM (only once!).
Variables: p: pointer to time-struct
Returns  : -
------------------------------------------------------------------------*/
void check_and_set_summertime(void)
{
    uint8_t        hr,day,lsun03,lsun10,dst_eep;
    static uint8_t advance_time = 0;
    static uint8_t revert_time  = 0;
    char           s[20];
    
    if (dt.mon == 3)
    {
        day    = ds3231_calc_dow(31,3,dt.year); // Find day-of-week for March 31th
        lsun03 = 31 - (day % 7);                // Find last Sunday in March
        sprintf(s,"lsun03=%d\n",lsun03); 
        uart_printf(s);
        switch (advance_time)
        {
        case 0: if ((dt.day == lsun03) && (dt.hour == 2) && (dt.min == 0))
                {   // At 2:00 AM advance time to 3 AM, check for one minute
                    advance_time = 1;
                } // if
                else if (dt.day < lsun03) dst_active = false;
                else if (dt.day > lsun03) dst_active = true;
                else if (dt.hour < 2)     dst_active = false;
                break;
        case 1: // Now advance time, do this only once
                ds3231_settime(3,0,dt.sec); // Set time to 3:00, leave secs the same
                advance_time = 2;
                dst_active   = true;
                eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x01); // set DST in eeprom
                break;
        case 2: 
                if (dt.min > 0) advance_time = 0; // At 3:01:00 back to normal
                dst_active = true;
        break;
        } // switch
    } // if
    else if (dt.mon == 10)
    {
        day    = ds3231_calc_dow(31,10,dt.year); // Find day-of-week for October 31th
        lsun10 = 31 - (day % 7);                 // Find last Sunday in October
        sprintf(s,"lsun10=%d\n",lsun10); 
        uart_printf(s);
        switch (revert_time)
        {
            case 0: if ((dt.day == lsun10) && (dt.hour == 3) && (dt.min == 0))
                    {   // At 3:00 AM revert time back to 2 AM, check for one minute
                        revert_time = 1;
                    } // if
                    else if (dt.day > lsun10) dst_active = false;
                    else if (dt.day < lsun10) dst_active = true;
                    else if (dt.hour < 3)     dst_active = true;
                    break;
            case 1: // Now revert time, do this only once
                    ds3231_settime(2,0,dt.sec); // Set time back to 2:00, leave secs the same
                    revert_time = 2;
                    dst_active  = false;
                    eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x00); // reset DST in eeprom
                    break;
            case 2: // make sure we passed 3 AM in order to prevent multiple reverts
                    if (dt.hour > 3) revert_time = 0; // at 4:00:00 back to normal
                    dst_active = false;
                    break;
        } // switch
    } // else if
    else if ((dt.mon < 3) || (dt.mon > 10)) dst_active = false;
    else                                    dst_active = true;

    //------------------------------------------------------------------------
    // If, for some reason, the clock was powered-off during the change to
    // summer- or winter-time, the eeprom value differs from the actual 
    // dst_active value. If so, set the actual sommer- and winter-time.
    //------------------------------------------------------------------------
    dst_eep = (uint8_t)eeprom_read_config(EEP_ADDR_DST_ACTIVE);
    if (dst_active && !dst_eep)
    {   // It is summer-time, but clock has not been advanced yet
        hr = (dt.hour >= 23) ? 0 : dt.hour + 1;
        ds3231_settime(hr,dt.min,dt.sec); // Set summer-time to 1 hour later
        eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x01); // set DST in eeprom
    } // if
    else if (!dst_active && dst_eep)
    {   // It is winter-time, but clock has not been moved back yet
        hr = (dt.hour > 0) ? dt.hour - 1 : 23;
        ds3231_settime(hr,dt.min,dt.sec); // Set summer-time to 1 hour earlier
        eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x00); // set DST in eeprom
    } // if
} // check_and_set_summertime()

/*-----------------------------------------------------------------------------
  Purpose  : This routine reads the date and time info from the DS3231 RTC and
             stores this info into the global variables seconds, minutes and
             hours. It is called every second.
  Variables: dt: global struct that stores time & date variables
  Returns  : -
  ---------------------------------------------------------------------------*/
void clock_task(void)
{
    ds3231_gettime(&dt); // Get time from DS3231 RTC
    powerup = false;     // Time received, so reset power-up flag
    switch (esp8266_std)
    {
    case ESP8266_INIT:
        if (++esp8266_tmr >= ESP8266_SECONDS) // 12 hours * 60 min. * 60 sec.
           esp8266_std = ESP8266_UPDATE;
        break;
    case ESP8266_UPDATE:
        last_esp8266 = false; // reset status
        esp8266_tmr  = 0;
        esp8266_std  = ESP8266_INIT;
        uart_printf("e0\n");  // update time from ESP8266
        break;
    default: 
        esp8266_tmr = 0;
        esp8266_std = ESP8266_INIT;
        break;
    } // switch
} // clock_task()

/*-----------------------------------------------------------------------------
  Purpose  : This routine prints the day-of-week to the uart
  Variables: 
        dow: [0..7], 1=Monday, 7 = Sunday
  Returns  : -
  ---------------------------------------------------------------------------*/
void print_dow(uint8_t dow)
{
    char day[8][4] = {"???","Mon","Tue","Wed","Thu","Fri","Sat","Sun"};

    uart_printf(day[dow]);
} // print_dow()

/*-----------------------------------------------------------------------------
  Purpose  : This routine reads the time and date from the DS3231 RTC and 
             prints this info to the uart.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void print_date_and_time(void)
{
    char s2[40]; // Used for printing to UART

    check_and_set_summertime();
    sprintf(s2," %d-%d-%d, %d:%d.%d",
               dt.day , dt.mon, dt.year,
               dt.hour, dt.min, dt.sec);
    uart_printf(s2);
    sprintf(s2," dow:%d, dst:%d, blanking:%d\n",
               dt.dow, dst_active, blanking_active());
    uart_printf(s2);
} // print_date_and_time()

/*------------------------------------------------------------------------
  Purpose  : This function converts hours and minutes to minutes.
  Variables: h  : hours of actual time
             min: minutes of actual time
  Returns  : time in minutes
  ------------------------------------------------------------------------*/
uint16_t cmin(uint8_t h, uint8_t m)
{
    return (uint16_t)h * 60 + m;
} // cmin()

/*------------------------------------------------------------------------
  Purpose  : This function decides if the current time falls between the
             blanking time for the LEDs.
  Variables: -
  Returns  : true: blanking is true, false: no blanking
  ------------------------------------------------------------------------*/
bool blanking_active(void)
{
	uint16_t x = cmin(dt.hour      , dt.min);
	uint16_t b = cmin(blank_begin_h, blank_begin_m);
	uint16_t e = cmin(blank_end_h  , blank_end_m);
	
	// (b>=e): Example: 23:30 and 05:30, active if x>=b OR  x<=e
	// (b< e): Example: 02:30 and 05:30, active if x>=b AND x<=e
	bool blanking = (b >= e) && ((x >= b) || (x <= e)) || ((x >= b) && (x < e)); 
        if (blanking_invert) blanking = !blanking;
        return blanking;
} // blanking_active()

/*-----------------------------------------------------------------------------
  Purpose: interpret commands which are received via the USB serial terminal:
  Variables: 
          s: the string that contains the command from RS232 serial port 0
  Returns  : -
  ---------------------------------------------------------------------------*/
void execute_single_command(char *s)
{
   uint8_t  num  = atoi(&s[1]); // convert number in command (until space is found)
   char     s2[40]; // Used for printing to RS232 port
   char     *s1;
   uint8_t  d,m,h,sec;
   uint16_t i,y;
   int16_t  temp;
   const char sep[] = ":-.";
   
   switch (s[0])
   {
        case 'd': // Set Date, 1 = Get Date
		 switch (num)
		 {
                    case 0: // Set Date
			    s1 = strtok(&s[3],sep);
                            d  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            m  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            y  = atoi(s1);
                            uart_printf("Date: ");
                            print_dow(ds3231_calc_dow(d,m,y));
                            sprintf(s2," %d-%d-%d\n",d,m,y);
                            uart_printf(s2);
                            ds3231_setdate(d,m,y); // write to DS3231 IC
                            break;
                    case 1: // Set Time
                            s1      = strtok(&s[3],sep);
                            h       = atoi(s1);
                            s1      = strtok(NULL ,sep);
                            m       = atoi(s1);
                            s1      = strtok(NULL ,sep);
                            sec     = atoi(s1);
                            sprintf(s2,"Time: %d:%d:%d\n",h,m,sec);
                            uart_printf(s2);
                            ds3231_settime(h,m,sec); // write to DS3231 IC
                            break;
                    case 2: // Get Date & Time
                            print_date_and_time(); 
                            sprintf(s2,"Blanking: %d:%d - %d:%d\n",
                                       blank_begin_h, blank_begin_m,
                                       blank_end_h  , blank_end_m);
                            uart_printf(s2);
                            break;
                    case 3: // Get Temperature
                            temp = ds3231_gettemp();
                            sprintf(s2,"DS3231: %d.",temp>>2);
                            uart_printf(s2);
                            switch (temp & 0x03)
                            {
				case 0: uart_printf("00 C\n"); break;
				case 1: uart_printf("25 C\n"); break;
				case 2: uart_printf("50 C\n"); break;
				case 3: uart_printf("75 C\n"); break;
                            } // switch
                            break;
                    case 4: // Set Start-Time for blanking Nixies
                            s1 = strtok(&s[3],sep);
                            h  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            m  = atoi(s1);
                            if ((h < 24) && (m < 60))
                            {
                                blank_begin_h = h;
                                blank_begin_m = m;
                                eeprom_write_config(EEP_ADDR_BBEGIN_H,blank_begin_h);
                                eeprom_write_config(EEP_ADDR_BBEGIN_M,blank_begin_m);
                            } // if
                            break;
                    case 5: // Set End-Time for blanking Nixies
                            s1 = strtok(&s[3],sep);
                            h  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            m  = atoi(s1);
                            if ((h < 24) && (m < 60))
                            {
                                blank_end_h = h;
                                blank_end_m = m;
                                eeprom_write_config(EEP_ADDR_BEND_H,blank_end_h);
                                eeprom_write_config(EEP_ADDR_BEND_M,blank_end_m);
                            } // if
                            break;		 
                   default: break;
                 } // switch
                 break;
  
        case 'e': // The e commands are responses back from the ESP8266 NTP Server
                  // Possible response: "e0 26-05-2021.15:55:23"
		 switch (num)
		 {
                 case 0: // E0 = Get Date & Time from the ESP8266
                    s1 = strtok(&s[3],sep);
                    d  = atoi(s1);
                    s1 = strtok(NULL ,sep);
                    m  = atoi(s1);
                    s1 = strtok(NULL ,sep);
                    y  = atoi(s1);
                    // Second part is the time from the ESP8266
                    s1 = strtok(NULL,sep);
                    h  = atoi(s1);
                    if (dst_active)
                    {
                        if (h == 23) 
                             h = 0;
                        else h++;
                    } // if
                    s1  = strtok(NULL ,sep);
                    m   = atoi(s1);
                    s1  = strtok(NULL ,sep);
                    sec = atoi(s1);
                    if (sec == 59) // add 1 second for the transmit delay
                         sec = 0;
                    else sec++;
                    if (y > 2020)
                    {   // Valid Date & Time received
                        ds3231_setdate(d,m,y);   // write to DS3231 IC
                        ds3231_settime(h,m,sec); // write to DS3231 IC
                        last_esp8266 = true;     // response was successful
                        set_col_white = true;    // show briefly on display
                        esp8266_tmr = 0;         // Reset update timer
                    } // if
                    else last_esp8266 = false;   // response not successful
                    uart_printf("Date: ");
                    print_dow(ds3231_calc_dow(d,m,y));
                    sprintf(s2," %d-%d-%d ",d,m,y);
                    uart_printf(s2);
                    sprintf(s2,"Time: %d:%d:%d\n",h,m,sec);
                    uart_printf(s2);
                    break;
                 } // switch
                 break;
                        
	case 'i': // "ix y": set intensity of WS2812 LEDs between 1..39
                  temp = atoi(&s[3]);
                  // x=0: Intensity of Red Leds
                  // x=1: Intensity of Green Leds
                  // x=2: Intensity of Blue Leds
                  if ((temp > 0) && (temp < 40))
                  {
                     switch (num)
                     {
                         case 0: // Red
                             led_intensity_r = temp;
                             eeprom_write_config(EEP_ADDR_INTENSITY_R,led_intensity_r);
                             uart_printf("Ired=");
                             break;
                         case 1: // Green
                             led_intensity_g = temp;
                             eeprom_write_config(EEP_ADDR_INTENSITY_G,led_intensity_g);
                             uart_printf("Igreen=");
                             break;
                         case 2: // Blue
                             led_intensity_b = temp;
                             eeprom_write_config(EEP_ADDR_INTENSITY_B,led_intensity_b);
                             uart_printf("Iblue=");
                             break;
                         default:
                             break;
                     } // switch
                     sprintf(s2,"%d\n",temp);
                     uart_printf(s2);
                  } // if
                  else uart_printf("nr error\n");
		 break;

	case 's': // System commands
		 switch (num)
		 {
                    case 0: // revision
                            uart_printf(ssd_clk_ver);
                            break;
                    case 1: // List all tasks
                            list_all_tasks(); 
                            break;
                    case 2: // I2C-scan
			    uart_printf("I2C-scan: ");
			    for (i = 0x02; i < 0xff; i+=2)
			    {
				if (i2c_start_bb(i) == I2C_ACK)
				{
					sprintf(s2,"0x%x, ",i);
		  		    	uart_printf(s2);
				} // if
				i2c_stop_bb();
			    } // for
			    uart_putc('\n');
                            break;
                   default: break;
                 } // switch
		 break;
                                        
	case 'w': // WS2812 test-pattern command
		 enable_test_pattern = (num > 0); // 1 = enable test-pattern
                 if (!num)
                 {  // clear all leds when finished with test-pattern
                    clear_all_leds();
                 } // if
		 break;

        default: break;
   } // switch
} // execute_single_command()

/*-----------------------------------------------------------------------------
  Purpose  : Non-blocking RS232 command-handler via the USB port
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void rs232_command_handler(void)
{
  char    ch;
  static uint8_t cmd_rcvd = 0;
  
  if (!cmd_rcvd && uart_kbhit())
  { // A new character has been received
    ch = tolower(uart_getc()); // get character as lowercase
    switch (ch)
	{
            case '\n': break;
            case '\r': cmd_rcvd  = 1;
		       rs232_inbuf[rs232_ptr] = '\0';
		       rs232_ptr = 0;
                       uart_putc('\n');
                       break;
            default  : if (rs232_ptr < UART_BUFLEN)
                       {   
                           rs232_inbuf[rs232_ptr++] = ch;
                           uart_putc(ch);
                       } // if
                       else rs232_ptr = 0;
                       break;
	} // switch
  } // if
  if (cmd_rcvd)
  {
	  cmd_rcvd = 0;
	  execute_single_command(rs232_inbuf);
  } // if
} // rs232_command_handler()

/*-----------------------------------------------------------------------------
  Purpose  : This functions initializes the independent watchdog (IWDG) and 
             sets the watchdog timeout to the maximum of T = 512 msec.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void init_watchdog(void)
{
	IWDG_KR  = IWDG_KR_KEY_ENABLE;  // start the IWDG
	IWDG_KR  = IWDG_KR_KEY_ACCESS;  // enable access to IWDG_PR and IWDG_RLR registers
	IWDG_PR  = 0x05;                // prescaler divider 128
	IWDG_RLR = 0xFF;	        // Reload register to maximum
	IWDG_KR  = IWDG_KR_KEY_REFRESH; // reset the IWDG
} // init_watchdog()

/*-----------------------------------------------------------------------------
  Purpose  : This is the main entry-point for the program
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
int main(void)
{
    uint8_t i2c_err;
	
    __disable_interrupt();
    initialise_system_clock(); // Set system-clock to 16 MHz
    setup_output_ports();      // Init. needed output-ports for LED and keys
    setup_timer2();            // Set Timer 2 to 1 kHz for scheduler
    setup_timer3();            // Set Timer 3 to 20 kHz for IRDA
    uart_init();               // Init. UART-peripheral
    ws2812b_init();            // Init. the WS2812B LEDs
    
    led_intensity_r = (uint8_t)eeprom_read_config(EEP_ADDR_INTENSITY_R);
    led_intensity_g = (uint8_t)eeprom_read_config(EEP_ADDR_INTENSITY_G);
    led_intensity_b = (uint8_t)eeprom_read_config(EEP_ADDR_INTENSITY_B);
    if (!led_intensity_r)
    {   // First time power-up: eeprom value is 0x00
        led_intensity_r = LED_INTENSITY;
    } // if
    if (!led_intensity_g)
    {   // First time power-up: eeprom value is 0x00
        led_intensity_g = LED_INTENSITY;
    } // if
    if (!led_intensity_b)
    {   // First time power-up: eeprom value is 0x00
        led_intensity_b = LED_INTENSITY;
    } // if
    blank_begin_h = (uint8_t)eeprom_read_config(EEP_ADDR_BBEGIN_H);
    blank_begin_m = (uint8_t)eeprom_read_config(EEP_ADDR_BBEGIN_M);
    blank_end_h   = (uint8_t)eeprom_read_config(EEP_ADDR_BEND_H);
    blank_end_m   = (uint8_t)eeprom_read_config(EEP_ADDR_BEND_M);
    
    // Initialise all tasks for the scheduler
    scheduler_init();                          // clear task_list struct
    add_task(pattern_task, "PTRN"  ,100, 100); // every 100 msec.
    add_task(ws2812_task , "WS2812",125, 500); // every 500 msec.
    add_task(ir_task     , "IR"    ,150, 100); // every 100 msec.
    add_task(clock_task  , "CLK"   , 75,1000); // every second
    init_watchdog();                           // init. the IWDG watchdog
    __enable_interrupt();

    i2c_err = i2c_reset_bus(); // Init. I2C-peripheral
    uart_printf(ssd_clk_ver);  // Print welcome message
    uart_printf("i2c_reset_bus:");
    if (i2c_err) uart_printf("err\n");
    else         uart_printf("ok\n");
    
    if (ds3231_write_register(REG_CTRL,0x00)) // Enable 1 Hz SQW output
    {   //1 = error, DS3231 RTC not found
        uart_printf("DS3231 not found\n");
        dt.hour = dt.min = dt.sec = 0; // init. 00:00:00 if no RTC present
        dt.day  = dt.mon = 1;          // init. to 01-01-2021 if no RTC present
        dt.year = 2021;
        dt.dow  = FRIDAY;
    } // if
    else ds3231_gettime(&dt); // Read time from DS3231 RTC
    print_date_and_time();    // and output to UART

    while (1)
    {   // background-processes
        dispatch_tasks();        // Run task-scheduler()
        rs232_command_handler(); // run command handler continuously
    } // while
} // main()
