#ifndef CLOCK_SSD_H
#define CLOCK_SSD_H
/*-----------------------------------------------------------------------------------------------
  Schematic of the connections to the STM8S105C6T6 MCU.
  Pin count: 48, max. nr. of GPIO: 38
  Flash: 32K, EEPROM: 1024 bytes, RAM: 2K
  
                                 STM8S105C6T6
      MCU pin-name            Function    |    MCU pin-name        Function
   ---------------------------------------|--------------------------------
   01 NRST                                | 48 PD7/TLI[TIM1_CH4]   -
   02 PA1/OSC                 -           | 47 PD6/UART2_RX        RX
   03 PA2/OSCOUT              -           | 46 PD5/UART2_TX        TX
   04 VSSIO_1                             | 45 PD4/TIM2_CH1[BEEP]  -
   05 VSS                                 | 44 PD3/TIM2_CH2...     -
   06 VCAP                                | 43 PD2/TIM3_CH1...     - 
   07 VDD                                 | 42 PD1/SWIM            SWIM
   08 VDDIO_1                             | 41 PD0/TIM3_CH2...     -
   09 PA3/TIM2_CH3[TIME3_CH1]             | 40 PE0/CLK_CC0         -
   10 PA4                                 | 39 PE1/I2C_SCL         I2C_SCL
   11 PA5                                 | 38 PE2/I2C_SDA         I2C_SDA
   12 PA6                                 | 37 PE3/TIM1_BKIN       -
   ---------------------------------------|--------------------------------
   13 VDDA                                | 36 PG1                     -  
   14 VSSA                                | 35 PG0                     - 
   15 PB7/AIN7            -               | 34 PC7/SPI_MISO            -
   16 PB6/AIN6            -               | 33 PC6/SPI_MOSI            - 
   17 PB5/AIN5[I2C_SDA]   -               | 32 VDDIO_2  
   18 PB4/AIN4[I2C_SCL]   -               | 31 VSSIO_2   
   19 PB3/AIN3[TIM1_ETR]  -               | 30 PC5/SPI_SCK             -
   20 PB2/AIN2[TIM1_CH3N] -               | 29 PC4/TIM1_CH4            IR_RCV 
   21 PB1/AIN1[TIM1_CH2N] -               | 28 PC3/TIM1_CH3            DI_3V3 
   22 PB0/AIN0[TIM1_CH1N] -               | 27 PC2/TIM1_CH2            - 
   23 PE7/AIN8            SQW             | 26 PC1/TIM1_CH1/UART2_CK   -
   24 PE6/AIN9                            | 25 PE5/SPI_NSS             - 
   ---------------------------------------------------------------------
   NOTE  : PA1, PA2, PG0 and PG1 do NOT have interrupt capability!
-----------------------------------------------------------------------------------------------*/
#include <iostm8s105c6.h>
#include <intrinsics.h> 
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

//-----------------------------------------------------------------------------------------------
// LED 00 - LED 03 : segment e
// LED 04 - LED 07 : segment d
// LED 08 - LED 11 : segment c
// LED 12 - LED 15 : segment g
// LED 16 - LED 19 : segment b
// LED 20 - LED 23 : segment a
// LED 24 - LED 27 : segment f
// LED 28          : decimal-point
//-----------------------------------------------------------------------------------------------
#define SEG_DP  (0x80)
#define SEG_A   (0x40)
#define SEG_B   (0x20)
#define SEG_C   (0x10)
#define SEG_D   (0x08)
#define SEG_E   (0x04)
#define SEG_F   (0x02)
#define SEG_G   (0x01)

#define I2C_SCL (0x02) /* PE1 */
#define I2C_SDA (0x04) /* PE2 */
#define DI_3V3  (0x08) /* PC3 */
#define IR_RCV  (0x10) /* PC4 */
#define TX      (0x20) /* PD5 */
#define RX      (0x40) /* PD6 */
#define SQW     (0x80) /* PE7 */
#define IRQ_LED (0x40) /* PE6 */

#define DI_3V3b (PC_ODR_ODR3)
#define IR_RCVb (PC_IDR_IDR4)
#define IRQ_LEDb (PE_ODR_ODR6)

//-----------------------------------------------------------------------------------------------
// https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
//
// At 16 MHz, 1 NOP is approx. 62.5 nsec.
//
// Symbol Parameter	                Min	Typical	Max	Units Measured
// T0H	  0 code ,high voltage time	200	350	500	ns    360..400
// T1H	  1 code ,high voltage time	550	700	5.500	ns    760
// TLD	  data, low voltage time	450	600	5.000	ns    1120
// TLL	  latch, low voltage time	6.000			ns    3120 (max)
//
//-----------------------------------------------------------------------------------------------
#define wait_T0H  __asm("nop\n nop\n nop\n nop\n nop")
#define wait_T1H  wait_T0H; wait_T0H; __asm("nop")

#define ws2812b_send_1   DI_3V3b = 1; /* Turn PC3 on */  \
                         wait_T1H;                       \
                         DI_3V3b = 0; /* Turn PC3 off */ 
#define ws2812b_send_0   DI_3V3b = 1; /* Turn PC3 on */  \
                         wait_T0H;                       \
                         DI_3V3b = 0; /* Turn PC3 off */ 
                             
//-------------------------------------------------
// The Number of WS2812B devices present
// For the binary clock, this is a total of 20
//-------------------------------------------------
#define NR_BOARDS         (6)
#define NR_LEDS_PER_BOARD (29)   /* 4 * 7-segments + 1 dp */
#define NR_LEDS           (NR_LEDS_PER_BOARD * NR_BOARDS)                    
#define LED_INTENSITY     (0x10) /* initial value for LED intensity */

//-------------------------------------------------
// Constants for the independent watchdog (IWDG)
//-------------------------------------------------
#define IWDG_KR_KEY_ENABLE  (0xCC)
#define IWDG_KR_KEY_REFRESH (0xAA)
#define IWDG_KR_KEY_ACCESS  (0x55)

//-------------------------------------------------
// Address values (16-bit) for EEPROM
//-------------------------------------------------
#define EEP_ADDR_INTENSITY_B (0x0C) /* LED intensity Blue */
#define EEP_ADDR_INTENSITY_G (0x0E) /* LED intensity Green */
#define EEP_ADDR_INTENSITY_R (0x10) /* LED intensity Red */
#define EEP_ADDR_BBEGIN_H    (0x12) /* Blanking begin-time hours */
#define EEP_ADDR_BBEGIN_M    (0x13) /* Blanking begin-time minutes */
#define EEP_ADDR_BEND_H      (0x14) /* Blanking end-time hours */
#define EEP_ADDR_BEND_M      (0x15) /* Blanking end-time minutes */
#define EEP_ADDR_DST_ACTIVE  (0x20) /* 1 = Day-light Savings Time active */
 
//-------------------------------------------------
// VS1838B IR infrared remote
//-------------------------------------------------
// see: https://www.sbprojects.net/knowledge/ir/nec.php

#define CLK_TICKS         (32) // 1 clock-tick = 32 usec
// LSB first, 1 start bit + 16 bit address + 8 bit command + 8 bit inverted command + 1 stop bit.
#define NEC_ADDRESS_BITS  (16) // 16 bit address or 8 bit address and 8 bit inverted address
#define NEC_COMMAND_BITS  (16) // Command and inverted command
#define NEC_BITS          (NEC_ADDRESS_BITS + NEC_COMMAND_BITS)
#define NEC_UNIT          (560)

#define NEC_HDR_MARK      (16 * NEC_UNIT) /* mark (0) time for header */
#define NEC_HDR_SPACE     (8 * NEC_UNIT)  /* space (1) time for header */
#define NEC_BIT_MARK      (NEC_UNIT)      
#define NEC_ONE_SPACE     (3 * NEC_UNIT)  /* space length of a one */
#define NEC_ZERO_SPACE    (NEC_UNIT)      /* space length of a zero */
#define NEC_RPT_SPACE     (4 * NEC_UNIT)  /* repeat space length */

#define HDR_MARK_LTICKS   ((NEC_HDR_MARK/CLK_TICKS)   - 20) /* approx.  7 % less */
#define HDR_MARK_HTICKS   ((NEC_HDR_MARK/CLK_TICKS)   + 20) /* approx.  7 % more */
#define HDR_SPACE_LTICKS  ((NEC_HDR_SPACE/CLK_TICKS)  - 10) /* approx.  7 % less */
#define HDR_SPACE_HTICKS  ((NEC_HDR_SPACE/CLK_TICKS)  + 10) /* approx.  7 % more */
#define ONE_SPACE_LTICKS  ((NEC_ONE_SPACE/CLK_TICKS)  -  5) /* approx. 10 % less */
#define ONE_SPACE_HTICKS  ((NEC_ONE_SPACE/CLK_TICKS)  +  5) /* approx. 10 % more */
#define BIT_MARK_LTICKS   ((NEC_BIT_MARK/CLK_TICKS)   -  3) /* approx. 17 % less */
#define BIT_MARK_HTICKS   ((NEC_BIT_MARK/CLK_TICKS)   +  3) /* approx. 17 % more */
#define ZERO_SPACE_LTICKS ((NEC_ZERO_SPACE/CLK_TICKS) -  3) /* approx. 17 % less */
#define ZERO_SPACE_HTICKS ((NEC_ZERO_SPACE/CLK_TICKS) +  3) /* approx. 17 % more */
#define RPT_SPACE_LTICKS  ((NEC_RPT_SPACE/CLK_TICKS)  - 10) /* approx. 14 % less */
#define RPT_SPACE_HTICKS  ((NEC_RPT_SPACE/CLK_TICKS)  + 10) /* approx. 14 % more */
                         
// ISR State-Machine : Receiver States
#define STATE_IDLE      0
#define STATE_MARK      1
#define STATE_SPACE     2
#define STATE_STOP      3

// KEY values for remote control
#define IR_CHARS         "0123456789ULRDOAHX?"
#define IR_0             (0x00)
#define IR_1		 (0x01)
#define IR_2		 (0x02)
#define IR_3		 (0x03)
#define IR_4		 (0x04)
#define IR_5		 (0x05)
#define IR_6		 (0x06)
#define IR_7		 (0x07)
#define IR_8		 (0x08)
#define IR_9		 (0x09)
#define IR_UP            (0x0A)
#define IR_LEFT          (0x0B)
#define IR_RIGHT         (0x0C)
#define IR_DOWN          (0x0D)
#define IR_OK            (0x0E)
#define IR_ASTERISK      (0x0F)
#define IR_HASH          (0x10)
#define IR_REPEAT        (0x11)
#define IR_NONE          (0x12)

#define IR_CODE_0        (0x00FF4AB5)
#define IR_CODE_1	 (0x00FF6897)
#define IR_CODE_2	 (0x00FF9867)
#define IR_CODE_3	 (0x00FFB04F)
#define IR_CODE_4	 (0x00FF30CF)
#define IR_CODE_5	 (0x00FF18E7)
#define IR_CODE_6	 (0x00FF7A85)
#define IR_CODE_7	 (0x00FF10EF)
#define IR_CODE_8	 (0x00FF38C7)
#define IR_CODE_9	 (0x00FF5AA5)
#define IR_CODE_UP       (0x00FF629D)
#define IR_CODE_LEFT     (0x00FF22DD)
#define IR_CODE_RIGHT    (0x00FFC23D)
#define IR_CODE_DOWN     (0x00FFA857)
#define IR_CODE_OK       (0x00FF02FD)
#define IR_CODE_ASTERISK (0x00FF42BD)
#define IR_CODE_HASH     (0x00FF52AD)
#define IR_CODE_REPEAT   (0xFFFFFFFF)

#define IR_CMD_IDLE      (0)
#define IR_CMD_0         (1)
#define IR_CMD_1         (2)
#define IR_CMD_2         (3)
#define IR_CMD_3         (4)
#define IR_CMD_4         (5)
#define IR_CMD_5         (6)
#define IR_CMD_6         (7) /* Invert Blanking-Active signal */
#define IR_CMD_7         (8) /* Enable Testpattern */
#define IR_CMD_8         (9) /* Set blanking-begin time */
#define IR_CMD_9        (10) /* Set blanking-end time */
#define IR_CMD_HASH     (11) /* Show date & year for 10 seconds */
#define IR_CMD_CURSOR   (12)
#define IR_CMD_COL_CURSOR (13)
                         
// Defines for show_date_IR variable
#define IR_SHOW_TIME     (0)
#define IR_SHOW_DATE     (1)
#define IR_SHOW_YEAR     (2)
#define IR_SHOW_TEMP     (3)
#define IR_SHOW_VER      (4)
#define IR_SHOW_ESP_STAT (5)
                         
// Defines for set_time_IR variable
#define IR_NO_TIME      (0)
#define IR_BB_TIME      (1)
#define IR_BE_TIME      (2)

#define COL_RED          (1)
#define COL_GREEN        (2)
#define COL_YELLOW       (COL_RED + COL_GREEN)
#define COL_BLUE         (4)
#define COL_MAGENTA      (COL_RED + COL_BLUE)
#define COL_CYAN         (COL_GREEN + COL_BLUE)
#define COL_WHITE        (COL_RED + COL_GREEN + COL_BLUE)
                         
#define ESP8266_INIT   (0)
#define ESP8266_UPDATE (1)
                         
void     print_date_and_time(void);
void     print_dow(uint8_t dow);
uint16_t cmin(uint8_t h, uint8_t m);
uint8_t  encode_to_bcd2(uint8_t x);
uint16_t encode_to_bcd4(uint16_t x);
bool     blanking_active(void);
void     clear_all_leds(void);
void     check_and_set_summertime(void);
void     execute_single_command(char *s);
void     rs232_command_handler(void);

#endif
