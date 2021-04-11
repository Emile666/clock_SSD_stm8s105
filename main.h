#ifndef CLOCK_SSD_H
#define CLOCK_SSD_H

#include <iostm8s105c6.h>
#include <intrinsics.h> 
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

/*-----------------------------------------------------------------------------------------------
  Schematic of the connections to the MCU.
  
                                 STM8S105C6T6
      MCU pin-name            Function    |    MCU pin-name        Function
   ---------------------------------------|--------------------------------
   01 NRST                                | 13 VDDA
   02 PA1/OSC                 -           | 14 VSSA
   03 PA2/OSCOUT              -           | 15 PB7/AIN7            -
   04 VSSIO_1                             | 16 PB6/AIN6            -
   05 VSS                                 | 17 PB5/AIN5[I2C_SDA]   -
   06 VCAP                                | 18 PB4/AIN4[I2C_SCL]   -
   07 VDD                                 | 19 PB3/AIN3[TIM1_ETR]  -
   08 VDDIO_1                             | 20 PB2/AIN2[TIM1_CH3N] -
   09 PA3/TIM2_CH3[TIME3_CH1]             | 21 PB1/AIN1[TIM1_CH2N] -
   10 PA4                                 | 22 PB0/AIN0[TIM1_CH1N] -
   11 PA5                                 | 23 PE7/AIN8            SQW
   12 PA6                                 | 24 PE6/AIN9            
   ---------------------------------------|--------------------------------
   25 PE5/SPI_NSS             -           | 37 PE3/TIM1_BKIN       -
   26 PC1/TIM1_CH1/UART2_CK   -           | 38 PE2/I2C_SDA         I2C_SDA
   27 PC2/TIM1_CH2            -           | 39 PE1/I2C_SCL         I2C_SCL
   28 PC3/TIM1_CH3            DI_3V3      | 40 PE0/CLK_CC0         -
   29 PC4/TIM1_CH4            IR_RCV      | 41 PD0/TIM3_CH2...     -
   30 PC5/SPI_SCK             -           | 42 PD1/SWIM            SWIM
   31 VSSIO_2                             | 43 PD2/TIM3_CH1...     - 
   32 VDDIO_2                             | 44 PD3/TIM2_CH2...     -
   33 PC6/SPI_MOSI            -           | 45 PD4/TIM2_CH1[BEEP]  -
   34 PC7/SPI_MISO            -           | 46 PD5/UART2_TX        TX
   35 PG0                     -           | 47 PD6/UART2_RX        RX
   36 PG1                     -           | 48 PD7/TLI[TIM1_CH4]   -
   ---------------------------------------------------------------------
   
   NOTE  : PA1, PA2, PG0 and PG1 do NOT have interrupt capability!
-----------------------------------------------------------------------------------------------*/

//-----------------------------------------------------------------------------------------------
// LED 01 - LED 04 : segment e
// LED 05 - LED 08 : segment d
// LED 09 - LED 12 : segment c
// LED 13 - LED 16 : segment g
// LED 17 - LED 20 : segment b
// LED 21 - LED 24 : segment a
// LED 25 - LED 28 : segment f
// LED 29          : decimal-point
//-----------------------------------------------------------------------------------------------
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

#define ws2812b_send_1   PC_ODR |=  DI_3V3; /* Turn PC3 on */  \
                         wait_T1H;                           \
                         PC_ODR &= ~DI_3V3; /* Turn PC3 off */ 
#define ws2812b_send_0   PC_ODR |=  DI_3V3; /* Turn PC3 on */  \
                         wait_T0H;                           \
                         PC_ODR &= ~DI_3V3; /* Turn PC3 off */ 
                             
//-------------------------------------------------
// The Number of WS2812B devices present
// For the binary clock, this is a total of 20
//-------------------------------------------------
#define NR_BOARDS (6)
#define NR_LEDS   (29 * NR_BOARDS)                    
#define LED_INTENSITY (0x10) /* initial value for LED intensity */

//-------------------------------------------------
// Constants for the independent watchdog (IWDG)
//-------------------------------------------------
#define IWDG_KR_KEY_ENABLE  (0xCC)
#define IWDG_KR_KEY_REFRESH (0xAA)
#define IWDG_KR_KEY_ACCESS  (0x55)

//-------------------------------------------------
// Constants for EEPROM
//-------------------------------------------------
#define EEP_ADDR_INTENSITY  (0x10) /* LED intensity */
#define EEP_ADDR_DST_ACTIVE (0x12) /* 1 = Day-light Savings Time active */
#define EEP_ADDR_BBEGIN_H   (0x14) /* Blanking begin-time hours */
#define EEP_ADDR_BBEGIN_M   (0x16) /* Blanking begin-time minutes */
#define EEP_ADDR_BEND_H     (0x18) /* Blanking end-time hours */
#define EEP_ADDR_BEND_M     (0x1A) /* Blanking end-time minutes */
                         
void     print_date_and_time(void);
void     print_dow(uint8_t dow);
uint16_t cmin(uint8_t h, uint8_t m);
bool     blanking_active(void);
void     check_and_set_summertime(void);
void     execute_single_command(char *s);
void     rs232_command_handler(void);

#endif
