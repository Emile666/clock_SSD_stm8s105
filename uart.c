/*==================================================================
  File Name    : uart.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This file contains the UART related functions 
            for the STM8 uC.
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
  ==================================================================*/ 
#include <stdio.h>
#include "delay.h"
#include "uart.h"
#include "ring_buffer.h"

// buffers for use with the ring buffer (belong to the USART)
bool     ovf_buf_in; // true = input buffer overflow
uint16_t isr_cnt = 0;

struct ring_buffer ring_buffer_out;
struct ring_buffer ring_buffer_in;
uint8_t            out_buffer[TX_BUF_SIZE];
uint8_t            in_buffer[RX_BUF_SIZE];

//-----------------------------------------------------------------------------
// UART Transmit complete Interrupt.
//
// This interrupt will be executed when the TXE (Transmit Data Register Empty)
// bit in UART2_SR is set. The TXE bit is set by hardware when the contents of 
// the TDR register has been transferred into the shift register. An interrupt 
// is generated if the TIEN bit =1 in the UART_CR1 register. It is cleared by a
// write to the UART_DR register.
//-----------------------------------------------------------------------------
#pragma vector=UART2_T_TXE_vector
__interrupt void UART_TX_IRQHandler()
{
	if (!ring_buffer_is_empty(&ring_buffer_out))
	{   // if there is data in the ring buffer, fetch it and send it
		UART2_DR = ring_buffer_get(&ring_buffer_out);
	} // if
    else
    {   // no more data to send, turn off interrupt
        UART2_CR2_TIEN = 0;
    } // else
} /* UART_TX_IRQHandler() */

//-----------------------------------------------------------------------------
// UART Receive Complete Interrupt.

// This interrupt will be executed when the RXNE (Read Data-Register Not Empty)
// bit in UART2_SR is set. This bit is set by hardware when the contents of the 
// RDR shift register has been transferred to the UART2_DR register. An interrupt 
// is generated if RIEN=1 in the UART2_CR2 register. It is cleared by a read to 
// the UART1_DR register. It can also be cleared by writing 0.
//-----------------------------------------------------------------------------
#pragma vector=UART2_R_RXNE_vector
__interrupt void UART_RX_IRQHandler(void)
{
	volatile uint8_t ch;
	
	if (!ring_buffer_is_full(&ring_buffer_in))
	{
		ring_buffer_put(&ring_buffer_in, UART2_DR);
		ovf_buf_in = false;
	} // if
	else
	{
		ch = UART2_DR; // clear RXNE flag
		ovf_buf_in = true;
	} // else
	isr_cnt++;
} /* UART_RX_IRQHandler() */

/*------------------------------------------------------------------
  Purpose  : This function initializes the UART to 115200,N,8,1
             Master clock is 16 MHz, baud-rate is 115200 Baud.
  Variables: -
  Returns  : -
  ------------------------------------------------------------------*/
void uart_init(void)
{
    //
    //  Clear the Idle Line Detected bit in the status register by a read
    //  to the UART1_SR register followed by a Read to the UART1_DR register.
    //
    uint8_t tmp = UART2_SR;
    tmp = UART2_DR;

    //  Reset the UART registers to the reset values.
    UART2_CR1 = 0;
    UART2_CR2 = 0;
    UART2_CR4 = 0;
    UART2_CR3 = 0;
    UART2_CR6 = 0;
    UART2_GTR = 0;
    UART2_PSCR = 0;

    // initialize the in and out buffer for the UART
    ring_buffer_out = ring_buffer_init(out_buffer, TX_BUF_SIZE);
    ring_buffer_in  = ring_buffer_init(in_buffer , RX_BUF_SIZE);

    //  Now setup the port to 115200,n,8,1.
    UART2_CR1_M    = 0;     //  8 Data bits.
    UART2_CR1_PCEN = 0;     //  Disable parity.
    UART2_CR3_STOP = 0;     //  1 stop bit.
    UART2_BRR2     = 0x0b;  //  Set the baud rate registers to 115200 baud
    UART2_BRR1     = 0x08;  //  based upon a 16 MHz system clock.

    //  Disable the transmitter and receiver.
    UART2_CR2_TEN = 0;      //  Disable transmit.
    UART2_CR2_REN = 0;      //  Disable receive.

    //  Set the clock polarity, clock phase and last bit clock pulse.
    UART2_CR3_CPOL = 0;
    UART2_CR3_CPHA = 0;
    UART2_CR3_LBCL = 0;

    //  Turn on the UART transmit, receive and the UART clock.
    UART2_CR2_TIEN = 1; // Enable Transmit interrupt
    UART2_CR2_RIEN = 1; // Enable Receive interrupt
    UART2_CR2_TEN  = 1; // Enable Transmitter
    UART2_CR2_REN  = 1; // Enable Receiver
    UART2_CR3_CKEN = 0; // set to 0 or receive will not work!!
} // uart_init()

/*------------------------------------------------------------------
  Purpose  : This function writes one data-byte to the uart.	
  Variables: ch: the byte to send to the uart.
  Returns  : -
  ------------------------------------------------------------------*/
void uart_putc(uint8_t ch)
{    
    // At 19200 Baud, sending 1 byte takes a max. of 0.52 msec.
    while (ring_buffer_is_full(&ring_buffer_out)) delay_msec(1);
    __disable_interrupt(); // Disable interrupts to get exclusive access to ring_buffer_out
    if (ring_buffer_is_empty(&ring_buffer_out))
    {
        UART2_CR2_TIEN = 1; // First data in buffer, enable data ready interrupt
    } // if
    ring_buffer_put(&ring_buffer_out, ch); // Put data in buffer
    __enable_interrupt(); // Re-enable interrupts
} // uart_putc()

/*------------------------------------------------------------------
  Purpose  : This function writes a string to the UART, using
             the uart_putc() routine.
  Variables:
         s : The string to write to serial port 0
  Returns  : the number of characters written
  ------------------------------------------------------------------*/
void uart_printf(char *s)
{
    char *ch = s;
    while (*ch)
    {
        if (*ch == '\n')
        {
            uart_putc('\r'); // add CR
        } // if
        uart_putc(*ch);
        ch++;                //  Grab the next character.
    } // while
} // uart_printf()

/*------------------------------------------------------------------
  Purpose  : This function checks if a character is present in the
             receive buffer.
  Variables: -
  Returns  : 1 if a character is received, 0 otherwise
  ------------------------------------------------------------------*/
bool uart_kbhit(void)
{
    return !ring_buffer_is_empty(&ring_buffer_in);
} // uart_kbhit()

/*------------------------------------------------------------------
  Purpose  : This function reads one data-byte from the uart.	
  Variables: -
  Returns  : the data-byte read from the uart
  ------------------------------------------------------------------*/
uint8_t uart_getc(void)
{
    return ring_buffer_get(&ring_buffer_in);
} // uart_getch()

