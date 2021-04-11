#ifndef _STM8_UART_H
#define _STM8_UART_H

#include <intrinsics.h>
#include <stdint.h>
#include <stdbool.h>
#include "main.h"

#define UART_BUFLEN (15)
#define TX_BUF_SIZE (20)
#define RX_BUF_SIZE (20)

void    uart_init(void);
void    uart_printf(char *s);
bool    uart_kbhit(void);
uint8_t uart_getc(void);
void    uart_putc(uint8_t ch);

#endif