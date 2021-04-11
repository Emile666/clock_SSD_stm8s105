#ifndef _DELAY_H
#define _DELAY_H

#include <stdint.h>

#define wait_for_interrupt() {_asm("wfi\n");} /* Wait For Interrupt */

uint32_t millis(void);
void     delay_msec(uint16_t ms);
void     delay_usec(uint16_t us);

#endif