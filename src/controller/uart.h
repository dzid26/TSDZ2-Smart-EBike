/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, MSpider65 2020.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _UART_H
#define _UART_H

#include "main.h"

void uart2_init(void);

#if __SDCC_REVISION < 9624
void uart_put_char(char c);
#else
int uart_put_char(int c);
#endif

#if __SDCC_REVISION < 9989
char uart_get_char(void);
#else
int uart_get_char(void);
#endif

#endif /* _UART_H */

