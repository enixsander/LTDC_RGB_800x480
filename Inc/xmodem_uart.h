#ifndef __XMODEM_UART_H
#define __XMODEM_UART_H

#include "main.h"

void start_recv(void);
void uart_xmodem_receive(void);
static uint8_t check_crc(const uint8_t *buf,uint16_t size); 
static int16_t rx_usart(uint16_t ms);

#endif // __XMODEM_UART_H 
