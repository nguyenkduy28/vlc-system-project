#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdarg.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

// Bind UART handle used by debug logger.
void bsp_uart_init(UART_HandleTypeDef *huart);
// Print formatted text to UART (blocking).
void bsp_uart_printf(const char *fmt, ...);
// Poll one received UART byte without blocking.
uint8_t bsp_uart_read_byte(uint8_t *out_byte);

#endif
