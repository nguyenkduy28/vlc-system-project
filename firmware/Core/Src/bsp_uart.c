#include "bsp_uart.h"
#include <stdio.h>

static UART_HandleTypeDef *s_huart;

// Store UART handle for logging backend.
void bsp_uart_init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
}

// Send formatted debug text via blocking UART TX.
void bsp_uart_printf(const char *fmt, ...)
{
  char buf[384];
  int n;
  va_list args;

  if ((s_huart == 0) || (fmt == 0))
  {
    return;
  }

  va_start(args, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (n <= 0)
  {
    return;
  }

  if (n >= (int)sizeof(buf))
  {
    n = (int)sizeof(buf) - 1;
  }

  HAL_UART_Transmit(s_huart, (uint8_t *)buf, (uint16_t)n, 50U);
}

// Poll one UART byte with zero timeout so command parsing can run in foreground.
uint8_t bsp_uart_read_byte(uint8_t *out_byte)
{
  if ((s_huart == 0) || (out_byte == 0))
  {
    return 0U;
  }

  return (HAL_UART_Receive(s_huart, out_byte, 1U, 0U) == HAL_OK) ? 1U : 0U;
}
