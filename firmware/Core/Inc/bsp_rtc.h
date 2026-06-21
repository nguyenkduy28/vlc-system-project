#ifndef BSP_RTC_H
#define BSP_RTC_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

extern RTC_HandleTypeDef hrtc;

void bsp_rtc_init(void);
void bsp_rtc_process(void);
uint8_t bsp_rtc_is_valid(void);
void bsp_rtc_get_datetime_string(char *buf, uint16_t buf_len);
void bsp_rtc_print_log(void);
uint8_t bsp_rtc_set_datetime(uint16_t year, uint8_t month, uint8_t day,
                             uint8_t hour, uint8_t min, uint8_t sec);
void bsp_rtc_reset_to_default(void);
void bsp_rtc_print_backup_log(void);

#endif
