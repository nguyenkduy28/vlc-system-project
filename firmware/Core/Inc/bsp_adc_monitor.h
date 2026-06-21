#ifndef BSP_ADC_MONITOR_H
#define BSP_ADC_MONITOR_H

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "app_config.h"

typedef struct
{
  uint16_t rx_out_a_mv;
  uint16_t vmon_bu_mv;
  uint16_t vmon_bu_3v_mv;
  uint16_t vmon_main_sys_mv;
  uint16_t vmon_main_mv;
  uint16_t vmon_5v_sys_mv;
  // VMON_3V3_SYS uses 1.5x divider scale.
  uint16_t vmon_3v3_sys_mv;
} bsp_adc_voltages_t;

// Bind ADC handle used by DMA monitor.
void bsp_adc_monitor_init(ADC_HandleTypeDef *hadc);
// Start ADC1 DMA in circular mode for continuous sampling.
void bsp_adc_monitor_start(void);
// Copy latest raw ADC samples from DMA buffer.
void bsp_adc_monitor_get_raw(uint16_t *dst, uint8_t count);
// Convert latest ADC samples to scaled millivolt values.
void bsp_adc_monitor_get_voltages(bsp_adc_voltages_t *out);

#endif
