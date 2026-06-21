#ifndef BSP_DAC_THRESHOLD_H
#define BSP_DAC_THRESHOLD_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

// Bind DAC handle used to control comparator threshold.
void bsp_dac_threshold_init(DAC_HandleTypeDef *hdac);
// Set DAC_OUT1 threshold in millivolts.
void bsp_dac_threshold_set_mv(uint16_t threshold_mv);
// Get currently applied DAC_OUT1 threshold in millivolts.
uint16_t bsp_dac_threshold_get_mv(void);

#endif
