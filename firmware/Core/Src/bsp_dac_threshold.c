#include "bsp_dac_threshold.h"
#include "app_config.h"

static DAC_HandleTypeDef *s_hdac;
static uint16_t s_threshold_mv;

// Store DAC handle for threshold output service.
void bsp_dac_threshold_init(DAC_HandleTypeDef *hdac)
{
  s_hdac = hdac;
  s_threshold_mv = 0U;
}

// Program comparator threshold voltage on DAC channel 1.
void bsp_dac_threshold_set_mv(uint16_t threshold_mv)
{
  uint32_t dac_raw;

  if (s_hdac == 0)
  {
    return;
  }

  if (threshold_mv > APP_ADC_VREF_MV)
  {
    threshold_mv = APP_ADC_VREF_MV;
  }

  dac_raw = ((uint32_t)threshold_mv * 4095U) / APP_ADC_VREF_MV;

  HAL_DAC_Start(s_hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(s_hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_raw);
  s_threshold_mv = threshold_mv;
}

// Return the last threshold value applied through BSP setter.
uint16_t bsp_dac_threshold_get_mv(void)
{
  return s_threshold_mv;
}
