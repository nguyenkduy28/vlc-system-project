#include "bsp_adc_monitor.h"

static ADC_HandleTypeDef *s_hadc;
static uint16_t s_adc_raw[APP_ADC_CHANNEL_COUNT];

// Convert ADC raw code to millivolts with divider scale factor.
static uint16_t raw_to_mv(uint16_t raw, uint16_t scale_x10)
{
  uint32_t mv = ((uint32_t)raw * APP_ADC_VREF_MV) / 4095U;
  mv = (mv * scale_x10) / 10U;
  return (uint16_t)mv;
}

// Store ADC handle for monitor service.
void bsp_adc_monitor_init(ADC_HandleTypeDef *hadc)
{
  s_hadc = hadc;
}

// Start ADC DMA stream in circular mode for continuous updates.
void bsp_adc_monitor_start(void)
{
  if (s_hadc == 0)
  {
    return;
  }

  if (s_hadc->DMA_Handle != 0)
  {
    s_hadc->DMA_Handle->Init.Mode = DMA_CIRCULAR;
    HAL_DMA_Init(s_hadc->DMA_Handle);
    __HAL_LINKDMA(s_hadc, DMA_Handle, *s_hadc->DMA_Handle);
  }

  HAL_ADC_Start_DMA(s_hadc, (uint32_t *)s_adc_raw, APP_ADC_CHANNEL_COUNT);
}

// Copy a snapshot of current raw ADC samples.
void bsp_adc_monitor_get_raw(uint16_t *dst, uint8_t count)
{
  uint8_t i;
  if (dst == 0)
  {
    return;
  }

  if (count > APP_ADC_CHANNEL_COUNT)
  {
    count = APP_ADC_CHANNEL_COUNT;
  }

  for (i = 0U; i < count; ++i)
  {
    dst[i] = s_adc_raw[i];
  }
}

// Convert current ADC snapshot to engineering voltages.
void bsp_adc_monitor_get_voltages(bsp_adc_voltages_t *out)
{
  if (out == 0)
  {
    return;
  }

  // ADC rank map:
  // raw[0]=PA1 RX_OUT_A, raw[1]=PB0 VMON_BU, raw[2]=PB1 VMON_BU_3V,
  // raw[3]=PC0 VMON_MAIN_SYS, raw[4]=PC1 VMON_MAIN,
  // raw[5]=PA5 VMON_5V_SYS, raw[6]=PA6 VMON_3V3_SYS.
  out->rx_out_a_mv     = raw_to_mv(s_adc_raw[0], 10U);
  out->vmon_bu_mv      = raw_to_mv(s_adc_raw[1], 32U);
  out->vmon_bu_3v_mv   = raw_to_mv(s_adc_raw[2], 15U);
  out->vmon_main_sys_mv= raw_to_mv(s_adc_raw[3], 32U);
  out->vmon_main_mv    = raw_to_mv(s_adc_raw[4], 32U);
  out->vmon_5v_sys_mv  = raw_to_mv(s_adc_raw[5], 20U);
  out->vmon_3v3_sys_mv = raw_to_mv(s_adc_raw[6], 15U);
}
