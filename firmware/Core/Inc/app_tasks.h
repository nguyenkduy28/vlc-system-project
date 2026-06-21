#ifndef APP_TASKS_H
#define APP_TASKS_H

#include "stm32f4xx_hal.h"

// Initialize all app modules with CubeMX peripheral handles.
void app_tasks_init(UART_HandleTypeDef *huart2,
                    TIM_HandleTypeDef *htim1,
                    TIM_HandleTypeDef *htim2,
                    TIM_HandleTypeDef *htim4,
                    ADC_HandleTypeDef *hadc1,
                    DAC_HandleTypeDef *hdac);

// Start runtime services: DAC, ADC DMA, TIM4 edge counter, TX PWM, and TIM2 IRQ clock.
void app_tasks_start(void);
// Run non-interrupt background work (logging, queue consume).
void app_tasks_process(void);
// Handle timer period elapsed callback from TIM2 IRQ context.
void app_tasks_on_timer_period_elapsed(TIM_HandleTypeDef *htim);

#endif
