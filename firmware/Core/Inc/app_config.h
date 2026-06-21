#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#define APP_BIT_RATE_HZ                100000U
#define APP_TIM2_TICK_HZ               100000U
#define APP_TIM2_ARR_100KHZ            839U

#define APP_TIM1_PWM_PERIOD            167U
#define APP_TIM1_CARRIER_CCR_ON        84U
#define APP_TIM1_CARRIER_CCR_OFF       0U

#define APP_RX_EXPECTED_EDGES_PER_BIT  10U
#define APP_RX_EDGE_THRESHOLD          6U
#define APP_RX_USE_EDGE_COUNTER        1U

#define APP_FRAME_PREAMBLE_BYTE        0xAAU
#define APP_FRAME_PREAMBLE_BYTES       3U
#define APP_FRAME_SYNC_BYTE            0xD5U
#define APP_MAX_PAYLOAD_LEN            32U
#define APP_RX_QUEUE_DEPTH             32U
#define APP_RX_ERROR_EVENT_QUEUE_DEPTH 16U

#define APP_BOARD_ROLE_TX_ONLY         0U
#define APP_BOARD_ROLE_RX_ONLY         1U
#define APP_BOARD_ROLE_TX_RX_LOOPBACK  2U

#define APP_BOARD_ROLE                 APP_BOARD_ROLE_TX_RX_LOOPBACK

#if (APP_BOARD_ROLE > APP_BOARD_ROLE_TX_RX_LOOPBACK)
#error "Invalid APP_BOARD_ROLE"
#endif

#define APP_DAC_DEFAULT_THRESHOLD_MV   1650U

#define APP_ADC_CHANNEL_COUNT          7U
#define APP_ADC_VREF_MV                3300U

#define APP_LOG_PERIOD_MS              1000U
#define APP_ERR_LOG_MAX_PER_PERIOD     5U
#define APP_ERR_BITS_MAX_PER_PERIOD    1U
#define APP_ERR_BITS_ENABLE            1U

#define APP_RTC_ENABLE                 1U
#define APP_RTC_USE_LSE                1U
#define APP_RTC_MAGIC                  0x32F4U
#define APP_RTC_DEFAULT_YEAR           2026U
#define APP_RTC_DEFAULT_MONTH          1U
#define APP_RTC_DEFAULT_DAY            1U
#define APP_RTC_DEFAULT_HOUR           0U
#define APP_RTC_DEFAULT_MIN            0U
#define APP_RTC_DEFAULT_SEC            0U
#define APP_RTC_LOG_ENABLE             1U

#endif
