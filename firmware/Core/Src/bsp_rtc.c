#include "bsp_rtc.h"
#include "app_config.h"
#include "bsp_uart.h"
#include <stdio.h>

RTC_HandleTypeDef hrtc;

static uint8_t s_rtc_valid;
static uint8_t s_backup_ok;
static uint8_t s_lse_ok;

static uint8_t bsp_rtc_is_leap_year(uint16_t year)
{
  if ((year % 400U) == 0U)
  {
    return 1U;
  }
  if ((year % 100U) == 0U)
  {
    return 0U;
  }
  return ((year % 4U) == 0U) ? 1U : 0U;
}

static uint8_t bsp_rtc_days_in_month(uint16_t year, uint8_t month)
{
  static const uint8_t days[] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

  if ((month < 1U) || (month > 12U))
  {
    return 0U;
  }
  if ((month == 2U) && (bsp_rtc_is_leap_year(year) != 0U))
  {
    return 29U;
  }
  return days[month - 1U];
}

static uint8_t bsp_rtc_weekday(uint16_t year, uint8_t month, uint8_t day)
{
  static const uint8_t month_offsets[] = {0U, 3U, 2U, 5U, 0U, 3U, 5U, 1U, 4U, 6U, 2U, 4U};
  uint16_t y = year;
  uint8_t weekday;

  if (month < 3U)
  {
    y--;
  }

  weekday = (uint8_t)((y + (y / 4U) - (y / 100U) + (y / 400U) + month_offsets[month - 1U] + day) % 7U);
  return (weekday == 0U) ? RTC_WEEKDAY_SUNDAY : weekday;
}

static uint8_t bsp_rtc_validate_datetime(uint16_t year, uint8_t month, uint8_t day,
                                         uint8_t hour, uint8_t min, uint8_t sec)
{
  uint8_t max_day;

  if ((year < 2000U) || (year > 2099U) || (month < 1U) || (month > 12U))
  {
    return 0U;
  }

  max_day = bsp_rtc_days_in_month(year, month);
  if ((day < 1U) || (day > max_day) || (hour > 23U) || (min > 59U) || (sec > 59U))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t bsp_rtc_config_lse_clock(void)
{
#if (APP_RTC_USE_LSE == 1U)
  RCC_OscInitTypeDef osc = {0};
  uint32_t rtc_source;

  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  if (__HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
  {
    osc.OscillatorType = RCC_OSCILLATORTYPE_LSE;
    osc.LSEState = RCC_LSE_ON;
    osc.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
      return 0U;
    }
  }

  rtc_source = (RCC->BDCR & RCC_BDCR_RTCSEL);
  if (rtc_source == 0U)
  {
    __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
  }
  else if (rtc_source != RCC_RTCCLKSOURCE_LSE)
  {
    return 0U;
  }

  __HAL_RCC_RTC_ENABLE();
  return 1U;
#else
  return 0U;
#endif
}

static uint8_t bsp_rtc_hal_init(void)
{
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127U;
  hrtc.Init.SynchPrediv = 255U;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

  return (HAL_RTC_Init(&hrtc) == HAL_OK) ? 1U : 0U;
}

uint8_t bsp_rtc_set_datetime(uint16_t year, uint8_t month, uint8_t day,
                             uint8_t hour, uint8_t min, uint8_t sec)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};

  if ((s_lse_ok == 0U) || (bsp_rtc_validate_datetime(year, month, day, hour, min, sec) == 0U))
  {
    return 0U;
  }

  time.Hours = hour;
  time.Minutes = min;
  time.Seconds = sec;
  time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  time.StoreOperation = RTC_STOREOPERATION_RESET;

  date.WeekDay = bsp_rtc_weekday(year, month, day);
  date.Month = month;
  date.Date = day;
  date.Year = (uint8_t)(year - 2000U);

  if (HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN) != HAL_OK)
  {
    s_rtc_valid = 0U;
    return 0U;
  }
  if (HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN) != HAL_OK)
  {
    s_rtc_valid = 0U;
    return 0U;
  }

  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, APP_RTC_MAGIC);
  s_rtc_valid = 1U;
  s_backup_ok = 1U;
  return 1U;
}

void bsp_rtc_reset_to_default(void)
{
  (void)bsp_rtc_set_datetime(APP_RTC_DEFAULT_YEAR,
                             APP_RTC_DEFAULT_MONTH,
                             APP_RTC_DEFAULT_DAY,
                             APP_RTC_DEFAULT_HOUR,
                             APP_RTC_DEFAULT_MIN,
                             APP_RTC_DEFAULT_SEC);
}

void bsp_rtc_init(void)
{
#if (APP_RTC_ENABLE == 1U)
  uint32_t magic;
  char time_buf[24];

  s_rtc_valid = 0U;
  s_backup_ok = 0U;
  s_lse_ok = 0U;

  if (bsp_rtc_config_lse_clock() == 0U)
  {
    bsp_uart_printf("rtc_event error reason=lse_timeout\r\n");
    return;
  }
  s_lse_ok = 1U;

  if (bsp_rtc_hal_init() == 0U)
  {
    bsp_uart_printf("rtc_event error reason=hal_error\r\n");
    return;
  }

  magic = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
  if (magic == APP_RTC_MAGIC)
  {
    s_backup_ok = 1U;
    s_rtc_valid = 1U;
    bsp_uart_printf("rtc_event restored backup=ok\r\n");
    return;
  }

  bsp_rtc_reset_to_default();
  bsp_rtc_get_datetime_string(time_buf, sizeof(time_buf));
  if (s_rtc_valid != 0U)
  {
    bsp_uart_printf("rtc_event init_default time=%s reason=no_magic\r\n", time_buf);
  }
  else
  {
    bsp_uart_printf("rtc_event error reason=hal_error\r\n");
  }
#endif
}

void bsp_rtc_process(void)
{
}

uint8_t bsp_rtc_is_valid(void)
{
  return s_rtc_valid;
}

void bsp_rtc_get_datetime_string(char *buf, uint16_t buf_len)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};
  uint16_t year = 1970U;
  uint8_t month = 1U;
  uint8_t day = 1U;
  uint8_t hour = 0U;
  uint8_t min = 0U;
  uint8_t sec = 0U;

  if ((buf == 0) || (buf_len == 0U))
  {
    return;
  }

  if ((s_rtc_valid != 0U) &&
      (HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN) == HAL_OK) &&
      (HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN) == HAL_OK))
  {
    year = (uint16_t)(2000U + date.Year);
    month = date.Month;
    day = date.Date;
    hour = time.Hours;
    min = time.Minutes;
    sec = time.Seconds;
  }

  (void)snprintf(buf, buf_len, "%04u-%02u-%02u %02u:%02u:%02u",
                 year, month, day, hour, min, sec);
}

void bsp_rtc_print_log(void)
{
#if (APP_RTC_ENABLE == 1U) && (APP_RTC_LOG_ENABLE == 1U)
  char time_buf[24];

  bsp_rtc_get_datetime_string(time_buf, sizeof(time_buf));
  bsp_uart_printf("rtc time=%s valid=%u source=LSE backup=%s\r\n",
                  time_buf,
                  (unsigned int)s_rtc_valid,
                  (s_backup_ok != 0U) ? "ok" : "lost");
#endif
}

void bsp_rtc_print_backup_log(void)
{
#if (APP_RTC_ENABLE == 1U)
  uint32_t magic = 0U;

  if (s_lse_ok != 0U)
  {
    magic = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);
  }

  bsp_uart_printf("rtc_bkp magic=%04lX backup=%s\r\n",
                  (unsigned long)(magic & 0xFFFFUL),
                  (s_backup_ok != 0U) ? "ok" : "lost");
#endif
}
