#include "app_tasks.h"
#include "app_config.h"
#include "board_pins.h"
#include "bsp_uart.h"
#include "bsp_adc_monitor.h"
#include "bsp_dac_threshold.h"
#include "bsp_rtc.h"
#include "vlc_tx.h"
#include "vlc_rx.h"
#include "rx_sync.h"
#include <stdlib.h>
#include <string.h>

#define APP_LINK_STATS_PPM_SCALE 1000000ULL
#define APP_FRAME_BUFFER_SIZE (APP_FRAME_PREAMBLE_BYTES + 1U + 1U + APP_MAX_PAYLOAD_LEN + 1U)
#define APP_UART_CMD_BUFFER_SIZE 128U

static TIM_HandleTypeDef *s_htim2;
static volatile uint32_t s_tick_count;
static app_frame_t s_last_rx;
static uint8_t s_have_last_rx;
static char s_uart_cmd_buffer[APP_UART_CMD_BUFFER_SIZE];
static uint8_t s_uart_cmd_len;
static uint8_t s_uart_cmd_overflow;

typedef struct
{
  uint32_t total_payload_bits_checked;
  uint32_t total_payload_bit_errors;
  uint32_t payload_mismatch_frames;
  uint32_t good_payload_frames;
} app_link_stats_t;

static app_link_stats_t s_link_stats;

static void app_print_tx_frame_log(void);
static void app_print_expected_tx_frame_log(void);
static void app_handle_uart_rx(void);
static void app_handle_command_line(char *line);

static const char *app_board_role_to_text(void)
{
#if (APP_BOARD_ROLE == APP_BOARD_ROLE_TX_ONLY)
  return "TX_ONLY";
#elif (APP_BOARD_ROLE == APP_BOARD_ROLE_RX_ONLY)
  return "RX_ONLY";
#else
  return "TX_RX_LOOPBACK";
#endif
}

static void app_uart_print_hex_bytes(const uint8_t *data, uint8_t len)
{
  uint8_t i;

  for (i = 0U; i < len; ++i)
  {
    bsp_uart_printf("%02X", data[i]);
    if (i + 1U < len)
    {
      bsp_uart_printf(" ");
    }
  }
}

static uint8_t app_ascii_is_space(char c)
{
  return (uint8_t)((c == ' ') || (c == '\t')) ? 1U : 0U;
}

static uint8_t app_parse_fixed_u16(const char *text, uint8_t len, uint16_t *value)
{
  uint8_t i;
  uint16_t v = 0U;

  for (i = 0U; i < len; ++i)
  {
    if ((text[i] < '0') || (text[i] > '9'))
    {
      return 0U;
    }
    v = (uint16_t)((v * 10U) + (uint16_t)(text[i] - '0'));
  }

  *value = v;
  return 1U;
}

static uint8_t app_is_leap_year(uint16_t year)
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

static uint8_t app_days_in_month(uint16_t year, uint8_t month)
{
  static const uint8_t days[] = {31U, 28U, 31U, 30U, 31U, 30U, 31U, 31U, 30U, 31U, 30U, 31U};

  if ((month < 1U) || (month > 12U))
  {
    return 0U;
  }
  if ((month == 2U) && (app_is_leap_year(year) != 0U))
  {
    return 29U;
  }
  return days[month - 1U];
}

static uint8_t app_validate_datetime(uint16_t year, uint8_t month, uint8_t day,
                                     uint8_t hour, uint8_t min, uint8_t sec)
{
  uint8_t max_day;

  if ((year < 2000U) || (year > 2099U) || (month < 1U) || (month > 12U))
  {
    return 0U;
  }

  max_day = app_days_in_month(year, month);
  if ((day < 1U) || (day > max_day) || (hour > 23U) || (min > 59U) || (sec > 59U))
  {
    return 0U;
  }

  return 1U;
}

static uint8_t app_parse_datetime_string(const char *text,
                                         uint16_t *year,
                                         uint8_t *month,
                                         uint8_t *day,
                                         uint8_t *hour,
                                         uint8_t *min,
                                         uint8_t *sec)
{
  uint16_t y;
  uint16_t mo;
  uint16_t d;
  uint16_t h;
  uint16_t mi;
  uint16_t s;

  if ((text == 0) || (strlen(text) != 19U))
  {
    return 0U;
  }
  if ((text[4] != '-') || (text[7] != '-') || (text[10] != ' ') ||
      (text[13] != ':') || (text[16] != ':'))
  {
    return 0U;
  }

  if ((app_parse_fixed_u16(&text[0], 4U, &y) == 0U) ||
      (app_parse_fixed_u16(&text[5], 2U, &mo) == 0U) ||
      (app_parse_fixed_u16(&text[8], 2U, &d) == 0U) ||
      (app_parse_fixed_u16(&text[11], 2U, &h) == 0U) ||
      (app_parse_fixed_u16(&text[14], 2U, &mi) == 0U) ||
      (app_parse_fixed_u16(&text[17], 2U, &s) == 0U))
  {
    return 0U;
  }

  *year = y;
  *month = (uint8_t)mo;
  *day = (uint8_t)d;
  *hour = (uint8_t)h;
  *min = (uint8_t)mi;
  *sec = (uint8_t)s;
  return 1U;
}

static const char *app_rx_state_to_text(uint8_t state)
{
  switch ((rx_sync_state_t)state)
  {
    case RX_SYNC_WAIT_ACTIVITY: return "WAIT_ACTIVITY";
    case RX_SYNC_DETECT_PREAMBLE: return "DETECT_PREAMBLE";
    case RX_SYNC_LOCK_BIT_TIMING: return "LOCK_BIT_TIMING";
    case RX_SYNC_READ_SYNC: return "READ_SYNC";
    case RX_SYNC_READ_LEN: return "READ_LEN";
    case RX_SYNC_READ_PAYLOAD: return "READ_PAYLOAD";
    case RX_SYNC_READ_CHECKSUM: return "READ_CHECKSUM";
    default: return "UNKNOWN";
  }
}

static const char *app_err_reason_to_text(uint8_t reason)
{
  switch (reason)
  {
    case RX_SYNC_ERROR_SYNC: return "sync";
    case RX_SYNC_ERROR_LENGTH: return "length";
    case RX_SYNC_ERROR_PREAMBLE: return "preamble";
    default: return "other";
  }
}

static void app_get_expected_payload(const uint8_t **payload, uint8_t *length)
{
  *payload = app_protocol_get_expected_payload(length);
}

static uint8_t app_get_expected_frame(uint8_t *frame, uint8_t *frame_len)
{
  uint16_t len16 = app_protocol_get_expected_frame(frame, APP_FRAME_BUFFER_SIZE);
  if ((len16 == 0U) || (len16 > APP_FRAME_BUFFER_SIZE))
  {
    *frame_len = 0U;
    return 0U;
  }

  *frame_len = (uint8_t)len16;
  return 1U;
}

static void app_print_tx_status(void)
{
  uint8_t frame[APP_FRAME_BUFFER_SIZE];
  uint8_t frame_len = 0U;
  uint32_t tx_frame_id = 0U;
  uint8_t payload_len;

  if (vlc_tx_get_frame_snapshot(frame, sizeof(frame), &frame_len, &tx_frame_id) == 0U)
  {
    bsp_uart_printf("cmd_err tx_status reason=frame_unavailable\r\n");
    return;
  }

  if (frame_len < (APP_FRAME_PREAMBLE_BYTES + 3U))
  {
    bsp_uart_printf("cmd_err tx_status reason=frame_invalid\r\n");
    return;
  }

  payload_len = frame[APP_FRAME_PREAMBLE_BYTES + 1U];
  if ((uint16_t)(APP_FRAME_PREAMBLE_BYTES + 1U + 1U + payload_len + 1U) > frame_len)
  {
    bsp_uart_printf("cmd_err tx_status reason=frame_invalid\r\n");
    return;
  }

  bsp_uart_printf("tx_status tx_enabled=%u carrier_test=%u tx_frames=%lu tx_frame_id=%lu bit_rate=%u len=%u payload=",
                  (unsigned int)vlc_tx_is_enabled(),
                  (unsigned int)vlc_tx_is_carrier_test(),
                  (unsigned long)vlc_tx_get_frame_count(),
                  (unsigned long)tx_frame_id,
                  (unsigned int)APP_BIT_RATE_HZ,
                  payload_len);
  app_uart_print_hex_bytes(&frame[APP_FRAME_PREAMBLE_BYTES + 2U], payload_len);
  bsp_uart_printf(" checksum=%02X frame=", frame[APP_FRAME_PREAMBLE_BYTES + 2U + payload_len]);
  app_uart_print_hex_bytes(frame, frame_len);
  bsp_uart_printf("\r\n");
}

static void app_handle_tx_command_line(char *line)
{
#if (APP_BOARD_ROLE != APP_BOARD_ROLE_RX_ONLY)
  char *args = 0;
  char *cursor = line;

  while ((*cursor != '\0') && app_ascii_is_space(*cursor))
  {
    cursor++;
  }

  if (*cursor == '\0')
  {
    return;
  }

  args = cursor;
  while ((*args != '\0') && (app_ascii_is_space(*args) == 0U))
  {
    args++;
  }

  if (*args != '\0')
  {
    *args = '\0';
    args++;
    while ((*args != '\0') && app_ascii_is_space(*args))
    {
      args++;
    }
  }
  else
  {
    args = cursor + strlen(cursor);
  }

  if (strcmp(cursor, "tx_start") == 0)
  {
    vlc_tx_set_enabled(1U);
    bsp_uart_printf("cmd_ok tx_start\r\n");
    return;
  }

  if (strcmp(cursor, "tx_stop") == 0)
  {
    vlc_tx_set_enabled(0U);
    bsp_uart_printf("cmd_ok tx_stop\r\n");
    return;
  }

  if (strcmp(cursor, "tx_carrier_on") == 0)
  {
    vlc_tx_set_carrier_test(1U);
    bsp_uart_printf("cmd_ok tx_carrier_on\r\n");
    return;
  }

  if (strcmp(cursor, "tx_carrier_off") == 0)
  {
    vlc_tx_set_carrier_test(0U);
    bsp_uart_printf("cmd_ok tx_carrier_off\r\n");
    return;
  }

  if (strcmp(cursor, "tx_single") == 0)
  {
    vlc_tx_send_single_frame();
    bsp_uart_printf("cmd_ok tx_single\r\n");
    return;
  }

  if (strcmp(cursor, "tx_status") == 0)
  {
    app_print_tx_status();
    return;
  }

  if (strcmp(cursor, "tx_payload") == 0)
  {
    uint8_t payload[APP_MAX_PAYLOAD_LEN];
    uint8_t len = 0U;
    char *token = args;

    while (*token != '\0')
    {
      char *end = token;
      char *next = token;
      char *parse_end = 0;
      unsigned long value;

      while ((*end != '\0') && (app_ascii_is_space(*end) == 0U))
      {
        end++;
      }

      next = end;
      if (*end != '\0')
      {
        *end = '\0';
        next = end + 1;
      }

      if (len >= APP_MAX_PAYLOAD_LEN)
      {
        bsp_uart_printf("cmd_err tx_payload reason=too_long\r\n");
        return;
      }

      value = strtoul(token, &parse_end, 16);
      if ((parse_end == token) || (*parse_end != '\0'))
      {
        bsp_uart_printf("cmd_err tx_payload reason=invalid_hex\r\n");
        return;
      }
      if (value > 0xFFUL)
      {
        bsp_uart_printf("cmd_err tx_payload reason=invalid_hex\r\n");
        return;
      }

      payload[len++] = (uint8_t)value;

      token = next;
      while ((*token != '\0') && app_ascii_is_space(*token))
      {
        token++;
      }
    }

    if (len == 0U)
    {
      bsp_uart_printf("cmd_err tx_payload reason=empty\r\n");
      return;
    }

    if (vlc_tx_set_payload(payload, len) == 0U)
    {
      bsp_uart_printf("cmd_err tx_payload reason=frame_build_failed\r\n");
      return;
    }

    bsp_uart_printf("cmd_ok tx_payload len=%u checksum=%02X\r\n",
                    len,
                    app_protocol_checksum(len, payload));
    return;
  }

  bsp_uart_printf("cmd_err unknown reason=unsupported_command\r\n");
#else
  (void)line;
#endif
}

static uint8_t app_handle_rtc_command_line(char *command, char *args)
{
#if (APP_RTC_ENABLE == 1U)
  if (strcmp(command, "rtc_get") == 0)
  {
    bsp_rtc_print_log();
    return 1U;
  }

  if (strcmp(command, "rtc_bkp") == 0)
  {
    bsp_rtc_print_backup_log();
    return 1U;
  }

  if (strcmp(command, "rtc_reset") == 0)
  {
    char time_buf[24];

    bsp_rtc_reset_to_default();
    if (bsp_rtc_is_valid() == 0U)
    {
      bsp_uart_printf("cmd_err rtc_reset reason=hal_error\r\n");
      return 1U;
    }
    bsp_rtc_get_datetime_string(time_buf, sizeof(time_buf));
    bsp_uart_printf("cmd_ok rtc_reset time=%s\r\n", time_buf);
    return 1U;
  }

  if (strcmp(command, "rtc_set") == 0)
  {
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    char time_buf[24];

    if (app_parse_datetime_string(args, &year, &month, &day, &hour, &min, &sec) == 0U)
    {
      bsp_uart_printf("cmd_err rtc_set reason=invalid_format\r\n");
      return 1U;
    }

    if (app_validate_datetime(year, month, day, hour, min, sec) == 0U)
    {
      bsp_uart_printf("cmd_err rtc_set reason=invalid_range\r\n");
      return 1U;
    }

    if (bsp_rtc_set_datetime(year, month, day, hour, min, sec) == 0U)
    {
      bsp_uart_printf("cmd_err rtc_set reason=hal_error\r\n");
      return 1U;
    }

    bsp_rtc_get_datetime_string(time_buf, sizeof(time_buf));
    bsp_uart_printf("cmd_ok rtc_set time=%s\r\n", time_buf);
    return 1U;
  }
#else
  (void)command;
  (void)args;
#endif

  return 0U;
}

static void app_handle_command_line(char *line)
{
  char *args = 0;
  char *cursor = line;
  char *separator = 0;
  char separator_saved = '\0';

  while ((*cursor != '\0') && app_ascii_is_space(*cursor))
  {
    cursor++;
  }

  if (*cursor == '\0')
  {
    return;
  }

  args = cursor;
  while ((*args != '\0') && (app_ascii_is_space(*args) == 0U))
  {
    args++;
  }

  if (*args != '\0')
  {
    separator = args;
    separator_saved = *args;
    *args = '\0';
    args++;
    while ((*args != '\0') && app_ascii_is_space(*args))
    {
      args++;
    }
  }
  else
  {
    args = cursor + strlen(cursor);
  }

  if (app_handle_rtc_command_line(cursor, args) != 0U)
  {
    return;
  }

  if (separator != 0)
  {
    *separator = separator_saved;
  }

#if (APP_BOARD_ROLE != APP_BOARD_ROLE_RX_ONLY)
  app_handle_tx_command_line(line);
#else
  bsp_uart_printf("cmd_err unknown reason=unsupported_command\r\n");
#endif
}

static void app_handle_uart_rx(void)
{
  uint8_t byte = 0U;

  while (bsp_uart_read_byte(&byte) != 0U)
  {
    if ((byte == '\r') || (byte == '\n'))
    {
      if (s_uart_cmd_overflow != 0U)
      {
        bsp_uart_printf("cmd_err uart reason=line_too_long\r\n");
        s_uart_cmd_len = 0U;
        s_uart_cmd_overflow = 0U;
        continue;
      }

      if (s_uart_cmd_len != 0U)
      {
        s_uart_cmd_buffer[s_uart_cmd_len] = '\0';
        app_handle_command_line(s_uart_cmd_buffer);
        s_uart_cmd_len = 0U;
      }
      continue;
    }

    if ((byte < 0x20U) || (byte > 0x7EU))
    {
      continue;
    }

    if (s_uart_cmd_len >= (APP_UART_CMD_BUFFER_SIZE - 1U))
    {
      s_uart_cmd_overflow = 1U;
      continue;
    }

    s_uart_cmd_buffer[s_uart_cmd_len++] = (char)byte;
  }
}

static void app_print_startup_role_log(void)
{
  bsp_uart_printf("role board_role=%s\r\n", app_board_role_to_text());

#if (APP_BOARD_ROLE == APP_BOARD_ROLE_TX_ONLY)
  bsp_uart_printf("alive_tx tx_frames=%lu tx_frame_id=%lu bit_rate=%u tx_enabled=%u carrier_test=%u\r\n",
                  (unsigned long)vlc_tx_get_frame_count(),
                  (unsigned long)vlc_tx_get_frame_count(),
                  (unsigned int)APP_BIT_RATE_HZ,
                  (unsigned int)vlc_tx_is_enabled(),
                  (unsigned int)vlc_tx_is_carrier_test());
  app_print_tx_frame_log();
#elif (APP_BOARD_ROLE == APP_BOARD_ROLE_RX_ONLY)
  bsp_uart_printf("alive_rx rx_frames=0 frame_errors=0 checksum_errors=0 rx_sync_state=%u last_edge_delta=0 last_raw_bit=0\r\n",
                  (unsigned int)rx_sync_get_state());
  app_print_expected_tx_frame_log();
#else
  bsp_uart_printf("alive tx_frames=%lu rx_frames=0 frame_errors=0 checksum_errors=0 rx_sync_state=%u last_edge_delta=0 last_raw_bit=0\r\n",
                  (unsigned long)vlc_tx_get_frame_count(),
                  (unsigned int)rx_sync_get_state());
#endif
}

// Return 1 when mismatch found and output first mismatching payload byte/bit (MSB-first payload bit index).
static uint8_t app_find_payload_mismatch(const uint8_t *tx_payload,
                                         uint8_t tx_len,
                                         const uint8_t *rx_payload,
                                         uint8_t rx_len,
                                         uint8_t *mismatch_byte,
                                         uint16_t *mismatch_bit)
{
  uint8_t i;
  uint8_t compare_len = (tx_len < rx_len) ? tx_len : rx_len;

  for (i = 0U; i < compare_len; ++i)
  {
    if (tx_payload[i] != rx_payload[i])
    {
      uint8_t diff = (uint8_t)(tx_payload[i] ^ rx_payload[i]);
      uint8_t b;
      for (b = 0U; b < 8U; ++b)
      {
        if ((diff & (uint8_t)(1U << (7U - b))) != 0U)
        {
          *mismatch_byte = i;
          *mismatch_bit = (uint16_t)((uint16_t)i * 8U + b);
          return 1U;
        }
      }
    }
  }

  if (tx_len != rx_len)
  {
    *mismatch_byte = compare_len;
    *mismatch_bit = (uint16_t)((uint16_t)compare_len * 8U);
    return 1U;
  }

  return 0U;
}

static void app_print_tx_frame_log(void)
{
  uint8_t frame[APP_FRAME_BUFFER_SIZE];
  uint8_t frame_len = 0U;
  uint32_t tx_frame_id = 0U;
  uint8_t payload_len;

  if (vlc_tx_get_frame_snapshot(frame, sizeof(frame), &frame_len, &tx_frame_id) == 0U)
  {
    return;
  }

  if (frame_len < (APP_FRAME_PREAMBLE_BYTES + 3U))
  {
    return;
  }

  payload_len = frame[APP_FRAME_PREAMBLE_BYTES + 1U];
  if ((uint16_t)(APP_FRAME_PREAMBLE_BYTES + 1U + 1U + payload_len + 1U) > frame_len)
  {
    return;
  }

  bsp_uart_printf("tx_frame tx_frame_id=%lu len=%u payload=", (unsigned long)tx_frame_id, payload_len);
  app_uart_print_hex_bytes(&frame[APP_FRAME_PREAMBLE_BYTES + 2U], payload_len);
  bsp_uart_printf(" checksum=%02X frame=", frame[APP_FRAME_PREAMBLE_BYTES + 2U + payload_len]);
  app_uart_print_hex_bytes(frame, frame_len);
  bsp_uart_printf("\r\n");
}

static void app_print_expected_tx_frame_log(void)
{
  uint8_t frame[APP_FRAME_BUFFER_SIZE];
  uint8_t frame_len = 0U;
  uint8_t payload_len;

  if (app_get_expected_frame(frame, &frame_len) == 0U)
  {
    return;
  }

  if (frame_len < (APP_FRAME_PREAMBLE_BYTES + 3U))
  {
    return;
  }

  payload_len = frame[APP_FRAME_PREAMBLE_BYTES + 1U];
  if ((uint16_t)(APP_FRAME_PREAMBLE_BYTES + 1U + 1U + payload_len + 1U) > frame_len)
  {
    return;
  }

  bsp_uart_printf("expected_tx_frame len=%u payload=", payload_len);
  app_uart_print_hex_bytes(&frame[APP_FRAME_PREAMBLE_BYTES + 2U], payload_len);
  bsp_uart_printf(" checksum=%02X frame=", frame[APP_FRAME_PREAMBLE_BYTES + 2U + payload_len]);
  app_uart_print_hex_bytes(frame, frame_len);
  bsp_uart_printf("\r\n");
}

static void app_print_rx_frame_log(const app_frame_t *frame, uint32_t rx_frame_id)
{
  uint8_t full_frame[APP_FRAME_BUFFER_SIZE];
  uint8_t idx = 0U;
  uint8_t i;

  for (i = 0U; i < APP_FRAME_PREAMBLE_BYTES; ++i)
  {
    full_frame[idx++] = APP_FRAME_PREAMBLE_BYTE;
  }
  full_frame[idx++] = APP_FRAME_SYNC_BYTE;
  full_frame[idx++] = frame->length;
  for (i = 0U; i < frame->length; ++i)
  {
    full_frame[idx++] = frame->payload[i];
  }
  full_frame[idx++] = frame->checksum;

  bsp_uart_printf("rx_frame rx_frame_id=%lu len=%u payload=", (unsigned long)rx_frame_id, frame->length);
  app_uart_print_hex_bytes(frame->payload, frame->length);
  bsp_uart_printf(" checksum=%02X frame=", frame->checksum);
  app_uart_print_hex_bytes(full_frame, idx);
  bsp_uart_printf("\r\n");
}

static void app_print_error_bits(const uint8_t *tx_frame,
                                 uint8_t tx_len,
                                 const uint8_t *rx_frame,
                                 uint8_t rx_len,
                                 uint32_t frame_id)
{
  uint16_t bit_count = (tx_len < rx_len) ? (uint16_t)(tx_len * 8U) : (uint16_t)(rx_len * 8U);
  uint16_t bit;
  uint8_t started = 0U;

  bsp_uart_printf("err_bits frame_id=%lu tx_bits=", (unsigned long)frame_id);
  for (bit = 0U; bit < bit_count; ++bit)
  {
    uint8_t b = (uint8_t)((tx_frame[bit >> 3] >> (7U - (bit & 0x07U))) & 0x01U);
    bsp_uart_printf("%u", b);
  }

  bsp_uart_printf(" rx_bits=");
  for (bit = 0U; bit < bit_count; ++bit)
  {
    uint8_t b = (uint8_t)((rx_frame[bit >> 3] >> (7U - (bit & 0x07U))) & 0x01U);
    bsp_uart_printf("%u", b);
  }

  bsp_uart_printf(" mismatch_positions=");
  for (bit = 0U; bit < bit_count; ++bit)
  {
    uint8_t txb = (uint8_t)((tx_frame[bit >> 3] >> (7U - (bit & 0x07U))) & 0x01U);
    uint8_t rxb = (uint8_t)((rx_frame[bit >> 3] >> (7U - (bit & 0x07U))) & 0x01U);
    if (txb != rxb)
    {
      if (started != 0U)
      {
        bsp_uart_printf(",");
      }
      bsp_uart_printf("%u", bit);
      started = 1U;
    }
  }
  bsp_uart_printf("\r\n");
}

static uint32_t count_bit_errors_u8(uint8_t a, uint8_t b)
{
  uint8_t v = (uint8_t)(a ^ b);
  uint32_t c = 0U;

  while (v != 0U)
  {
    c += (uint32_t)(v & 0x01U);
    v >>= 1;
  }

  return c;
}

// This is payload BER over valid decoded frames only.
// Bit errors in invalid/checksum-failed frames are not directly observable in this simple protocol.
static void app_update_link_stats_from_frame(const app_frame_t *frame)
{
  uint32_t compare_len;
  uint32_t i;
  uint32_t bit_errors = 0U;
  uint32_t frame_len;
  uint8_t expected_len_u8 = 0U;
  const uint8_t *expected_payload = 0;
  uint32_t expected_len;

  if (frame == 0)
  {
    return;
  }

  app_get_expected_payload(&expected_payload, &expected_len_u8);
  if (expected_payload == 0)
  {
    return;
  }

  frame_len = frame->length;
  expected_len = expected_len_u8;
  compare_len = (frame_len < expected_len) ? frame_len : expected_len;

  for (i = 0U; i < compare_len; ++i)
  {
    bit_errors += count_bit_errors_u8(frame->payload[i], expected_payload[i]);
  }

  if (frame_len > expected_len)
  {
    bit_errors += (frame_len - expected_len) * 8U;
  }
  else if (expected_len > frame_len)
  {
    bit_errors += (expected_len - frame_len) * 8U;
  }

  s_link_stats.total_payload_bits_checked += ((frame_len > expected_len) ? frame_len : expected_len) * 8U;
  s_link_stats.total_payload_bit_errors += bit_errors;

  if ((frame->length == expected_len_u8) && (bit_errors == 0U))
  {
    s_link_stats.good_payload_frames++;
  }
  else
  {
    s_link_stats.payload_mismatch_frames++;
  }
}

static uint32_t app_calc_ber_ppm(void)
{
  if (s_link_stats.total_payload_bits_checked == 0U)
  {
    return 0U;
  }

  return (uint32_t)(((uint64_t)s_link_stats.total_payload_bit_errors * APP_LINK_STATS_PPM_SCALE) /
                    (uint64_t)s_link_stats.total_payload_bits_checked);
}

static uint32_t app_calc_per_ppm(uint32_t rx_frames, uint32_t frame_errors, uint32_t checksum_errors)
{
  uint32_t total = rx_frames + frame_errors + checksum_errors;
  if (total == 0U)
  {
    return 0U;
  }

  return (uint32_t)((((uint64_t)(frame_errors + checksum_errors)) * APP_LINK_STATS_PPM_SCALE) / (uint64_t)total);
}

// Enforce bring-up critical GPIO/TIM settings without editing CubeMX core blocks.
static void enforce_runtime_hw_settings(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = PIN_TX_READY_Pin | PIN_RX_READY_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PIN_TX_READY_GPIO_Port, &gpio);

  if (s_htim2 != 0)
  {
    __HAL_TIM_SET_AUTORELOAD(s_htim2, APP_TIM2_ARR_100KHZ);
    __HAL_TIM_SET_COUNTER(s_htim2, 0U);
  }
}

// Initialize application modules and bind HAL peripherals.
void app_tasks_init(UART_HandleTypeDef *huart2,
                    TIM_HandleTypeDef *htim1,
                    TIM_HandleTypeDef *htim2,
                    TIM_HandleTypeDef *htim4,
                    ADC_HandleTypeDef *hadc1,
                    DAC_HandleTypeDef *hdac)
{
  s_htim2 = htim2;
  s_tick_count = 0U;
  s_have_last_rx = 0U;
  s_link_stats.total_payload_bits_checked = 0U;
  s_link_stats.total_payload_bit_errors = 0U;
  s_link_stats.payload_mismatch_frames = 0U;
  s_link_stats.good_payload_frames = 0U;

  enforce_runtime_hw_settings();

  bsp_uart_init(huart2);
  bsp_rtc_init();
  bsp_adc_monitor_init(hadc1);
  bsp_dac_threshold_init(hdac);
  vlc_tx_init(htim1);
  vlc_rx_init(htim4);
}

// Start runtime services needed for TX/RX operation.
void app_tasks_start(void)
{
  bsp_dac_threshold_set_mv(APP_DAC_DEFAULT_THRESHOLD_MV);
  bsp_adc_monitor_start();

#if (APP_BOARD_ROLE == APP_BOARD_ROLE_TX_ONLY)
  vlc_tx_start();
#elif (APP_BOARD_ROLE == APP_BOARD_ROLE_RX_ONLY)
  vlc_rx_start();
#else
  vlc_tx_start();
  vlc_rx_start();
#endif

  app_print_startup_role_log();

  if (s_htim2 != 0)
  {
    HAL_TIM_Base_Start_IT(s_htim2);
  }
}

// Handle one TIM2 period event in ISR context.
void app_tasks_on_timer_period_elapsed(TIM_HandleTypeDef *htim)
{
  if ((htim == 0) || (htim != s_htim2))
  {
    return;
  }

  s_tick_count++;

#if (APP_BOARD_ROLE == APP_BOARD_ROLE_TX_ONLY)
  vlc_tx_on_bit_tick();
#elif (APP_BOARD_ROLE == APP_BOARD_ROLE_RX_ONLY)
  vlc_rx_on_bit_tick();
#else
  vlc_rx_on_bit_tick();
  vlc_tx_on_bit_tick();
#endif
}

// Run foreground tasks: consume RX queue and print periodic logs.
void app_tasks_process(void)
{
  static uint32_t last_log_ms = 0xFFFFFFFFUL;
  static uint32_t last_tx_frame_log_id = 0xFFFFFFFFUL;
  static uint32_t err_events_queued = 0U;
  static uint32_t err_events_printed = 0U;
  static uint32_t err_events_suppressed = 0U;
  static uint32_t err_bits_printed = 0U;
  vlc_rx_stats_t rx_stats;
  vlc_rx_error_event_t err_event;
  bsp_adc_voltages_t volts;
  uint32_t now;
  uint32_t ber_ppm;
  uint32_t per_ppm;
  uint32_t rx_total_observed;
  app_frame_t frame;
  uint32_t rx_frame_id;
  app_frame_t last_frame_for_log;
  uint32_t last_frame_id_for_log = 0U;
  uint8_t have_new_rx_frame = 0U;
  uint8_t tx_frame[APP_FRAME_BUFFER_SIZE];
  uint8_t tx_frame_len = 0U;
  uint32_t tx_frame_id = 0U;
  uint8_t i;

  bsp_rtc_process();
  app_handle_uart_rx();

  while (vlc_rx_pop_frame(&frame, &rx_frame_id) != 0U)
  {
    app_update_link_stats_from_frame(&frame);
    s_last_rx = frame;
    s_have_last_rx = 1U;
    last_frame_for_log = frame;
    last_frame_id_for_log = rx_frame_id;
    have_new_rx_frame = 1U;
  }

  while (vlc_rx_pop_error_event(&err_event) != 0U)
  {
#if (APP_BOARD_ROLE != APP_BOARD_ROLE_TX_ONLY)
    uint8_t expected_frame[APP_FRAME_BUFFER_SIZE];
    uint8_t expected_frame_len = 0U;
    uint8_t expected_payload_len = 0U;
    const uint8_t *expected_payload = 0;
    err_events_queued++;

    if (app_get_expected_frame(expected_frame, &expected_frame_len) == 0U)
    {
      err_events_suppressed++;
      continue;
    }

    app_get_expected_payload(&expected_payload, &expected_payload_len);
    if ((expected_payload == 0) || (err_events_printed >= APP_ERR_LOG_MAX_PER_PERIOD))
    {
      err_events_suppressed++;
      continue;
    }

    if (err_event.type == VLC_RX_ERR_TYPE_CHECKSUM)
    {
      uint8_t tx_payload_len = expected_payload_len;
      const uint8_t *tx_payload = expected_payload;
      uint8_t tx_checksum = expected_frame[APP_FRAME_PREAMBLE_BYTES + 2U + expected_payload_len];
      uint8_t mismatch_byte = 0U;
      uint16_t mismatch_bit = 0U;
      const char *mismatch_field;
      uint8_t rx_frame_full[APP_FRAME_BUFFER_SIZE];
      uint8_t rx_frame_full_len = 0U;
      uint8_t j;
      uint8_t payload_mismatch;

      for (j = 0U; j < APP_FRAME_PREAMBLE_BYTES; ++j)
      {
        rx_frame_full[rx_frame_full_len++] = APP_FRAME_PREAMBLE_BYTE;
      }
      rx_frame_full[rx_frame_full_len++] = APP_FRAME_SYNC_BYTE;
      rx_frame_full[rx_frame_full_len++] = err_event.rx_payload_len;
      for (j = 0U; j < err_event.rx_payload_len; ++j)
      {
        rx_frame_full[rx_frame_full_len++] = err_event.rx_payload[j];
      }
      rx_frame_full[rx_frame_full_len++] = err_event.rx_checksum;

      payload_mismatch = app_find_payload_mismatch(tx_payload,
                                                   tx_payload_len,
                                                   err_event.rx_payload,
                                                   err_event.rx_payload_len,
                                                   &mismatch_byte,
                                                   &mismatch_bit);
      if (payload_mismatch != 0U)
      {
        mismatch_field = "payload";
      }
      else
      {
        mismatch_field = "checksum";
        mismatch_byte = 255U;
        mismatch_bit = 65535U;
      }

      bsp_uart_printf("err_frame type=checksum tx_frame_id=%lu rx_frame_id=%lu state=%s tx_payload=",
                      (unsigned long)err_event.tx_frame_id,
                      (unsigned long)err_event.rx_frame_id,
                      app_rx_state_to_text(err_event.state));
      app_uart_print_hex_bytes(tx_payload, tx_payload_len);
      bsp_uart_printf(" rx_payload=");
      app_uart_print_hex_bytes(err_event.rx_payload, err_event.rx_payload_len);
      bsp_uart_printf(" tx_checksum=%02X rx_checksum=%02X mismatch_field=%s mismatch_byte=%u mismatch_bit=%u\r\n",
                      tx_checksum,
                      err_event.rx_checksum,
                      mismatch_field,
                      mismatch_byte,
                      mismatch_bit);
      err_events_printed++;

#if (APP_ERR_BITS_ENABLE == 1U)
      if (err_bits_printed < APP_ERR_BITS_MAX_PER_PERIOD)
      {
        app_print_error_bits(expected_frame, expected_frame_len, rx_frame_full, rx_frame_full_len, err_event.rx_frame_id);
        err_bits_printed++;
      }
#endif
    }
    else
    {
      bsp_uart_printf("err_frame type=frame reason=%s tx_frame_id=%lu rx_frame_id=%lu state=%s",
                      app_err_reason_to_text(err_event.reason),
                      (unsigned long)err_event.tx_frame_id,
                      (unsigned long)err_event.rx_frame_id,
                      app_rx_state_to_text(err_event.state));
      if (err_event.has_expected_received != 0U)
      {
        bsp_uart_printf(" expected=%02X received=%02X", err_event.expected, err_event.received);
      }
      bsp_uart_printf("\r\n");
      err_events_printed++;
    }
#endif
  }

  now = HAL_GetTick();
  if ((last_log_ms != 0xFFFFFFFFUL) && ((now - last_log_ms) < APP_LOG_PERIOD_MS))
  {
    return;
  }
  last_log_ms = now;

  vlc_rx_get_stats(&rx_stats);
  bsp_adc_monitor_get_voltages(&volts);
  ber_ppm = app_calc_ber_ppm();
  rx_total_observed = rx_stats.rx_frames + rx_stats.frame_errors + rx_stats.checksum_errors;
  per_ppm = app_calc_per_ppm(rx_stats.rx_frames, rx_stats.frame_errors, rx_stats.checksum_errors);

  bsp_uart_printf("role board_role=%s\r\n", app_board_role_to_text());

#if (APP_BOARD_ROLE == APP_BOARD_ROLE_TX_ONLY)
  bsp_uart_printf("alive_tx tx_frames=%lu tx_frame_id=%lu bit_rate=%u tx_enabled=%u carrier_test=%u\r\n",
                  (unsigned long)vlc_tx_get_frame_count(),
                  (unsigned long)vlc_tx_get_frame_count(),
                  (unsigned int)APP_BIT_RATE_HZ,
                  (unsigned int)vlc_tx_is_enabled(),
                  (unsigned int)vlc_tx_is_carrier_test());
#elif (APP_BOARD_ROLE == APP_BOARD_ROLE_RX_ONLY)
  bsp_uart_printf("alive_rx rx_frames=%lu frame_errors=%lu checksum_errors=%lu rx_sync_state=%u last_edge_delta=%u last_raw_bit=%u\r\n",
                  (unsigned long)rx_stats.rx_frames,
                  (unsigned long)rx_stats.frame_errors,
                  (unsigned long)rx_stats.checksum_errors,
                  (unsigned int)rx_sync_get_state(),
                  rx_stats.last_edge_delta,
                  rx_stats.last_raw_bit);
  app_print_expected_tx_frame_log();
#else
  bsp_uart_printf("alive tx_frames=%lu rx_frames=%lu frame_errors=%lu checksum_errors=%lu rx_sync_state=%u last_edge_delta=%u last_raw_bit=%u\r\n",
                  (unsigned long)vlc_tx_get_frame_count(),
                  (unsigned long)rx_stats.rx_frames,
                  (unsigned long)rx_stats.frame_errors,
                  (unsigned long)rx_stats.checksum_errors,
                  (unsigned int)rx_sync_get_state(),
                  rx_stats.last_edge_delta,
                  rx_stats.last_raw_bit);
#endif

#if (APP_BOARD_ROLE != APP_BOARD_ROLE_RX_ONLY)
  if (vlc_tx_get_frame_snapshot(tx_frame, sizeof(tx_frame), &tx_frame_len, &tx_frame_id) != 0U)
  {
    if ((tx_frame_id != last_tx_frame_log_id) || (last_tx_frame_log_id == 0xFFFFFFFFUL))
    {
      app_print_tx_frame_log();
      last_tx_frame_log_id = tx_frame_id;
    }
  }
#endif

#if (APP_BOARD_ROLE != APP_BOARD_ROLE_TX_ONLY)
  if (have_new_rx_frame != 0U)
  {
    app_print_rx_frame_log(&last_frame_for_log, last_frame_id_for_log);
  }

  if (s_have_last_rx != 0U)
  {
    bsp_uart_printf("last_rx len=%u payload=", s_last_rx.length);
    for (i = 0U; i < s_last_rx.length; ++i)
    {
      bsp_uart_printf("%02X", s_last_rx.payload[i]);
      if (i + 1U < s_last_rx.length)
      {
        bsp_uart_printf(" ");
      }
    }
    bsp_uart_printf("\r\n");
  }
  else
  {
    bsp_uart_printf("last_rx none\r\n");
  }
#endif

#if (APP_BOARD_ROLE != APP_BOARD_ROLE_TX_ONLY)
  bsp_uart_printf("link_stats payload_bits=%lu payload_bit_errors=%lu payload_ber_ppm=%lu invalid_frames=%lu payload_mismatch_frames=%lu good_payload_frames=%lu\r\n",
                  (unsigned long)s_link_stats.total_payload_bits_checked,
                  (unsigned long)s_link_stats.total_payload_bit_errors,
                  (unsigned long)ber_ppm,
                  (unsigned long)(rx_stats.frame_errors + rx_stats.checksum_errors),
                  (unsigned long)s_link_stats.payload_mismatch_frames,
                  (unsigned long)s_link_stats.good_payload_frames);
  bsp_uart_printf("link_quality rx_total_observed=%lu per_ppm=%lu\r\n",
                  (unsigned long)rx_total_observed,
                  (unsigned long)per_ppm);
  bsp_uart_printf("err_summary queued=%lu printed=%lu suppressed=%lu\r\n",
                  (unsigned long)err_events_queued,
                  (unsigned long)err_events_printed,
                  (unsigned long)err_events_suppressed);
#endif

  bsp_uart_printf("adc_mv rx_out_a=%u vmon_bu=%u vmon_bu_3v=%u vmon_main_sys=%u vmon_main=%u vmon_5v_sys=%u vmon_3v3_sys=%u\r\n",
                  volts.rx_out_a_mv,
                  volts.vmon_bu_mv,
                  volts.vmon_bu_3v_mv,
                  volts.vmon_main_sys_mv,
                  volts.vmon_main_mv,
                  volts.vmon_5v_sys_mv,
                  volts.vmon_3v3_sys_mv);
  bsp_uart_printf("dac_mv threshold=%u\r\n", bsp_dac_threshold_get_mv());
  bsp_rtc_print_log();

#if (APP_BOARD_ROLE != APP_BOARD_ROLE_TX_ONLY)
  err_events_queued = 0U;
  err_events_printed = 0U;
  err_events_suppressed = 0U;
  err_bits_printed = 0U;
#endif
}
