#include "rx_sync.h"
#include "app_config.h"

static rx_sync_state_t s_state;
static uint8_t s_shift;
static uint8_t s_bit_count;
static uint32_t s_preamble_shift;
static app_frame_t s_frame;
static uint8_t s_payload_index;

// Shift one bit into preamble detector and check 0xAA 0xAA 0xAA pattern.
static uint8_t rx_sync_process_preamble_bit(uint8_t raw_bit)
{
  s_preamble_shift = ((s_preamble_shift << 1) | ((uint32_t)raw_bit & 0x01U)) & 0x00FFFFFFU;
  return (s_preamble_shift == 0x00AAAAAAU) ? 1U : 0U;
}

// Clear frame parsing accumulators before a new decode.
static void rx_sync_reset_frame(void)
{
  s_shift = 0U;
  s_bit_count = 0U;
  s_preamble_shift = 0U;
  s_payload_index = 0U;
  s_frame.length = 0U;
  s_frame.checksum = 0U;
}

// Reset byte assembly state while keeping FSM state.
static void rx_sync_reset_byte_assembly(void)
{
  s_shift = 0U;
  s_bit_count = 0U;
}

// Shift in one bit and report completed byte when available.
static uint8_t rx_sync_push_bit(uint8_t raw_bit, uint8_t *out_byte)
{
  s_shift = (uint8_t)((s_shift << 1) | (raw_bit & 0x01U));
  s_bit_count++;

  if (s_bit_count >= 8U)
  {
    s_bit_count = 0U;
    *out_byte = s_shift;
    return 1U;
  }

  return 0U;
}

// Return to idle activity search state.
static void rx_sync_go_wait_activity(void)
{
  s_state = RX_SYNC_WAIT_ACTIVITY;
  rx_sync_reset_frame();
}

// Initialize frame-sync state.
void rx_sync_init(void)
{
  s_state = RX_SYNC_WAIT_ACTIVITY;
  rx_sync_reset_frame();
}

// Get current state for debug/inspection.
rx_sync_state_t rx_sync_get_state(void)
{
  return s_state;
}

// Consume one bit window and advance RX synchronization FSM.
void rx_sync_on_tick(uint8_t raw_bit, rx_sync_result_t *result)
{
  uint8_t value = 0U;
  uint8_t byte_ready = 0U;
  result->has_frame = 0U;
  result->checksum_error = 0U;
  result->frame_error = 0U;
  result->error_reason = RX_SYNC_ERROR_NONE;
  result->error_state = (uint8_t)s_state;
  result->expected = 0U;
  result->received = 0U;
  result->has_expected_received = 0U;
  result->rx_checksum = 0U;
  result->has_rx_checksum = 0U;
  result->rx_payload_len = 0U;

  switch (s_state)
  {
    case RX_SYNC_WAIT_ACTIVITY:
      if (raw_bit != 0U)
      {
        rx_sync_reset_frame();
        s_state = RX_SYNC_DETECT_PREAMBLE;
      }
      else
      {
        return;
      }
      /* no break */

    case RX_SYNC_DETECT_PREAMBLE:
      if (rx_sync_process_preamble_bit(raw_bit) != 0U)
      {
        s_state = RX_SYNC_LOCK_BIT_TIMING;
        rx_sync_reset_byte_assembly();
        s_state = RX_SYNC_READ_SYNC;
      }
      return;

    case RX_SYNC_LOCK_BIT_TIMING:
      s_state = RX_SYNC_READ_SYNC;
      break;

    case RX_SYNC_READ_SYNC:
    case RX_SYNC_READ_LEN:
    case RX_SYNC_READ_PAYLOAD:
    case RX_SYNC_READ_CHECKSUM:
      byte_ready = rx_sync_push_bit(raw_bit, &value);
      if (byte_ready == 0U)
      {
        return;
      }
      break;

    default:
      rx_sync_go_wait_activity();
      return;
  }

  switch (s_state)
  {
    case RX_SYNC_READ_SYNC:
      if (value == APP_FRAME_SYNC_BYTE)
      {
        s_state = RX_SYNC_READ_LEN;
      }
      else
      {
        result->frame_error = 1U;
        result->error_reason = RX_SYNC_ERROR_SYNC;
        result->error_state = RX_SYNC_READ_SYNC;
        result->expected = APP_FRAME_SYNC_BYTE;
        result->received = value;
        result->has_expected_received = 1U;
        rx_sync_go_wait_activity();
      }
      break;

    case RX_SYNC_READ_LEN:
      if ((value == 0U) || (value > APP_MAX_PAYLOAD_LEN))
      {
        result->frame_error = 1U;
        result->error_reason = RX_SYNC_ERROR_LENGTH;
        result->error_state = RX_SYNC_READ_LEN;
        result->expected = APP_MAX_PAYLOAD_LEN;
        result->received = value;
        result->has_expected_received = 1U;
        rx_sync_go_wait_activity();
      }
      else
      {
        s_frame.length = value;
        s_payload_index = 0U;
        s_state = RX_SYNC_READ_PAYLOAD;
      }
      break;

    case RX_SYNC_READ_PAYLOAD:
      s_frame.payload[s_payload_index++] = value;
      if (s_payload_index >= s_frame.length)
      {
        s_state = RX_SYNC_READ_CHECKSUM;
      }
      break;

    case RX_SYNC_READ_CHECKSUM:
      s_frame.checksum = value;
      result->rx_checksum = value;
      result->has_rx_checksum = 1U;
      result->rx_payload_len = s_frame.length;
      {
        uint8_t i;
        for (i = 0U; i < s_frame.length; ++i)
        {
          result->rx_payload[i] = s_frame.payload[i];
        }
      }
      if (s_frame.checksum == app_protocol_checksum(s_frame.length, s_frame.payload))
      {
        result->has_frame = 1U;
        result->frame = s_frame;
      }
      else
      {
        result->checksum_error = 1U;
      }

      rx_sync_go_wait_activity();
      break;

    default:
      rx_sync_go_wait_activity();
      break;
  }
}
