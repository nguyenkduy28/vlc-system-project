#include "vlc_tx.h"
#include "app_protocol.h"
#include "app_config.h"

#define TX_FRAME_BUFFER_SIZE  (APP_FRAME_PREAMBLE_BYTES + 1U + 1U + APP_MAX_PAYLOAD_LEN + 1U)

static TIM_HandleTypeDef *s_htim1;
static uint8_t s_frame[TX_FRAME_BUFFER_SIZE];
static uint16_t s_frame_len;
static uint16_t s_bit_index;
static uint32_t s_frames_sent;
static uint8_t s_tx_enabled;
static uint8_t s_carrier_test_mode;
static uint8_t s_single_frame_mode;

// Apply current TX bit by gating TIM1 PWM duty.
static void vlc_tx_apply_bit(uint8_t bit)
{
  if (s_htim1 == 0)
  {
    return;
  }

  __HAL_TIM_SET_COMPARE(s_htim1, TIM_CHANNEL_1, (bit != 0U) ? APP_TIM1_CARRIER_CCR_ON : APP_TIM1_CARRIER_CCR_OFF);
}

// Prepare TX frame generator state.
void vlc_tx_init(TIM_HandleTypeDef *htim1)
{
  uint8_t payload_len = 0U;
  const uint8_t *payload = app_protocol_get_expected_payload(&payload_len);

  s_htim1 = htim1;
  s_frame_len = app_protocol_build_frame(s_frame, sizeof(s_frame), payload, payload_len);
  s_bit_index = 0U;
  s_frames_sent = 0U;
  s_tx_enabled = 1U;
  s_carrier_test_mode = 0U;
  s_single_frame_mode = 0U;
}

// Enable PWM output for optical carrier.
void vlc_tx_start(void)
{
  if ((s_htim1 == 0) || (s_frame_len == 0U))
  {
    return;
  }

  HAL_TIM_PWM_Start(s_htim1, TIM_CHANNEL_1);
  vlc_tx_apply_bit(0U);
}

// Shift out one MSB-first frame bit per timing tick.
void vlc_tx_on_bit_tick(void)
{
  uint16_t byte_index;
  uint8_t bit_pos;
  uint8_t bit;
  uint16_t total_bits;

  if (s_carrier_test_mode != 0U)
  {
    vlc_tx_apply_bit(1U);
    return;
  }

  if ((s_tx_enabled == 0U) && (s_single_frame_mode == 0U))
  {
    vlc_tx_apply_bit(0U);
    return;
  }

  if (s_frame_len == 0U)
  {
    vlc_tx_apply_bit(0U);
    return;
  }

  total_bits = (uint16_t)(s_frame_len * 8U);
  byte_index = (uint16_t)(s_bit_index >> 3);
  bit_pos = (uint8_t)(7U - (s_bit_index & 0x07U));
  bit = (uint8_t)((s_frame[byte_index] >> bit_pos) & 0x01U);
  vlc_tx_apply_bit(bit);

  s_bit_index++;
  if (s_bit_index >= total_bits)
  {
    s_bit_index = 0U;
    s_frames_sent++;

    if (s_single_frame_mode != 0U)
    {
      s_single_frame_mode = 0U;
      s_tx_enabled = 0U;
      vlc_tx_apply_bit(0U);
    }
  }
}

// Replace the active TX frame atomically relative to the TIM2 ISR.
uint8_t vlc_tx_set_payload(const uint8_t *payload, uint8_t len)
{
  uint8_t new_frame[TX_FRAME_BUFFER_SIZE];
  uint16_t new_len;
  uint16_t i;

  if ((payload == 0) || (len == 0U) || (len > APP_MAX_PAYLOAD_LEN))
  {
    return 0U;
  }

  new_len = app_protocol_build_frame(new_frame, sizeof(new_frame), payload, len);
  if ((new_len == 0U) || (new_len > TX_FRAME_BUFFER_SIZE))
  {
    return 0U;
  }

  __disable_irq();
  for (i = 0U; i < new_len; ++i)
  {
    s_frame[i] = new_frame[i];
  }
  s_frame_len = new_len;
  s_bit_index = 0U;
  __enable_irq();
  return 1U;
}

// Control continuous frame transmission state.
void vlc_tx_set_enabled(uint8_t enabled)
{
  __disable_irq();
  s_carrier_test_mode = 0U;
  s_single_frame_mode = 0U;
  s_tx_enabled = (enabled != 0U) ? 1U : 0U;
  s_bit_index = 0U;
  __enable_irq();

  if (enabled == 0U)
  {
    vlc_tx_apply_bit(0U);
  }
}

// Control direct carrier test mode.
void vlc_tx_set_carrier_test(uint8_t enabled)
{
  __disable_irq();
  s_carrier_test_mode = (enabled != 0U) ? 1U : 0U;
  s_single_frame_mode = 0U;
  s_tx_enabled = 0U;
  s_bit_index = 0U;
  __enable_irq();

  vlc_tx_apply_bit((enabled != 0U) ? 1U : 0U);
}

// Arm one complete frame transmission from the next bit tick.
void vlc_tx_send_single_frame(void)
{
  __disable_irq();
  s_carrier_test_mode = 0U;
  s_single_frame_mode = 1U;
  s_tx_enabled = 0U;
  s_bit_index = 0U;
  __enable_irq();
}

// Read current TX enabled state.
uint8_t vlc_tx_is_enabled(void)
{
  return s_tx_enabled;
}

// Read current carrier test state.
uint8_t vlc_tx_is_carrier_test(void)
{
  return s_carrier_test_mode;
}

// Read serialized frame length.
uint16_t vlc_tx_get_frame_len(void)
{
  return s_frame_len;
}

// Read serialized frame buffer pointer.
const uint8_t *vlc_tx_get_frame_ptr(void)
{
  return s_frame;
}

// Return total number of transmitted frames.
uint32_t vlc_tx_get_frame_count(void)
{
  return s_frames_sent;
}

// Copy immutable TX frame buffer plus current completed-frame counter.
uint8_t vlc_tx_get_frame_snapshot(uint8_t *dst, uint8_t dst_size, uint8_t *out_len, uint32_t *out_frame_id)
{
  uint8_t i;
  uint8_t len8;

  if ((dst == 0) || (out_len == 0) || (out_frame_id == 0))
  {
    return 0U;
  }

  len8 = (uint8_t)s_frame_len;
  if ((s_frame_len == 0U) || (dst_size < len8))
  {
    return 0U;
  }

  for (i = 0U; i < len8; ++i)
  {
    dst[i] = s_frame[i];
  }

  *out_len = len8;
  *out_frame_id = s_frames_sent;
  return 1U;
}
