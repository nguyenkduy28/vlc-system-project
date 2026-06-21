#ifndef RX_SYNC_H
#define RX_SYNC_H

#include <stdint.h>
#include "app_protocol.h"

typedef enum
{
  RX_SYNC_WAIT_ACTIVITY = 0,
  RX_SYNC_DETECT_PREAMBLE,
  RX_SYNC_LOCK_BIT_TIMING,
  RX_SYNC_READ_SYNC,
  RX_SYNC_READ_LEN,
  RX_SYNC_READ_PAYLOAD,
  RX_SYNC_READ_CHECKSUM
} rx_sync_state_t;

typedef struct
{
  uint8_t has_frame;
  app_frame_t frame;
  uint8_t checksum_error;
  uint8_t frame_error;
  uint8_t error_reason;
  uint8_t error_state;
  uint8_t expected;
  uint8_t received;
  uint8_t has_expected_received;
  uint8_t rx_checksum;
  uint8_t has_rx_checksum;
  uint8_t rx_payload_len;
  uint8_t rx_payload[APP_MAX_PAYLOAD_LEN];
} rx_sync_result_t;

#define RX_SYNC_ERROR_NONE        0U
#define RX_SYNC_ERROR_SYNC        1U
#define RX_SYNC_ERROR_LENGTH      2U
#define RX_SYNC_ERROR_PREAMBLE    3U
#define RX_SYNC_ERROR_OTHER       4U

// Reset RX sync state machine to initial idle state.
void rx_sync_init(void);
// Process one sampled RX bit and update frame-decode result.
void rx_sync_on_tick(uint8_t raw_bit, rx_sync_result_t *result);
// Read current RX sync FSM state.
rx_sync_state_t rx_sync_get_state(void);

#endif
