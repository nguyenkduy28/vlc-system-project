#ifndef VLC_RX_H
#define VLC_RX_H

#include <stdint.h>
#include "app_config.h"
#include "app_protocol.h"
#include "stm32f4xx_hal.h"

typedef struct
{
  uint32_t rx_frames;
  uint32_t frame_errors;
  uint32_t checksum_errors;
  uint16_t last_edge_delta;
  uint8_t last_raw_bit;
} vlc_rx_stats_t;

typedef enum
{
  VLC_RX_ERR_TYPE_FRAME = 0,
  VLC_RX_ERR_TYPE_CHECKSUM
} vlc_rx_error_type_t;

typedef struct
{
  uint8_t valid;
  vlc_rx_error_type_t type;
  uint8_t reason;
  uint8_t state;
  uint8_t expected;
  uint8_t received;
  uint8_t has_expected_received;
  uint8_t rx_payload_len;
  uint8_t rx_payload[APP_MAX_PAYLOAD_LEN];
  uint8_t rx_checksum;
  uint8_t has_rx_checksum;
  uint32_t tx_frame_id;
  uint32_t rx_frame_id;
} vlc_rx_error_event_t;

// Initialize RX queue, counters, and sync FSM.
void vlc_rx_init(TIM_HandleTypeDef *htim4);
// Start TIM4 based edge-counter RX frontend.
void vlc_rx_start(void);
// Process one RX bit window using TIM4 edge delta.
void vlc_rx_on_bit_tick(void);
// Pop one decoded frame payload from RX queue.
uint8_t vlc_rx_pop_payload(app_frame_t *frame);
// Pop next decoded frame plus its monotonically increasing RX frame ID.
uint8_t vlc_rx_pop_frame(app_frame_t *frame, uint32_t *rx_frame_id);
// Copy current RX statistics counters.
void vlc_rx_get_stats(vlc_rx_stats_t *stats);
// Pop one deferred RX error event captured in ISR context.
uint8_t vlc_rx_pop_error_event(vlc_rx_error_event_t *event);

#endif
