#ifndef VLC_TX_H
#define VLC_TX_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

// Initialize TX module with TIM1 PWM handle.
void vlc_tx_init(TIM_HandleTypeDef *htim1);
// Start TIM1 PWM output and force initial carrier-off state.
void vlc_tx_start(void);
// Send one TX bit on each TIM2 bit tick.
void vlc_tx_on_bit_tick(void);
// Replace the TX payload and rebuild the serialized frame.
uint8_t vlc_tx_set_payload(const uint8_t *payload, uint8_t len);
// Enable or disable continuous frame transmission.
void vlc_tx_set_enabled(uint8_t enabled);
// Force continuous carrier test mode on or off.
void vlc_tx_set_carrier_test(uint8_t enabled);
// Send exactly one frame, then stop with carrier off.
void vlc_tx_send_single_frame(void);
// Read current TX enabled flag.
uint8_t vlc_tx_is_enabled(void);
// Read current carrier test flag.
uint8_t vlc_tx_is_carrier_test(void);
// Read current serialized frame length.
uint16_t vlc_tx_get_frame_len(void);
// Read pointer to current serialized frame buffer.
const uint8_t *vlc_tx_get_frame_ptr(void);
// Get number of completed TX frames.
uint32_t vlc_tx_get_frame_count(void);
// Copy current TX frame bytes and metadata for foreground logging.
uint8_t vlc_tx_get_frame_snapshot(uint8_t *dst, uint8_t dst_size, uint8_t *out_len, uint32_t *out_frame_id);

#endif
