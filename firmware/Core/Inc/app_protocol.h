#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>
#include "app_config.h"

typedef struct
{
  uint8_t length;
  uint8_t payload[APP_MAX_PAYLOAD_LEN];
  uint8_t checksum;
} app_frame_t;

// Return pointer to the shared expected payload used by TX and RX evaluation.
const uint8_t *app_protocol_get_expected_payload(uint8_t *length);
// Compute checksum = length + sum(payload) modulo 256.
uint8_t app_protocol_checksum(uint8_t length, const uint8_t *payload);
// Build the shared expected frame used by TX and RX evaluation.
uint16_t app_protocol_get_expected_frame(uint8_t *dst, uint16_t dst_size);
// Build full TX frame: preamble + sync + length + payload + checksum.
uint16_t app_protocol_build_frame(uint8_t *dst, uint16_t dst_size, const uint8_t *payload, uint8_t length);

#endif
