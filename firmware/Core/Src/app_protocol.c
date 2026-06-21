#include "app_protocol.h"

static const uint8_t s_expected_payload[] = {0x55U, 0xA5U, 0x3CU, 0xC3U};

// Return the shared payload definition used for TX generation and RX comparison.
const uint8_t *app_protocol_get_expected_payload(uint8_t *length)
{
  if (length != 0)
  {
    *length = (uint8_t)sizeof(s_expected_payload);
  }

  return s_expected_payload;
}

// Compute frame checksum using the agreed protocol rule.
uint8_t app_protocol_checksum(uint8_t length, const uint8_t *payload)
{
  uint16_t sum = length;
  uint8_t i;

  for (i = 0U; i < length; ++i)
  {
    sum = (uint16_t)(sum + payload[i]);
  }

  return (uint8_t)(sum & 0xFFU);
}

// Build the shared expected frame from the canonical payload definition.
uint16_t app_protocol_get_expected_frame(uint8_t *dst, uint16_t dst_size)
{
  uint8_t length = 0U;
  const uint8_t *payload = app_protocol_get_expected_payload(&length);
  return app_protocol_build_frame(dst, dst_size, payload, length);
}

// Build serialized frame bytes for TX.
uint16_t app_protocol_build_frame(uint8_t *dst, uint16_t dst_size, const uint8_t *payload, uint8_t length)
{
  uint16_t i;
  uint16_t idx = 0U;

  if ((dst == 0) || (payload == 0) || (length > APP_MAX_PAYLOAD_LEN))
  {
    return 0U;
  }

  if (dst_size < (uint16_t)(APP_FRAME_PREAMBLE_BYTES + 1U + 1U + length + 1U))
  {
    return 0U;
  }

  for (i = 0U; i < APP_FRAME_PREAMBLE_BYTES; ++i)
  {
    dst[idx++] = APP_FRAME_PREAMBLE_BYTE;
  }

  dst[idx++] = APP_FRAME_SYNC_BYTE;
  dst[idx++] = length;

  for (i = 0U; i < length; ++i)
  {
    dst[idx++] = payload[i];
  }

  dst[idx++] = app_protocol_checksum(length, payload);

  return idx;
}
