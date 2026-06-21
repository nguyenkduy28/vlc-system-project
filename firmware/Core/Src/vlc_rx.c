#include "vlc_rx.h"
#include "rx_sync.h"
#include "app_config.h"
#include "vlc_tx.h"

#define VLC_RX_ERR_QUEUE_DEPTH APP_RX_ERROR_EVENT_QUEUE_DEPTH

typedef struct
{
  app_frame_t frame;
  uint32_t rx_frame_id;
} rx_frame_item_t;

typedef struct
{
  rx_frame_item_t items[APP_RX_QUEUE_DEPTH];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
} rx_queue_t;

typedef struct
{
  vlc_rx_error_event_t items[VLC_RX_ERR_QUEUE_DEPTH];
  uint8_t head;
  uint8_t tail;
  uint8_t count;
} rx_error_queue_t;

static TIM_HandleTypeDef *s_htim4;
static uint16_t s_prev_tim4_count;
static rx_queue_t s_queue;
static rx_error_queue_t s_err_queue;
static vlc_rx_stats_t s_stats;
static uint32_t s_rx_observed_frames;

// Push decoded frame into bounded queue (drop oldest on overflow).
static void queue_push(const app_frame_t *frame, uint32_t rx_frame_id)
{
  if (s_queue.count >= APP_RX_QUEUE_DEPTH)
  {
    s_queue.tail = (uint8_t)((s_queue.tail + 1U) % APP_RX_QUEUE_DEPTH);
    s_queue.count--;
  }

  s_queue.items[s_queue.head].frame = *frame;
  s_queue.items[s_queue.head].rx_frame_id = rx_frame_id;
  s_queue.head = (uint8_t)((s_queue.head + 1U) % APP_RX_QUEUE_DEPTH);
  s_queue.count++;
}

// Push error event into bounded queue (drop oldest on overflow).
static void err_queue_push(const vlc_rx_error_event_t *event)
{
  if (s_err_queue.count >= VLC_RX_ERR_QUEUE_DEPTH)
  {
    s_err_queue.tail = (uint8_t)((s_err_queue.tail + 1U) % VLC_RX_ERR_QUEUE_DEPTH);
    s_err_queue.count--;
  }

  s_err_queue.items[s_err_queue.head] = *event;
  s_err_queue.head = (uint8_t)((s_err_queue.head + 1U) % VLC_RX_ERR_QUEUE_DEPTH);
  s_err_queue.count++;
}

// Initialize RX queue, edge-counter context, and sync FSM.
void vlc_rx_init(TIM_HandleTypeDef *htim4)
{
  s_htim4 = htim4;
  s_prev_tim4_count = 0U;

  s_queue.head = 0U;
  s_queue.tail = 0U;
  s_queue.count = 0U;
  s_err_queue.head = 0U;
  s_err_queue.tail = 0U;
  s_err_queue.count = 0U;

  s_stats.rx_frames = 0U;
  s_stats.frame_errors = 0U;
  s_stats.checksum_errors = 0U;
  s_stats.last_edge_delta = 0U;
  s_stats.last_raw_bit = 0U;
  s_rx_observed_frames = 0U;

  rx_sync_init();
}

// Start TIM4 and capture initial edge counter baseline.
void vlc_rx_start(void)
{
#if (APP_RX_USE_EDGE_COUNTER == 1U)
  if (s_htim4 == 0)
  {
    return;
  }

  __HAL_TIM_SET_COUNTER(s_htim4, 0U);
  HAL_TIM_Base_Start(s_htim4);
  s_prev_tim4_count = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim4);
#endif
}

// Process one bit window from TIM4 edge delta and decode frame bits.
void vlc_rx_on_bit_tick(void)
{
  rx_sync_result_t result;
  vlc_rx_error_event_t event;
  uint8_t i;
  uint16_t now_count = 0U;
  uint16_t delta_edges = 0U;
  uint8_t raw_bit = 0U;

#if (APP_RX_USE_EDGE_COUNTER == 1U)
  if (s_htim4 == 0)
  {
    return;
  }

  now_count = (uint16_t)__HAL_TIM_GET_COUNTER(s_htim4);
  delta_edges = (uint16_t)(now_count - s_prev_tim4_count);
  s_prev_tim4_count = now_count;

  if (delta_edges >= APP_RX_EDGE_THRESHOLD)
  {
    raw_bit = 1U;
  }
  else
  {
    raw_bit = 0U;
  }
#else
  return;
#endif

  s_stats.last_edge_delta = delta_edges;
  s_stats.last_raw_bit = raw_bit;

  rx_sync_on_tick(raw_bit, &result);

  if (result.has_frame != 0U)
  {
    s_rx_observed_frames++;
    s_stats.rx_frames++;
    queue_push(&result.frame, s_rx_observed_frames);
  }
  else if (result.frame_error != 0U)
  {
    s_rx_observed_frames++;
    s_stats.frame_errors++;

    event.valid = 1U;
    event.type = VLC_RX_ERR_TYPE_FRAME;
    event.reason = result.error_reason;
    event.state = result.error_state;
    event.expected = result.expected;
    event.received = result.received;
    event.has_expected_received = result.has_expected_received;
    event.rx_payload_len = 0U;
    event.rx_checksum = 0U;
    event.has_rx_checksum = 0U;
    event.tx_frame_id = vlc_tx_get_frame_count();
    event.rx_frame_id = s_rx_observed_frames;
    for (i = 0U; i < APP_MAX_PAYLOAD_LEN; ++i)
    {
      event.rx_payload[i] = 0U;
    }
    err_queue_push(&event);
  }

  else if (result.checksum_error != 0U)
  {
    s_rx_observed_frames++;
    s_stats.checksum_errors++;

    event.valid = 1U;
    event.type = VLC_RX_ERR_TYPE_CHECKSUM;
    event.reason = RX_SYNC_ERROR_NONE;
    event.state = RX_SYNC_READ_CHECKSUM;
    event.expected = 0U;
    event.received = 0U;
    event.has_expected_received = 0U;
    event.rx_payload_len = result.rx_payload_len;
    if (event.rx_payload_len > APP_MAX_PAYLOAD_LEN)
    {
      event.rx_payload_len = APP_MAX_PAYLOAD_LEN;
    }
    event.rx_checksum = result.rx_checksum;
    event.has_rx_checksum = result.has_rx_checksum;
    event.tx_frame_id = vlc_tx_get_frame_count();
    event.rx_frame_id = s_rx_observed_frames;
    for (i = 0U; i < APP_MAX_PAYLOAD_LEN; ++i)
    {
      event.rx_payload[i] = result.rx_payload[i];
    }
    err_queue_push(&event);
  }
}

// Pop next decoded frame if available.
uint8_t vlc_rx_pop_payload(app_frame_t *frame)
{
  if ((frame == 0) || (s_queue.count == 0U))
  {
    return 0U;
  }

  *frame = s_queue.items[s_queue.tail].frame;
  s_queue.tail = (uint8_t)((s_queue.tail + 1U) % APP_RX_QUEUE_DEPTH);
  s_queue.count--;

  return 1U;
}

// Pop next decoded frame and RX frame ID if available.
uint8_t vlc_rx_pop_frame(app_frame_t *frame, uint32_t *rx_frame_id)
{
  if ((frame == 0) || (rx_frame_id == 0) || (s_queue.count == 0U))
  {
    return 0U;
  }

  *frame = s_queue.items[s_queue.tail].frame;
  *rx_frame_id = s_queue.items[s_queue.tail].rx_frame_id;
  s_queue.tail = (uint8_t)((s_queue.tail + 1U) % APP_RX_QUEUE_DEPTH);
  s_queue.count--;
  return 1U;
}

// Read current RX statistics.
void vlc_rx_get_stats(vlc_rx_stats_t *stats)
{
  if (stats == 0)
  {
    return;
  }

  *stats = s_stats;
}

// Pop one deferred RX error event if available.
uint8_t vlc_rx_pop_error_event(vlc_rx_error_event_t *event)
{
  if ((event == 0) || (s_err_queue.count == 0U))
  {
    return 0U;
  }

  *event = s_err_queue.items[s_err_queue.tail];
  s_err_queue.tail = (uint8_t)((s_err_queue.tail + 1U) % VLC_RX_ERR_QUEUE_DEPTH);
  s_err_queue.count--;
  return 1U;
}
