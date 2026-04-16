//
// dsn_protocol.c
//
// Dyson vacuum UART protocol handler — dynamic TLV-based implementation.
//
// Replaces the static-template approach in protocol.c with a
// pair dispatcher that dynamically parses incoming TLV requests and builds
// response frames at runtime. This correctly handles:
//   - Rolling counters (echoed from request)
//   - Variable-length TLV payloads
//   - All message types via a single generic frame analyzer
//   - CRC32 with 4-byte-alignment padding
//   - CRC8 header checksum (poly 0xE0, reflected)
//   - Byte stuffing (0x12 -> 0xDB 0xDE, 0xDB -> 0xDB 0xDD)
//
// Frame wire format:
//   [0x12] [stuffed frame] [0x12]
//
// Frame (after unstuffing):
//   [SIZE_LO] [SIZE_HI] [HDR_CRC8] [DIR] [0xC0] [SRC] [CLASS] [PAYLOAD...] [CRC32 x4]
//
// Author : Vladislav Gyurov
// License: GNU GPL v3 or later
//

//-----------------------------------------------------------------------------
//    INCLUDE FILES
//-----------------------------------------------------------------------------
#include "dsn_protocol.h"
#include "sw_timer.h"
#include "string.h"
#include "serial.h"
#include "crc.h"
#include "bms.h"
#include "bms_adc.h"
#include "dio.h"
#include "config.h"

//-----------------------------------------------------------------------------
//    DECLARATION OF LOCAL MACROS/#DEFINES
//-----------------------------------------------------------------------------

#ifdef PROT_DEBUG_PRINT
#include "serial_debug.h"
#define DSN_PRINT(...) \
  { \
    char _dbg_tmp[DEBUG_MSG_BUFFER_SIZE]; \
    snprintf(_dbg_tmp, sizeof(_dbg_tmp), __VA_ARGS__); \
    serial_debug_send_message(_dbg_tmp);  \
  }
#else
  #define DSN_PRINT(...)
#endif

// Frame structure
#define FRAME_DELIM             0x12
#define FRAME_MARKER            0xC0
#define FRAME_DIR_DATA          0x01   // DIR byte for data direction
#define FRAME_CLASS_RESPONSE    0x01   // CLASS byte: response

// Byte offsets within unstuffed frame (after start delimiter removed)
#define OFF_SIZE_LO             0
#define OFF_SIZE_HI             1
#define OFF_HDR_CRC8            2
#define OFF_DIR                 3
#define OFF_MARKER              4
#define OFF_SRC                 5
#define OFF_CLASS               6
#define OFF_PAYLOAD             7
#define FRAME_HDR_OVERHEAD      4      // in-frame header (DIR+MARKER+SRC+CLASS) = CRC32 size

// Byte stuffing
#define STUFF_ESCAPE            0xDB
#define STUFF_DELIM_REPLACE     0xDE   // 0xDB 0xDE -> 0x12
#define STUFF_ESCAPE_REPLACE    0xDD   // 0xDB 0xDD -> 0xDB

// Little-endian byte order helpers (wire format is LE)
#define LE16TOH(buf)        ((uint16_t)(buf)[0] | ((uint16_t)(buf)[1] << 8))

#define LE32TOH(buf)        ((uint32_t)(buf)[0]        | ((uint32_t)(buf)[1] << 8)   | \
                            ((uint32_t)(buf)[2] << 16) | ((uint32_t)(buf)[3] << 24))

#define HTOLE16(buf, val)   do { (buf)[0] = (uint8_t)( (val)       & 0xFF); \
                                 (buf)[1] = (uint8_t)(((val) >> 8) & 0xFF); } while(0)

#define HTOLE32(buf, val)   do { (buf)[0] = (uint8_t)((val) >>  0); \
                                 (buf)[1] = (uint8_t)((val) >>  8); \
                                 (buf)[2] = (uint8_t)((val) >> 16); \
                                 (buf)[3] = (uint8_t)((val) >> 24); } while(0)

// Buffer sizes
#define RX_BUF_SIZE             128
#define TX_BUF_SIZE             160
#define MAX_RESPONSE_FRAME      140    // max response frame (SIZE+3)
#define MAX_RESPONSE_PAYLOAD    120    // max payload bytes in response

// Protocol timing
#define TX_WAIT_TICKS           (10 / SW_TIMER_TICK_MS)
#define SESSION_TIMEOUT_MS      2000
#define RX_TIMEOUT_MS           5
#define MOTOR_SPEED_WDT_MS      2000

// TLV pair IDs used in analyze_frame response logic
#define PAIR_TLV_READ           0x1002
#define PAIR_TLV_READ_RES       0x1001

// TLV register keys: (TYPE << 8) | REG
#define TLV_TRIGGER_STATE       0x8100   // 1 byte: trigger on/off
#define TLV_CHARGER_CONNECTED   0x2101   // 1 byte: charger bool
#define TLV_SOC                 0x250D   // 2 bytes: percent * 100
#define TLV_RUNTIME             0x2202   // 4 bytes: seconds (aliased from 0x22)
#define TLV_SOC2                0x8105   // 2 bytes: filtered SOC percent * 100
#define TLV_BMS_STATUS          0x8106   // 1 byte: status enum
#define TLV_MAX_CELL_V          0x250B   // 2 bytes: max pack cell mV
#define TLV_MIN_CELL_V          0x250C   // 2 bytes: min pack cell mV
#define TLV_MIN_PACK_V          0x8114   // 2 bytes: min pack voltage mV
#define TLV_MAX_PACK_V          0x8115   // 2 bytes: max pack voltage mV
#define TLV_BATTERY_TYPE        0x8102   // 2 bytes: battery type ID
#define TLV_FULL_CHARGE_CAP     0x0201   // 4 bytes: capacity in 0.01 mAh
#define TLV_WAKEUP_SOURCE       0x810A   // 1 byte: wakeup source

// Handshake configuration
#define HANDSHAKE_NUM_CELLS     7
#define HANDSHAKE_MIN_CELL_MV   2650
#define HANDSHAKE_MAX_CELL_MV   4200
#define V11_BATTERY_TYPE        0x001F
#define V11_CAPACITY_001MAH     (PACK_MAX_CAPACITY_MAH * 100u)  // in 0.01 mAh units

// Firmware version string (22 bytes, queried by pair 0x0306)
#define FW_VERSION_STR_LEN      22
static const char fw_version_str[FW_VERSION_STR_LEN] = "V11-BMS-1.0";

// Motor speed thresholds from trigger messages
#define MOTOR_SPEED_OFF         0
#define MOTOR_SPEED_IDLE        15000
#define MOTOR_SPEED_ON          660000

//-----------------------------------------------------------------------------
//    DEFINITION OF LOCAL TYPES
//-----------------------------------------------------------------------------
typedef enum
{
  DSN_INIT,
  DSN_IDLE,
  DSN_WAIT_HANDSHAKE,
  DSN_WAIT_FRAME,
  DSN_TX_FRAME,
  DSN_WAIT_TX,
  DSN_SLEEP,
} dsn_state_t;

typedef enum
{
  RX_INIT,
  RX_RECEIVING,
  RX_COMPLETE,
} rx_state_t;

// Processing context for frame analyzer — tracks input/output cursors
typedef struct
{
  const uint8_t *in_ptr;
  uint16_t       in_remaining;
  uint8_t       *out_ptr;
  uint16_t       out_remaining;
} proc_ctx_t;

//-----------------------------------------------------------------------------
//    DEFINITION OF LOCAL VARIABLES
//-----------------------------------------------------------------------------
static uint8_t    rx_buf[RX_BUF_SIZE];
static uint8_t    rx_level;
static rx_state_t rx_state;

static uint8_t    tx_buf[TX_BUF_SIZE];
static uint8_t    tx_length;

static dsn_state_t dsn_state;
static sw_timer    wait_timer;
static sw_timer    session_timer;

static bool        trigger_state;
static bool        sleep_flag;
static bool        vacuum_connected;

static uint32_t    last_motor_speed;
static bool        pending_sleep;
static bool        charger_at_sleep;   // latch charger state when entering DSN_SLEEP
static sw_timer    motor_speed_timer;
static bool        frames_seen;
static bool        motor_speed_seen;
static bool        handshake_key_seen;
static uint32_t    wakeup_source;

//-----------------------------------------------------------------------------
//    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
//-----------------------------------------------------------------------------
static void     frame_unstuff(uint8_t *buf, uint8_t *len);
static uint8_t  frame_stuff(uint8_t *buf, uint8_t frame_len, uint8_t buf_size);
static bool     frame_verify_hdr_crc8(const uint8_t *buf, uint8_t len);
static bool     frame_verify_crc32(const uint8_t *buf);
static void     frame_append_crc32(uint8_t *buf);
static uint8_t  frame_compute_hdr_crc8(const uint8_t *buf);

static bool     rx_byte_handler(uint8_t ch);
static bool     process_rx_frame(void);
static uint16_t analyze_frame(proc_ctx_t *ctx, uint8_t req_class);
static bool     dispatch_pair(proc_ctx_t *in_ctx, uint16_t pair, uint8_t *out_data, uint16_t out_size, uint16_t *out_len);
static bool     dispatch_tlv_read(uint16_t key, uint8_t *out_data, uint16_t *out_len);
static void     build_trigger_response(uint8_t *out_data, uint16_t *out_len);
static void     handle_sleep(void);

//-----------------------------------------------------------------------------
//    DEFINITION OF GLOBAL FUNCTIONS
//-----------------------------------------------------------------------------

/** @brief Initialize protocol state machine and flags. */
void dsn_prot_init(void)
{
  dsn_state = DSN_INIT;
  rx_state  = RX_INIT;
  rx_level  = 0;
  trigger_state    = false;
  sleep_flag       = false;
  vacuum_connected = false;
  last_motor_speed = 0;
  pending_sleep    = false;
  motor_speed_seen = false;
  frames_seen      = false;
  wakeup_source    = 0;
}

/**
 * @brief Set trigger state for protocol responses.
 * @param state  Trigger state to set.
 */
void dsn_prot_set_trigger(bool state)
{
  trigger_state = state;
  if (state)
    pending_sleep = false;
}

/** @brief Reset protocol to initial state. */
void dsn_prot_reset(void)
{
  dsn_state        = DSN_INIT;
  sleep_flag       = false;
  vacuum_connected = false;
  pending_sleep    = false;
  motor_speed_seen = false;
  frames_seen      = false;
}

/**
 * @brief Check if vacuum requested sleep.
 * @return true if sleep requested.
 */
bool dsn_prot_get_sleep_flag(void)
{
  return sleep_flag;
}

/**
 * @brief Check if vacuum is currently connected.
 * @return true if connected.
 */
bool dsn_prot_get_vacuum_connected(void)
{
  return vacuum_connected;
}

/** @brief Protocol main loop — poll UART, process frames, manage session timeout. */
void dsn_prot_mainloop(void)
{
  // Session timeout: no messages for 2 s -> disconnect
  if (sw_timer_is_elapsed(&session_timer, SESSION_TIMEOUT_MS))
    vacuum_connected = false;

  switch (dsn_state)
  {
    //------------------------------------------------------------------------
    case DSN_INIT:
      if (sleep_flag)
        DSN_PRINT("PROT:WAKE\r\n");
      port_pin_set_output_level(PRECHARGE_PIN, true);
      port_pin_set_output_level(MODE_BUTTON_PULLUP_ENABLE_PIN, true);
      rx_level  = 0;
      rx_state  = RX_INIT;
      sleep_flag       = false;
      vacuum_connected = false;
      frames_seen = false;
      motor_speed_seen = false;
      sw_timer_start(&session_timer);
      sw_timer_start(&motor_speed_timer);
      dsn_state = DSN_WAIT_HANDSHAKE;
      break;

    //------------------------------------------------------------------------
    case DSN_WAIT_HANDSHAKE:
    case DSN_WAIT_FRAME:
    {
      uint8_t ch;

      while (serial_rx_byte(&ch))
      {
        if (rx_byte_handler(ch))
        {
          frames_seen = true;

          if (process_rx_frame())
          {
            sw_timer_start(&wait_timer);
            dsn_state = DSN_WAIT_TX;
          }
          rx_level = 0;
          rx_state = RX_RECEIVING;
          break;
        }
      }

      // Motor speed watchdog: if we previously received motor speed
      // but it stopped arriving, the vacuum is stuck — power cycle it
      if (   motor_speed_seen
          && sw_timer_is_elapsed(&motor_speed_timer, MOTOR_SPEED_WDT_MS))
      {
        DSN_PRINT("PROT:MS_WDT\r\n");
        port_pin_set_output_level(PRECHARGE_PIN, false);
        port_pin_set_output_level(MODE_BUTTON_PULLUP_ENABLE_PIN, false);
        delay_ms(500);
        dsn_state = DSN_INIT;
      }
      break;
    }

    //------------------------------------------------------------------------
    case DSN_WAIT_TX:
    {
      if (sw_timer_is_elapsed(&wait_timer, TX_WAIT_TICKS))
        dsn_state = DSN_TX_FRAME;
      break;
    }

    //------------------------------------------------------------------------
    case DSN_TX_FRAME:
      serial_send(tx_buf, tx_length);
      // flush RX 
      rx_level = 0;
      rx_state = RX_INIT;
      
      if (pending_sleep && vacuum_connected && !trigger_state)
      {
        pending_sleep = false;
        handle_sleep();
      }
      else if (vacuum_connected)
      {
        dsn_state = DSN_WAIT_FRAME;
      }
      else
      {
        // discard sleep request received before handshake completes
        pending_sleep = false;
        // restart motor speed watchdog and clear flags
        frames_seen = false;
        motor_speed_seen = false;
        sw_timer_start(&motor_speed_timer);
        dsn_state = DSN_WAIT_HANDSHAKE;
      }
      break;

    //------------------------------------------------------------------------
    case DSN_SLEEP:
      if (     dio_read(DIO_MODE_BUTTON)
           ||  dio_read(DIO_TRIGGER_PRESSED)
           || (charger_at_sleep && !dio_read(DIO_CHARGER_CONNECTED)))
      {
        dsn_state = DSN_INIT;
      }
      break;

    default:
      dsn_state = DSN_INIT;
      break;
  }
}

//-----------------------------------------------------------------------------
//    DEFINITION OF LOCAL FUNCTIONS
//-----------------------------------------------------------------------------

/**
 * @brief  RX byte handler: accumulate frame bytes, detect delimiter.
 * @param  ch  Received byte.
 * @return true on complete frame.
 */
static bool rx_byte_handler(uint8_t ch)
{
  bool complete = false;

  switch (rx_state)
  {
    //-------------------------------------------------------------------------
    case RX_INIT:
    case RX_COMPLETE:
      // Discard bytes until delimiter to re-sync
      if (ch == FRAME_DELIM)
      {
        rx_level = 0;
        rx_state = RX_RECEIVING;
      }
      break;
    //-------------------------------------------------------------------------
    case RX_RECEIVING:
      if (rx_level >= RX_BUF_SIZE)
      {
        DSN_PRINT("RX:OVERFLOW\r\n");
        rx_level = 0;
        rx_state = RX_INIT;
      }
      else if (ch == FRAME_DELIM)
      {
        if (rx_level > (OFF_PAYLOAD + 1))
        {
          rx_state = RX_COMPLETE;
          complete = true;
        }
        else
        {
          if (rx_level > 0)
            DSN_PRINT("RX:SHORT len=%u hdr=%02X%02X%02X%02X\r\n", rx_level, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
          rx_level = 0;
        }
      }
      else
      {
        rx_buf[rx_level++] = ch;
      }
      break;
  }

  return complete;
}

/**
 * @brief  In-place byte unstuffing (0xDB 0xDE -> 0x12, 0xDB 0xDD -> 0xDB).
 * @param  buf  Buffer containing stuffed frame (no delimiters).
 * @param  len  Pointer to buffer length; updated to unstuffed length on return.
 */
static void frame_unstuff(uint8_t *buf, uint8_t *len)
{
  uint8_t dst = 0;
  uint8_t src;
  bool esc = false;

  for (src = 0; src < *len; src++)
  {
    if (esc)
    {
      if (buf[src] == STUFF_DELIM_REPLACE)
        buf[dst++] = FRAME_DELIM;
      else if (buf[src] == STUFF_ESCAPE_REPLACE)
        buf[dst++] = STUFF_ESCAPE;
      esc = false;
    }
    else if (buf[src] == STUFF_ESCAPE)
    {
      esc = true;
    }
    else
    {
      buf[dst++] = buf[src];
    }
  }
  *len = dst;
}

/**
 * @brief  Verify CRC8 over the 2-byte SIZE field.
 * @param  buf      Pointer to frame buffer.
 * @param  len  Length of frame buffer.
 * @return true if valid.
 */
static bool frame_verify_hdr_crc8(const uint8_t *buf, uint8_t len)
{
  uint8_t expected;

  if (len < 4)
    return false;

  expected = calc_crc8(&buf[OFF_SIZE_LO], 2);
  return (expected == buf[OFF_HDR_CRC8]);
}

/**
 * @brief  Compute CRC8 for response frame header.
 * @param  buf  Pointer to frame buffer.
 * @return CRC8 byte.
 */
static uint8_t frame_compute_hdr_crc8(const uint8_t *buf)
{
  return calc_crc8(&buf[OFF_SIZE_LO], 2);
}

/**
 * @brief  Verify CRC32 over frame from DIR to end of payload.
 * @param  buf  Pointer to frame buffer.
 * @return true if valid.
 */
static bool frame_verify_crc32(const uint8_t *buf)
{
  uint16_t size = LE16TOH(&buf[OFF_SIZE_LO]);
  uint32_t computed = calc_crc32(&buf[OFF_DIR], size - FRAME_HDR_OVERHEAD);
  uint32_t stored = LE32TOH(&buf[size - 1]);
  return (computed == stored);
}

/**
 * @brief  Compute and append CRC32 to response frame.
 * @param  buf  Pointer to frame buffer.
 */
static void frame_append_crc32(uint8_t *buf)
{
  uint16_t size = LE16TOH(&buf[OFF_SIZE_LO]);
  uint32_t crc = calc_crc32(&buf[OFF_DIR], size - FRAME_HDR_OVERHEAD);

  HTOLE32(&buf[size - 1], crc);
}

/**
 * @brief  Byte-stuff frame in-place and add delimiters.
 * @param  buf       Buffer containing frame (output includes delimiters).
 * @param  frame_len Length of the unstuffed frame.
 * @param  buf_size  Total buffer capacity.
 * @return Total stuffed length, 0 on overflow.
 */
static uint8_t frame_stuff(uint8_t *buf, uint8_t frame_len, uint8_t buf_size)
{
  uint8_t extra = 0;
  uint8_t total;
  uint8_t dst;
  uint8_t i;
  int8_t  src;

  if (frame_len < 2)
    return 0;

  for (i = 0; i < frame_len; i++)
  {
    if (buf[i] == FRAME_DELIM || buf[i] == STUFF_ESCAPE)
      extra++;
  }

  total = 1 + frame_len + extra + 1; // start delimiter + frame data + stuff + end delimiter
  if (total > buf_size)
    return 0; // not enough space for stuffed frame

  dst = (total - 1);         // index for end delimiter
  buf[dst--] = FRAME_DELIM;  // fill end delimiter

  // start from back to avoid overwriting unprocessed data
  for (src = (frame_len - 1); src >= 0; src--)
  {
    if (buf[src] == FRAME_DELIM)
    {
      buf[dst--] = STUFF_DELIM_REPLACE;
      buf[dst--] = STUFF_ESCAPE;
    }
    else if (buf[src] == STUFF_ESCAPE)
    {
      buf[dst--] = STUFF_ESCAPE_REPLACE;
      buf[dst--] = STUFF_ESCAPE;
    }
    else
    {
      buf[dst--] = buf[src];
    }
  }

  buf[0] = FRAME_DELIM;  // start delimiter
  return total;
}

/**
 * @brief  Process complete received frame: unstuff, verify CRCs, dispatch TLV pairs.
 * @return true if response built in tx_buf.
 */
static bool process_rx_frame(void)
{
  uint16_t size;
  uint8_t  req_class;
  uint16_t payload_len;
  proc_ctx_t ctx;
  uint16_t resp_payload_len;
  uint16_t resp_size;

  // Unstuff in place
  frame_unstuff(rx_buf, &rx_level);

  // Verify header CRC8
  if (!frame_verify_hdr_crc8(rx_buf, rx_level))
  {
    DSN_PRINT("RX:BAD_CRC8 len=%u hdr=%02X%02X%02X%02X\r\n", rx_level, rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);
    return false;
  }

  // Check frame length matches SIZE field
  size = LE16TOH(&rx_buf[OFF_SIZE_LO]);

  if (size < 8 || rx_level != (size + OFF_DIR))
  {
    DSN_PRINT("RX:BAD_SIZE sz=%u lvl=%u\r\n", size, rx_level);
    return false;
  }

  // Verify CRC32
  if (!frame_verify_crc32(rx_buf))
  {
    DSN_PRINT("RX:BAD_CRC32\r\n");
    return false;
  }

  // Handle discovery broadcast (SRC=0xFF): extract payload, no response
  if (rx_buf[OFF_SRC] == 0xFF)
  {
    payload_len = size - FRAME_HDR_OVERHEAD - 4;
    if (payload_len >= 4)
    {
      const uint8_t *p = &rx_buf[OFF_PAYLOAD];
      wakeup_source = LE32TOH(p);
      DSN_PRINT("PROT:DISC 0x%08lX\r\n", wakeup_source);
    }
    return false;  // no TX for broadcasts
  }

  // Accept SRC=0x01 (standard data) only.
  if (rx_buf[OFF_SRC] != 0x01)
  {
    DSN_PRINT("RX:BAD_SRC 0x%02X\r\n", rx_buf[OFF_SRC]);
    return false;
  }

  // Reset session timer on any valid frame addressed to us
  sw_timer_start(&session_timer);

  // handshake_key_seen is checked after analyze_frame() below

  // Extract header fields
  req_class = rx_buf[OFF_CLASS];  // 0x02 = data req, 0x03 = control req

  // Payload starts after the 4-byte header (DIR/MARKER/SRC/CLASS)
  payload_len = size - FRAME_HDR_OVERHEAD - 4;  // subtract CRC32 (4)

  // Set up processing context
  ctx.in_ptr       = &rx_buf[OFF_PAYLOAD];
  ctx.in_remaining = payload_len;

  // Build response frame directly in tx_buf
  tx_buf[OFF_DIR]    = FRAME_DIR_DATA;
  tx_buf[OFF_MARKER] = FRAME_MARKER;
  tx_buf[OFF_SRC]    = req_class;  // response SRC = request CLASS (Dyson protocol convention)
  tx_buf[OFF_CLASS]  = FRAME_CLASS_RESPONSE;

  ctx.out_ptr       = &tx_buf[OFF_PAYLOAD];
  ctx.out_remaining = MAX_RESPONSE_PAYLOAD;

  // Clear handshake key flag before processing
  handshake_key_seen = false;

  // Run the TLV frame analyzer
  resp_payload_len = analyze_frame(&ctx, req_class);

  // Complete handshake when the startup-only key (TLV_MAX_PACK_V) was queried
  if (!vacuum_connected && handshake_key_seen)
  {
    DSN_PRINT("PROT:HS\r\n");
    vacuum_connected = true;
  }

  if (resp_payload_len == 0)
  {
    DSN_PRINT("RX:EMPTY_RESP cls=0x%02X\r\n", req_class);
    return false;
  }

  // Compute SIZE = payload + 4 (DIR+MARKER+SRC+CLASS) + 4 (CRC32)
  resp_size = resp_payload_len + 4 + 4;
  HTOLE16(&tx_buf[OFF_SIZE_LO], resp_size);

  // Compute and store header CRC8
  tx_buf[OFF_HDR_CRC8] = frame_compute_hdr_crc8(tx_buf);

  // Compute and append CRC32
  frame_append_crc32(tx_buf);

  // Byte-stuff the response frame and wrap with delimiters
  tx_length = frame_stuff(tx_buf, (uint8_t)(resp_size + OFF_DIR), TX_BUF_SIZE);

  if (tx_length == 0)
  {
    DSN_PRINT("TX:STUFF_OVERFLOW\r\n");
    return false;
  }

  return true;
}

/**
 * @brief  TLV frame analyzer: process pairs from payload, dispatch each, build response.
 * @param  ctx        Processing context tracking input/output cursors.
 * @param  req_class  Request class byte from frame header.
 * @return Bytes written to output.
 */
static uint16_t analyze_frame(proc_ctx_t *ctx, uint8_t req_class)
{
  uint16_t total_written = 0;
  uint16_t pair;
  uint8_t  resp_data[24];
  uint16_t resp_len;
  bool     ok;
  uint16_t needed;
  uint16_t resp_pair;

  (void)req_class;

  if (ctx->in_remaining == 0)
    return 0;

  // Copy rolling counter to output
  if (ctx->out_remaining > 0)
  {
    *ctx->out_ptr = *ctx->in_ptr;
    ctx->out_ptr++;
    ctx->out_remaining--;
    ctx->in_ptr++;
    ctx->in_remaining--;
    total_written++;
  }

  // Process TLV pairs: read 2-byte pair ID, dispatch, write response
  while (ctx->in_remaining >= 2 && ctx->out_remaining >= 2)
  {
    pair = LE16TOH(ctx->in_ptr);
    ctx->in_ptr       += 2;
    ctx->in_remaining -= 2;

    resp_len = 0;
    ok = dispatch_pair(ctx, pair, resp_data, sizeof(resp_data), &resp_len);

    if (!ok)
    {
      // Unknown pair — we don't know how many bytes to consume,
      // so we must stop processing to avoid corrupting subsequent pairs.
      DSN_PRINT("RX:UNK_PAIR 0x%04X\r\n", pair);
      break;
    }

    if (resp_len == 0)
    {
      // Known pair with no response data (silent ack).
      continue;
    }

    // Check space for response pair header + data
    needed = 2 + resp_len;
    if (ctx->out_remaining < needed)
      break;

    // Write response pair header (pair + 1 for response, per Dyson protocol)
    resp_pair = pair + 1;
    // Special case: TLV read request 0x1002 -> response 0x1001
    if (pair == PAIR_TLV_READ)
      resp_pair = PAIR_TLV_READ_RES;

    HTOLE16(ctx->out_ptr, resp_pair);
    ctx->out_ptr       += 2;
    ctx->out_remaining -= 2;
    total_written      += 2;

    // Copy response data
    memcpy(ctx->out_ptr, resp_data, resp_len);
    ctx->out_ptr       += resp_len;
    ctx->out_remaining -= resp_len;
    total_written      += resp_len;
  }

  return total_written;
}

/**
 * @brief  Dispatch a TLV pair: consume input bytes and produce response data.
 *
 * @param in_ctx    Processing context (input cursor advanced on success).
 * @param pair      16-bit pair ID from the incoming frame.
 * @param out_data  Buffer for response payload.
 * @param out_size  Size of out_data buffer in bytes.
 * @param out_len   Number of response bytes written.
 * @return true if pair is known, false to abort frame processing.
 */
static bool dispatch_pair(proc_ctx_t *in_ctx, uint16_t pair, uint8_t *out_data, uint16_t out_size, uint16_t *out_len)
{
  const uint8_t *in;
  uint8_t  reg;
  uint8_t  type;
  uint16_t key;
  uint8_t  val_buf[4];
  uint16_t val_len;

  *out_len = 0;

  switch (pair)
  {
    //---------------------------------------------------------------------------------------------
    // ECHO_REQ: [0x07 0x10] [BYTE] -> [0x08 0x10] [BYTE]
    case 0x1007:
      if (in_ctx->in_remaining < 1)
        return false;

      in = in_ctx->in_ptr;
      in_ctx->in_ptr       += 1;
      in_ctx->in_remaining -= 1;

      if (out_size < 1)
        return true;
      out_data[0] = in[0];
      *out_len = 1;
      return true;

    //---------------------------------------------------------------------------------------------
    // TLV_READ: [0x02 0x10] [REG] [TYPE] -> [0x01 0x10] [REG] [TYPE] [LEN_LO] [LEN_HI] [DATA...]
    case 0x1002:
      if (in_ctx->in_remaining < 2)
        return false;

      in = in_ctx->in_ptr;
      in_ctx->in_ptr       += 2;
      in_ctx->in_remaining -= 2;

      reg  = in[0];
      type = in[1];
      key = ((uint16_t)type << 8) | reg;

      val_len = 0;
      if (!dispatch_tlv_read(key, val_buf, &val_len))
        return true;   // unknown register — skip

      if (out_size < 4 + val_len)
        return true;

      // Output format: [REG] [TYPE] [LEN_LO] [LEN_HI] [DATA...]
      out_data[0] = reg;
      out_data[1] = type;
      HTOLE16(&out_data[2], val_len);
      memcpy(&out_data[4], val_buf, val_len);
      *out_len = 4 + val_len;
      return true;

    //---------------------------------------------------------------------------------------------
    // FW_VERSION_STR: [0x06 0x03] [OFFSET] [REQ_LEN] -> [0x07 0x03] [ACTUAL_LEN] [DATA...]
    case 0x0306:
    {
      uint8_t req_len;

      if (in_ctx->in_remaining < 2)
        return false;

      in = in_ctx->in_ptr;
      in_ctx->in_ptr       += 2;
      in_ctx->in_remaining -= 2;

      req_len = in[1];

      if (req_len > FW_VERSION_STR_LEN)
        req_len = FW_VERSION_STR_LEN;

      if (1 + req_len > out_size)
        return true;

      out_data[0] = req_len;
      memcpy(&out_data[1], fw_version_str, req_len);
      *out_len = 1 + req_len;
      return true;
    }

    //---------------------------------------------------------------------------------------------
    // BMS_MASKED_WRITE: [0x16 0x82] [VAL_LO] [VAL_HI] [MASK_LO] [MASK_HI] -> silent ack
    case 0x8216:
      if (in_ctx->in_remaining < 4)
        return false;

      in_ctx->in_ptr       += 4;
      in_ctx->in_remaining -= 4;
      return true;

    //---------------------------------------------------------------------------------------------
    // CONNECTION_ACK: [0x02 0x82] (no payload, no response)
    case 0x8202:
      return true;

    //---------------------------------------------------------------------------------------------
    // MOTOR_SPEED: [0x00 0x82] [SPEED LE 4B] -> [0x01 0x82] [ACK_SPEED LE 4B] [0x00] [FLAGS]
    case 0x8200:
      if (in_ctx->in_remaining < 4)
        return false;

      in = in_ctx->in_ptr;
      in_ctx->in_ptr       += 4;
      in_ctx->in_remaining -= 4;

      last_motor_speed = LE32TOH(in);
      motor_speed_seen = true;
      sw_timer_start(&motor_speed_timer);

      if (last_motor_speed == MOTOR_SPEED_OFF)
        pending_sleep = true;

      if (out_size < 6)
        return true;
      build_trigger_response(out_data, out_len);
      return true;

    //---------------------------------------------------------------------------------------------
    default:
      return false;
  }
}

/**
 * @brief  TLV register dispatcher: map (TYPE<<8)|REG keys to BMS sensor values.
 * @return true if key known.
 */
static bool dispatch_tlv_read(uint16_t key, uint8_t *out_data, uint16_t *out_len)
{
  uint16_t soc;
  uint16_t val;
  uint32_t val32;

  *out_len = 0;

  switch (key)
  {
    //--- 1-byte registers ---

    case TLV_TRIGGER_STATE:  // 0x8100: trigger on/off
      out_data[0] = trigger_state ? 1 : 0;
      *out_len = 1;
      return true;

    case TLV_CHARGER_CONNECTED:  // 0x2101: charger connected bool
      out_data[0] = dio_read(DIO_CHARGER_CONNECTED) ? 1 : 0;
      *out_len = 1;
      return true;

    case TLV_BMS_STATUS:  // 0x8106: BMS status byte
      out_data[0] = 0x01;  // 1 = OK
      *out_len = 1;
      return true;

    case TLV_WAKEUP_SOURCE:  // 0x810A: wakeup source (from discovery)
      out_data[0] = (uint8_t)wakeup_source;
      *out_len = 1;
      return true;

    //--- 2-byte registers ---

    case TLV_SOC:  // 0x250D: SOC in percent * 100
      soc = bms_get_soc_x100();
      HTOLE16(out_data, soc);
      *out_len = 2;
      return true;

    case TLV_SOC2:  // 0x8105: filtered SOC percent * 100
      soc = bms_get_soc_x100();
      HTOLE16(out_data, soc);
      *out_len = 2;
      return true;

    case TLV_MAX_CELL_V:  // 0x250B: max cell voltage mV
      HTOLE16(out_data, HANDSHAKE_MAX_CELL_MV);
      *out_len = 2;
      return true;

    case TLV_MIN_CELL_V:  // 0x250C: min cell voltage mV
      HTOLE16(out_data, HANDSHAKE_MIN_CELL_MV);
      *out_len = 2;
      return true;

    case TLV_MIN_PACK_V:  // 0x8114: min pack voltage mV
      HTOLE16(out_data, HANDSHAKE_MIN_CELL_MV * HANDSHAKE_NUM_CELLS);
      *out_len = 2;
      return true;

    case TLV_MAX_PACK_V:  // 0x8115: max pack voltage mV
      HTOLE16(out_data, HANDSHAKE_MAX_CELL_MV * HANDSHAKE_NUM_CELLS);
      *out_len = 2;
      return true;
      
    case TLV_BATTERY_TYPE:  // 0x8102: battery type ID
      val = V11_BATTERY_TYPE;
      HTOLE16(out_data, val);
      *out_len = 2;
      handshake_key_seen = true;
      return true;

    //--- 4-byte registers ---
    case TLV_RUNTIME:  // 0x2202: runtime in seconds
      val32 = bms_get_runtime_seconds();
      HTOLE32(out_data, val32);
      *out_len = 4;
      return true;

    case TLV_FULL_CHARGE_CAP:  // 0x0201: full charge capacity (0.01 mAh)
      val32 = V11_CAPACITY_001MAH;
      HTOLE32(out_data, val32);
      *out_len = 4;
      return true;

    default:
      return false;
  }
}

/**
 * @brief  Build 6-byte trigger response: [ack_speed LE 4B] [0x00] [flags 1B].
 */
static void build_trigger_response(uint8_t *out_data, uint16_t *out_len)
{
  uint32_t ack_speed;
  bool charger_connected = dio_read(DIO_CHARGER_CONNECTED);

  if (last_motor_speed == MOTOR_SPEED_OFF)
  {
    ack_speed = MOTOR_SPEED_OFF;
  }
  else if (!charger_connected && trigger_state)
  {
    ack_speed = MOTOR_SPEED_ON;
  }
  else
  {
    ack_speed = MOTOR_SPEED_IDLE;
  }

  HTOLE32(out_data, ack_speed);
  out_data[4] = 0x00;
  out_data[5] = 0x00;
  *out_len = 6;
}

/**
 * @brief  Enter sleep state: set flags, disable precharge, transition to DSN_SLEEP.
 */
static void handle_sleep(void)
{
  sleep_flag       = true;
  vacuum_connected = false;
  charger_at_sleep = dio_read(DIO_CHARGER_CONNECTED);
  port_pin_set_output_level(PRECHARGE_PIN, false);
  port_pin_set_output_level(MODE_BUTTON_PULLUP_ENABLE_PIN, false);
  delay_ms(300);
  dsn_state = DSN_SLEEP;
  DSN_PRINT("PROT:SLEEP\r\n");
}

//-----------------------------------------------------------------------------
//    END OF MODULE
//-----------------------------------------------------------------------------
