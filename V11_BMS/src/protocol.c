/*
 * protocol.c
 *
 * Created: 21-Jan-26 10:36:23
 *  Author: GYV1SF4
 */ 
 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "protocol.h"
#include "sw_timer.h"
#include "string.h"
#include "serial.h"
#include "assert.h"
#include "crc32.h"
#include "bms.h"
#include "bms_adc.h"

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
#define PROT_DEBUG_RX_RAW                   1
#define PROT_DEBUG_RX_ANALYZED              1


#define MSG_DELIM_IDX                       0
#define MSG_ID_IDX                          1
// message 0x12 0x38 .. indexes
#define BMS_MSG_ROLLING_COUNTER_IDX         8
#define BMS_MSG_TRIGGER_IDX                 15
#define BMS_MSG_CHARGER_CONNECTED_IDX       22
#define BMS_MSG_SOC_LO_IDX                  29
#define BMS_MSG_SOC_HI_IDX                  30
#define BMS_MSG_BAT_RUNTIME_LO_IDX          37
#define BMS_MSG_BAT_RUNTIME_HI_IDX          38
#define BMS_MSG_SOC2_LO_IDX                 47
#define BMS_MSG_SOC2_HI_IDX                 48

#define MSG_DELIM_CHAR                      0x12
#define MSG_DELIM_SIZE                      1
#define CRC_SIZE                            4
#define TX_WAIT_TIME                        (5 / SW_TIMER_TICK_MS)

#define BYTE_0x12_REPLACEMENT               (0xDEDB)  // 0x12 is replaced with 0xDB 0xDE to avoid framing error
#define BYTE_0xDB_REPLACEMENT               (0xDDDB)  // 0xDB is replaced with 0xDB 0xDD to avoid framing error

#ifdef PROT_DEBUG_PRINT
  #define PROT_PRINT(...) \
  { \
    sprintf(debug_msg_buffer, __VA_ARGS__); \
    serial_debug_send_message(debug_msg_buffer);  \
  }
#else
  #define PROT_PRINT(...)
#endif

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/
typedef enum
{
  PROT_INIT,
  PROT_IDLE,
  PROT_WAIT_FRAME,
  PROT_PREPARE_DATA_FRAME,
  PROT_TX_FRAME,
  PROT_TX_TRIGGER_RESP,
  PROT_WAIT_STATE,
  PROT_DISABLED,
}prot_states;

typedef enum
{
  PROT_RX_INIT,
  PROT_RX_WAIT_START,
  PROT_RX_CHECK_SIZE,
  PROT_RX_FRAME,
  PROT_RX_FRAME_RECEIVED
}rx_states;

typedef struct
{
  const uint8_t* msg_req_ptr;
  const uint8_t* msg_res_ptr;
  uint8_t        msg_req_cmp_size;
  uint8_t        msg_req_size;
  uint8_t        msg_res_size;
  prot_states    dest_state;
#ifdef PROT_DEBUG_PRINT
  bool           dump_bytes;
  const char*    debug_str;
#endif
}prot_cfg_t;

typedef struct
{
  uint8_t  msg_id_size; // from byte 1, message id/size
  uint8_t  msg_size;
  sw_timer timestamp;
  int32_t  time_diff;
}rx_debug_t;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
static sw_timer wait_timer = 0;
static bool     prot_trigger_state = false;
static bool     prot_vacuum_connected = false;

uint8_t serial_buffer_rx[80]       = {0};
uint8_t serial_buffer_tx[80]       = {0};
static uint8_t serial_buffer_level = 0;
static prot_states prot_state      = PROT_INIT;
static prot_states prot_state_next = PROT_INIT;
static uint8_t tx_msg_idx          = 0;
static uint8_t tx_length           = 0;
static rx_states rx_state          = PROT_RX_INIT;


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/
#if (PROT_DEBUG_RX_RAW != 0)
volatile rx_debug_t rx_debug_raw[32]      = {0};
volatile uint8_t rx_debug_raw_id_mask     = 0;
volatile uint8_t rx_debug_raw_cnt         = 0;
#endif

#if (PROT_DEBUG_RX_ANALYZED != 0)
volatile rx_debug_t rx_debug_analyzed[32]  = {0};
volatile uint8_t rx_debug_analyzed_id_mask = 0;
volatile uint8_t rx_debug_analyzed_cnt     = 0;
#endif
uint16_t discarded_frames_cnt = 0;

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/

// vacuum type message
//static const uint8_t msg_vac_type_v15[] = {0x12, 0x1A, 0x00, 0x31, 0x01, 0xC0, 0xFF, 0x03, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0xFF, 0x21, 0xBF, 0xCB, 0xAC, 0x12};
//static const uint8_t msg_vac_type_v11[] = {0x12, 0x1A, 0x00, 0x31, 0x01, 0xC0, 0x01, 0x02, 0x00, 0x00, 0x20, 0x00, 0x00, 0x04, 0x00, 0x23, 0x00, 0x03, 0x03, 0x02, 0x00, 0x00, 0x00, 0x10, 0x00, 0xFF, 0x69, 0x4D, 0x83, 0x84, 0x12};
//static const uint8_t msg_vac_type_res[] = {0x12, 0x0B, 0x00, 0x49, 0x01, 0xC0, 0x02, 0x01, 0x00, 0x01, 0x00, 0xF0, 0xA2, 0xFF, 0x14, 0x12};

// handshake between vacuum and bms, reguired to turn on unit
static const uint8_t msg_vac_v15_handshake_req[]   = {0x12, 0x22, 0x00, 0xE4, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0x13, 0x02, 0x10, 0x02, 0x81, 0x02, 0x10, 0x01, 0x02, 0x02, 0x10, 0x15, 0x81, 0x02, 0x10, 0x14, 0x81, 0x16, 0x82, 0x00, 0x00, 0x01, 0x00, 0x68, 0xCC, 0x78, 0x60, 0x12};
static const uint8_t msg_bms_v15_handshake_res[]   = {0x12, 0x30, 0x00, 0x2B, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x13, 0x01, 0x10, 0x02, 0x81, 0x02, 0x00, 0xB8, 0x02, 0x01, 0x10, 0x01, 0x02, 0x04, 0x00, 0xC0, 0x27, 0x09, 0x00, 0x01, 0x10, 0x15, 0x81, 0x02, 0x00, 0xD8, 0x72, 0x01, 0x10, 0x14, 0x81, 0x02, 0x00, 0x76, 0x48, 0x01, 0x00, 0x21, 0x32, 0x21, 0xD1, 0x12};

static const uint8_t msg_vac_v11_handshake_req[]   = {0x12, 0x20, 0x00, 0x3E, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0x13, 0x02, 0x10, 0x00, 0x81, 0x02, 0x10, 0x02, 0x81, 0x02, 0x10, 0x01, 0x02, 0x02, 0x10, 0x15, 0x81, 0x02, 0x10, 0x14, 0x81, 0xE7, 0xF3, 0xD3, 0xC0, 0x12};
static const uint8_t msg_bms_v11_handshake_res[]   = {0x12, 0x35, 0x00, 0x33, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x13, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x00, 0x01, 0x10, 0x02, 0x81, 0x02, 0x00, 0x1F, 0x00, 0x01, 0x10, 0x01, 0x02, 0x04, 0x00, 0x20, 0xDB, 0xDE, 0x0A, 0x00, 0x01, 0x10, 0x15, 0x81, 0x02, 0x00, 0xD8, 0x72, 0x01, 0x10, 0x14, 0x81, 0x02, 0x00, 0x76, 0x48, 0x87, 0x91, 0xA0, 0x74, 0x12};

// request/response between vacuum and bms (byte 9 is rolling counter)
static const uint8_t msg_vac_data_req[]    = {0x12, 0x21, 0x00, 0x53, 0x01, 0xC0, 0x01, 0x02, 0x04, 0x02, 0x10, 0x00, 0x81, 0x02, 0x10, 0x01, 0x21, 0x02, 0x10, 0x0D, 0x25, 0x02, 0x10, 0x02, 0x22, 0x02, 0x10, 0x05, 0x81, 0x02, 0x10, 0x06, 0x81, 0x96, 0xDD, 0x33, 0x81, 0x12};
// bms response of 0x21 is 0x38 (byte 9 is rolling counter, must be copied from 0x21)
static const uint8_t msg_bms_data_res[]   = {0x12, 0x38, 0x00, 0xC1, 0x01, 0xC0, 0x02, 0x01, 0x04, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x01, 0x01, 0x10, 0x01, 0x21, 0x01, 0x00, 0x00, 0x01, 0x10, 0x0D, 0x25, 0x02, 0x00, 0xFA, 0x25, 0x01, 0x10, 0x02, 0x22, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x05, 0x81, 0x02, 0x00, 0x67, 0x11, 0x01, 0x10, 0x06, 0x81, 0x01, 0x00, 0x01, 0x7F, 0x52, 0x4C, 0xE7, 0x12};

// vacuum trigger message request
static const uint8_t msg_vac_trig_req_ok[]  = {0x12, 0x16, 0x00, 0xAE, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0x11, 0x00, 0x82, 0x40, 0x42, 0x0F, 0x00, 0x02, 0x10, 0x00, 0x81, 0xC5, 0x6A, 0xE2, 0x0B, 0x12};
//                               unscrambled          0x12, 0x16, 0x00, 0xAE, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0xDB, 0xDE, 0x00, 0x82, 0x98, 0x3A, 0x00, 0x00, 0x02, 0x10, 0x00, 0x81, 0xD7, 0xBA, 0xB5, 0x68, 0x12
static const uint8_t msg_vac_trig_req_nok[] = {0x12, 0x16, 0x00, 0xAE, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0x12, 0x00, 0x82, 0x98, 0x3A, 0x00, 0x00, 0x02, 0x10, 0x00, 0x81, 0xD7, 0xBA, 0xB5, 0x68, 0x12};
// bms response of msg 0x16 is 0x1B
// vac running
static const uint8_t msg_bms_trig_on_res_ok[]   = {0x12, 0x1B, 0x00, 0x5C, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x11, 0x01, 0x82, 0x20, 0xDB, 0xDE, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x01, 0xA9, 0x82, 0xBE, 0x65, 0x12};
// vac not running
static const uint8_t msg_bms_trig_off_res_ok[]  = {0x12, 0x1B, 0x00, 0x5C, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x11, 0x01, 0x82, 0x98, 0x3A, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x00, 0x71, 0x76, 0x13, 0x64, 0x12};
static const uint8_t msg_bms_trig_off_res_nok[] = {0x12, 0x1B, 0x00, 0x5C, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0xDB, 0xDE, 0x01, 0x82, 0x98, 0x3A, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x00, 0xF5, 0x2D, 0x89, 0x37, 0x12};

static const prot_cfg_t prot_cfg[] = 
{
  {
    .msg_req_ptr      = msg_vac_data_req,
    .msg_res_ptr      = serial_buffer_tx,
    .msg_req_cmp_size = 6,
    .msg_req_size     = sizeof(msg_vac_data_req),
    .msg_res_size     = sizeof(msg_bms_data_res),
    .dest_state       = PROT_PREPARE_DATA_FRAME,
#ifdef PROT_DEBUG_PRINT
    .dump_bytes       = false,
    .debug_str        = NULL,
#endif
  },
  {
    .msg_req_ptr      = msg_vac_trig_req_ok,
    .msg_res_ptr      = NULL,
    .msg_req_cmp_size = 12,
    .msg_req_size     = sizeof(msg_vac_trig_req_ok),
    .msg_res_size     = 0,
    .dest_state       = PROT_TX_TRIGGER_RESP,
#ifdef PROT_DEBUG_PRINT
    .dump_bytes       = false,
    .debug_str        = NULL
#endif
  },
  //{
  //  .msg_req_ptr      = msg_vac_trig_req_nok,
  //  .msg_res_ptr      = msg_bms_trig_off_res_nok,
  //  .msg_req_cmp_size = 12,
  //  .msg_req_size     = sizeof(msg_vac_trig_req_nok),
  //  .msg_res_size     = sizeof(msg_bms_trig_off_res_nok),
  //  .dest_state       = PROT_TX_FRAME,
  //  #ifdef PROT_DEBUG_PRINT
  //  .dump_bytes       = false,
  //  .debug_str        = "TRG_NOK\r\n",
  //  #endif
  //},
  {
    .msg_req_ptr      = msg_vac_v11_handshake_req,
    .msg_res_ptr      = msg_bms_v11_handshake_res,
    .msg_req_cmp_size = sizeof(msg_vac_v11_handshake_req),
    .msg_req_size     = sizeof(msg_vac_v11_handshake_req),
    .msg_res_size     = sizeof(msg_bms_v11_handshake_res),
    .dest_state       = PROT_TX_FRAME,
#ifdef PROT_DEBUG_PRINT
    .dump_bytes       = false,
    .debug_str        = "V11\r\n",
#endif
  },
  {
    .msg_req_ptr      = msg_vac_v15_handshake_req,
    .msg_res_ptr      = msg_bms_v15_handshake_res,
    .msg_req_cmp_size = sizeof(msg_vac_v15_handshake_req),
    .msg_req_size     = sizeof(msg_vac_v15_handshake_req),
    .msg_res_size     = sizeof(msg_bms_v15_handshake_res),
    .dest_state       = PROT_TX_FRAME,
#ifdef PROT_DEBUG_PRINT
    .dump_bytes       = false,
    .debug_str        = "V15\r\n",
#endif
  },
};

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static prot_states prot_analyze_frame(prot_states current_state);
static void prot_assemble_data_frame(void);
static uint8_t prot_scramble_frame(uint8_t * dest, const uint8_t * src, size_t dest_len, size_t src_len);

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief
//- **************************************************************************
void prot_init(void)
{
  prot_state = PROT_INIT;
  serial_buffer_level = 0;
  prot_trigger_state = false;
  prot_vacuum_connected = false;

  if(sizeof(msg_bms_data_res) > sizeof(serial_buffer_rx) )
  {
    assert(false);
    while(1);
  }
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void prot_set_enable(bool enable)
{
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void prot_set_trigger(bool trigger_state)
{
  prot_trigger_state = trigger_state;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
bool prot_is_vacuum_connected(void)
{
  return prot_vacuum_connected;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void prot_reset(void)
{
  prot_state = PROT_INIT;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void prot_mainloop(void)
{

  switch(prot_state)
  {
    //------------------------------------------------------------------------
    case PROT_INIT:
    {
      serial_buffer_level = 0;
      prot_state = PROT_WAIT_FRAME;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_WAIT_FRAME:
    {
      prot_states prot_state_req = prot_analyze_frame(prot_state);

      if(prot_state != prot_state_req)
      {
        sw_timer_start(&wait_timer);
        prot_state_next = prot_state_req;
        prot_state      = PROT_WAIT_STATE;
      }
    }
    break;
    //------------------------------------------------------------------------
    case PROT_PREPARE_DATA_FRAME:
    {
      prot_assemble_data_frame();
      prot_state = PROT_TX_FRAME;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_TX_FRAME:
    {
      if(tx_msg_idx < (sizeof(prot_cfg) / sizeof(prot_cfg[0])))
      {
        serial_send((uint8_t*)prot_cfg[tx_msg_idx].msg_res_ptr, tx_length);
      }
      
      prot_state = PROT_WAIT_FRAME;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_TX_TRIGGER_RESP:
    {
      bool charger_connected = port_pin_get_input_level(CHARGER_CONNECTED_PIN);

      if(charger_connected == false && prot_trigger_state != false)
        serial_send((uint8_t*)msg_bms_trig_on_res_ok, sizeof(msg_bms_trig_on_res_ok));
      else
        serial_send((uint8_t*)msg_bms_trig_off_res_ok, sizeof(msg_bms_trig_off_res_ok));

      prot_state = PROT_WAIT_FRAME;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_WAIT_STATE:
    {
      if(sw_timer_is_elapsed(&wait_timer, TX_WAIT_TIME))
      {
        prot_state = prot_state_next;
      }
    }
    break;
    //------------------------------------------------------------------------
    case PROT_DISABLED:
    {
      // do nothing here, communication is disabled 
    }
    break;
    default: break;
  }
}

//- **************************************************************************
//! \brief 
//- **************************************************************************
void prot_serial_rx_callback(uint8_t ch)
{
  static uint8_t ch_prev = 0;

  switch(rx_state)
  {
    case PROT_RX_INIT:
    {
      ch_prev = 0;
      serial_buffer_level = 0;
      rx_state = PROT_RX_WAIT_START;
    }
    // no break here, we want to capture first byte
    case PROT_RX_WAIT_START:
    {
      if(ch == MSG_DELIM_CHAR)
      {
        serial_buffer_rx[serial_buffer_level++] = ch;
        rx_state = PROT_RX_CHECK_SIZE;
      }
    }
    break;
    case PROT_RX_CHECK_SIZE:
    {
    // the byte after frame delimiter is message size, filter invalid sizes from the beginning
      if(ch > 15 && ch < 35)  //check msg size
      {
        serial_buffer_rx[serial_buffer_level++] = ch;
        rx_state = PROT_RX_FRAME;
      }
      else
      {
        rx_state = PROT_RX_INIT;
      }
    }
    break;
    case PROT_RX_FRAME:
    {
      if(serial_buffer_level < sizeof(serial_buffer_rx))
      {
        // unscramble the frame
        if((ch_prev == 0xDB) && (ch == 0xDE)) 
        { // 0xDB 0xDE = 0x12
          serial_buffer_rx[serial_buffer_level - 1] = 0x12;
        }
        else if((ch_prev == 0xDB) && (ch == 0xDD)) 
        {// 0xDB 0xDD = 0xDB
          serial_buffer_rx[serial_buffer_level - 1] = 0xDB;
        }
        else
        {
          serial_buffer_rx[serial_buffer_level++] = ch;
        }

        if(ch == MSG_DELIM_CHAR)
        {
          rx_state = PROT_RX_FRAME_RECEIVED;

#if (PROT_DEBUG_RX_RAW != 0)
          if((rx_debug_raw_id_mask == 0) || ((rx_debug_raw_id_mask ^ serial_buffer_rx[1]) == 0))
          {
            // store debug info
            rx_debug_raw[rx_debug_raw_cnt].msg_id_size = serial_buffer_rx[1];
            rx_debug_raw[rx_debug_raw_cnt].msg_size    = serial_buffer_level;
            sw_timer_start((sw_timer *)&rx_debug_raw[rx_debug_raw_cnt].timestamp);
            uint8_t end_idx = (rx_debug_raw_cnt - 1) % (sizeof(rx_debug_raw) / sizeof(rx_debug_raw[0]) );
            rx_debug_raw[rx_debug_raw_cnt].time_diff = rx_debug_raw[rx_debug_raw_cnt].timestamp - rx_debug_raw[end_idx].timestamp;
            rx_debug_raw_cnt = (rx_debug_raw_cnt + 1) % (sizeof(rx_debug_raw) / sizeof(rx_debug_raw[0]) );
          }
#endif
        }
      }
      else
      {
        // immediately reset state machine
        rx_state = PROT_RX_INIT;
      }
    }
    break;
    case PROT_RX_FRAME_RECEIVED:
    {
      // do nothing here, wait for state reset
    }
    break;
    default: rx_state = PROT_RX_INIT; break;
  }

  // store last byte
  ch_prev = ch;
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief
//- **************************************************************************
static prot_states prot_analyze_frame(prot_states current_state)
{
  prot_states  res = current_state;
  uint16_t msg_size;
  bool frame_found = false;

  if(rx_state == PROT_RX_FRAME_RECEIVED)
  { // check it is correct frame 
    if(serial_buffer_rx[0] == MSG_DELIM_CHAR)
    {
      // extract frame size
      msg_size = serial_buffer_rx[1] + (CRC_SIZE + MSG_DELIM_SIZE);
      // check serial buffer is matching of frame size
      if((uint16_t)serial_buffer_level == msg_size)
      {
        for(size_t cfg_idx = 0; cfg_idx < (sizeof(prot_cfg) / sizeof(prot_cfg[0])); cfg_idx++)      
        { // first check frame is matching to request size
          if(serial_buffer_level == prot_cfg[cfg_idx].msg_req_size)
          { // compare frame
            if(0 == memcmp(serial_buffer_rx, prot_cfg[cfg_idx].msg_req_ptr, prot_cfg[cfg_idx].msg_req_cmp_size))
            {
              frame_found = true;
              res         = prot_cfg[cfg_idx].dest_state;
              tx_length   = prot_cfg[cfg_idx].msg_res_size;
              tx_msg_idx  = cfg_idx;
#ifdef PROT_DEBUG_PRINT
              if(prot_cfg[cfg_idx].debug_str != NULL)
              {
                PROT_PRINT(prot_cfg[cfg_idx].debug_str);
              }

              if(prot_cfg[cfg_idx].dump_bytes != false)
              {
                PROT_PRINT("PROT:RX:");
                for(uint8_t i = 0; i < serial_buffer_level; i++)
                {
                  PROT_PRINT("%02X ", serial_buffer_rx[i]);
                }
                PROT_PRINT("\r\n");
              }
#endif
#if (PROT_DEBUG_RX_ANALYZED != 0)
              if((rx_debug_analyzed_id_mask == 0) || ((rx_debug_analyzed_id_mask ^ serial_buffer_rx[1]) == 0))
              {
                // store debug info
                rx_debug_analyzed[rx_debug_analyzed_cnt].msg_id_size = serial_buffer_rx[1];
                rx_debug_analyzed[rx_debug_analyzed_cnt].msg_size    = serial_buffer_level;
                sw_timer_start((sw_timer *)&rx_debug_analyzed[rx_debug_analyzed_cnt].timestamp);
                rx_debug_analyzed_cnt = (rx_debug_analyzed_cnt + 1) % (sizeof(rx_debug_analyzed) /sizeof(rx_debug_analyzed[0]) );
              }
#endif
              // leave the loop
              break;
            }
          }
        }
      }
    }
    // reset rx state machine state after processing
    rx_state = PROT_RX_INIT;

    if(frame_found == false)
      discarded_frames_cnt++;
  }

  return res;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static void prot_assemble_data_frame(void)
{
  uint8_t serial_buffer_tmp[sizeof(msg_bms_data_res)] = {0};
  uint16_t soc_percent_x100  = bms_get_soc_x100();  // state of charge in percent * 100 
  uint16_t runtime_sec       = bms_get_runtime_seconds();
  bool     charger_connected = port_pin_get_input_level(CHARGER_CONNECTED_PIN);
  uint8_t  rolling_counter;
  uint32_t crc;

  // first copy rolling counter from serial buffer
  rolling_counter = serial_buffer_rx[BMS_MSG_ROLLING_COUNTER_IDX];

  // copy data response to serial buffer
  memcpy(serial_buffer_tmp, msg_bms_data_res, sizeof(msg_bms_data_res));
  // copy back rolling counter to response
  serial_buffer_tmp[BMS_MSG_ROLLING_COUNTER_IDX] = rolling_counter;
  // fill charger connected state
  serial_buffer_tmp[BMS_MSG_CHARGER_CONNECTED_IDX] = charger_connected;
  // fill trigger state
  serial_buffer_tmp[BMS_MSG_TRIGGER_IDX] = prot_trigger_state;
  // fill estimated runtime in seconds
  serial_buffer_tmp[BMS_MSG_BAT_RUNTIME_LO_IDX] = (uint8_t)((runtime_sec >> 0) & 0x00FF); // lsb first
  serial_buffer_tmp[BMS_MSG_BAT_RUNTIME_HI_IDX] = (uint8_t)((runtime_sec >> 8) & 0x00FF);

  // fill state of charge
  serial_buffer_tmp[BMS_MSG_SOC_LO_IDX] = (uint8_t)((soc_percent_x100 >> 0) & 0x00FF); // lsb first
  serial_buffer_tmp[BMS_MSG_SOC_HI_IDX] = (uint8_t)((soc_percent_x100 >> 8) & 0x00FF);
  // unknown field, chinese bms put SOC here 
  serial_buffer_tmp[BMS_MSG_SOC2_LO_IDX] = (uint8_t)((soc_percent_x100 >> 0) & 0x00FF); // lsb first
  serial_buffer_tmp[BMS_MSG_SOC2_HI_IDX] = (uint8_t)((soc_percent_x100 >> 8) & 0x00FF);
  
  // calculate crc
  crc = calc_crc32(serial_buffer_tmp, (sizeof(msg_bms_data_res) - (CRC_SIZE + MSG_DELIM_SIZE)));
  // fill crc into the message
  serial_buffer_tmp[(sizeof(msg_bms_data_res) - (CRC_SIZE + MSG_DELIM_SIZE)) + 0] = (uint8_t)((crc >> 0)  & 0x000000FFul);  // lsb first
  serial_buffer_tmp[(sizeof(msg_bms_data_res) - (CRC_SIZE + MSG_DELIM_SIZE)) + 1] = (uint8_t)((crc >> 8)  & 0x000000FFul);
  serial_buffer_tmp[(sizeof(msg_bms_data_res) - (CRC_SIZE + MSG_DELIM_SIZE)) + 2] = (uint8_t)((crc >> 16) & 0x000000FFul);
  serial_buffer_tmp[(sizeof(msg_bms_data_res) - (CRC_SIZE + MSG_DELIM_SIZE)) + 3] = (uint8_t)((crc >> 24) & 0x000000FFul);

  tx_length = prot_scramble_frame(serial_buffer_tx, serial_buffer_tmp, sizeof(serial_buffer_tx), sizeof(msg_bms_data_res));
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static uint8_t prot_scramble_frame(uint8_t * dest, const uint8_t * src, size_t dest_len, size_t src_len)
{
  size_t length = 0;
  
  if(src_len > (MSG_DELIM_SIZE + 1) && dest_len >= src_len)
  {
    // copy first 6 bytes including frame delimiter, which are fixed
    memcpy(dest, src, 6);

    for (size_t i = length = 6; i < (src_len - 1); i++, length++)
    {
      if((length + 2) < (uint8_t)dest_len)
      {
        if(src[i] == 0x12)
        {
          dest[length]   = (uint8_t)((BYTE_0x12_REPLACEMENT >> 0) & 0xFF);
          dest[length++] = (uint8_t)((BYTE_0x12_REPLACEMENT >> 8) & 0xFF);
        }
        else if(src[i] == 0xDB)
        {
          dest[length]   = (uint8_t)((BYTE_0xDB_REPLACEMENT >> 0) & 0xFF);
          dest[length++] = (uint8_t)((BYTE_0xDB_REPLACEMENT >> 8) & 0xFF);
        }
        else
        {
          dest[length] = src[i];
        }
      }
    }

    // copy delimiter - end of frame
    dest[length++] = src[(src_len - 1)];
  }

  return length;
}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/