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
    DEFINITION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

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
#define TX_WAIT_TIME                        (2 / SW_TIMER_TICK_MS)

#define BYTE_0x12_REPLACEMENT               (0xDEDB)  // 0x12 is replaced with 0xDB 0xDE to avoid framing error
#define BYTE_0xDB_REPLACEMENT               (0xDDDB)  // 0xDB is replaced with 0xDB 0xDD to avoid framing error

#ifdef SERIAL_DEBUG
  #define PROT_PRINT(...) \
  { \
    sprintf(debug_msg_buffer, __VA_ARGS__); \
    serial_debug_send_message(debug_msg_buffer);  \
  }
#else
  #define PROT_PRINT(X)
#endif

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/
typedef enum
{
  PROT_INIT,
  PROT_IDLE,
  PROT_WAIT_FRAME,
  PROT_TX_DATA_FRAME,
  PROT_TX_FRAME,
  PROT_TX_TRIGGER_RESP,
  PROT_WAIT_STATE,
  PROT_DISABLED,
}prot_states;

typedef struct
{
  const uint8_t* msg_req_ptr;
  const uint8_t* msg_res_ptr;
  uint8_t        msg_req_cmp_size;
  uint8_t        msg_res_size;
  prot_states    dest_state;
#ifdef SERIAL_DEBUG
  bool           dump_bytes;
  const char*    debug_str;
#endif
}prot_cfg_t;

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
static bool    frame_flag          = false;
static uint8_t tx_length           = 0;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/
//static const uint8_t msg_vac_handshake_req_0[] = {0x12, 0x1A, 0x00, 0x31, 0x00, 0xC0, 0xFF, 0x03, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0xFF, 0x06, 0xDA, 0xEE, 0x2D, 0x12};
static const uint8_t msg_vac_handshake_req[] = {0x12, 0x22, 0x00, 0xE4, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0x13, 0x02, 0x10, 0x02, 0x81, 0x02, 0x10, 0x01, 0x02, 0x02, 0x10, 0x15, 0x81, 0x02, 0x10, 0x14, 0x81, 0x16, 0x82, 0x00, 0x00, 0x01, 0x00, 0x68, 0xCC, 0x78, 0x60, 0x12};
static const uint8_t msg_bms_handshake_res[] = {0x12, 0x30, 0x00, 0x2B, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x13, 0x01, 0x10, 0x02, 0x81, 0x02, 0x00, 0xB8, 0x02, 0x01, 0x10, 0x01, 0x02, 0x04, 0x00, 0xC0, 0x27, 0x09, 0x00, 0x01, 0x10, 0x15, 0x81, 0x02, 0x00, 0xD8, 0x72, 0x01, 0x10, 0x14, 0x81, 0x02, 0x00, 0x76, 0x48, 0x01, 0x00, 0x21, 0x32, 0x21, 0xD1, 0x12};

// request/response between vacuum and bms (byte 9 is rolling counter)
static const uint8_t msg_vac_data_req[]    = {0x12, 0x21, 0x00, 0x53, 0x01, 0xC0, 0x01, 0x02, 0x04, 0x02, 0x10, 0x00, 0x81, 0x02, 0x10, 0x01, 0x21, 0x02, 0x10, 0x0D, 0x25, 0x02, 0x10, 0x02, 0x22, 0x02, 0x10, 0x05, 0x81, 0x02, 0x10, 0x06, 0x81, 0x96, 0xDD, 0x33, 0x81, 0x12};
// bms response of 0x21 is 0x38 (byte 9 is rolling counter, must be copied from 0x21)
static const uint8_t msg_bms_data_res[]   = {0x12, 0x38, 0x00, 0xC1, 0x01, 0xC0, 0x02, 0x01, 0x04, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x01, 0x01, 0x10, 0x01, 0x21, 0x01, 0x00, 0x00, 0x01, 0x10, 0x0D, 0x25, 0x02, 0x00, 0xFA, 0x25, 0x01, 0x10, 0x02, 0x22, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x05, 0x81, 0x02, 0x00, 0x67, 0x11, 0x01, 0x10, 0x06, 0x81, 0x01, 0x00, 0x01, 0x7F, 0x52, 0x4C, 0xE7, 0x12};

// challenge/response message between vacuum and bms - data is not changed
static const uint8_t msg_vac_trig_req[]           = {0x12, 0x16, 0x00, 0xAE, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0x11, 0x00, 0x82, 0x40, 0x42, 0x0F, 0x00, 0x02, 0x10, 0x00, 0x81, 0xC5, 0x6A, 0xE2, 0x0B, 0x12};
//static const uint8_t msg_vac_trigger_req_2[]    = {0x12, 0x16, 0x00, 0xAE, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0xDB, 0xDE, 0x00, 0x82, 0x98, 0x3A, 0x00, 0x00, 0x02, 0x10, 0x00, 0x81, 0xD7, 0xBA, 0xB5, 0x68, 0x12};
//static const uint8_t msg_vac_trigger_req_3[]    = {0x12, 0x16, 0x00, 0xAE, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0xDB, 0xDE, 0x00, 0x82, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x81, 0x86, 0x38, 0xD6, 0x01, 0x12};
//static const uint8_t msg_vac_trig_charging_req[]  = {0x12, 0x16, 0x00, 0xAE, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0x11, 0x00, 0x82, 0x00, 0x00, 0x00, 0x00, 0x02, 0x10, 0x00, 0x81, 0x48, 0x54, 0x1C, 0xBC, 0x12};
// bms response of msg 0x16 is 0x1B
// vac running
static const uint8_t msg_bms_trig_on_res[]   = {0x12, 0x1B, 0x00, 0x5C, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x11, 0x01, 0x82, 0x20, 0xDB, 0xDE, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x01, 0xA9, 0x82, 0xBE, 0x65, 0x12};
// vac not running
static const uint8_t msg_bms_trig_off_res[]  = {0x12, 0x1B, 0x00, 0x5C, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x11, 0x01, 0x82, 0x20, 0xDB, 0xDE, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x00, 0xE8, 0xB3, 0xA5, 0x7C, 0x12};
// some ... 
static const uint8_t msg_bms_trig_charging_res[]  = {0x12, 0x1B, 0x00, 0x5C, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x11, 0x01, 0x82, 0x98, 0x3A, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x00, 0x71, 0x76, 0x13, 0x64, 0x12};
//static const uint8_t msg_bms_trig_charging_off_res[] = {0x12, 0x1B, 0x00, 0x5C, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x11, 0x01, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x00, 0xC0, 0x30, 0xAD, 0x8F, 0x12};

// unknown vacuum message
//static const uint8_t vac_chal[]   = {0x12, 0x15, 0x00, 0x19, 0x01, 0xC0, 0x01, 0x02, 0x2D, 0x02, 0x10, 0x0B, 0x25, 0x02, 0x10, 0x0C, 0x25, 0x02, 0x10, 0x14, 0x81, 0xA6, 0x41, 0x73, 0x4D, 0x12};

static const prot_cfg_t prot_cfg[] = 
{
  {
    .msg_req_ptr      = msg_vac_data_req,
    .msg_res_ptr      = serial_buffer_tx,
    .msg_req_cmp_size = 8,
    .msg_res_size     = sizeof(msg_bms_data_res),
    .dest_state       = PROT_TX_DATA_FRAME,
#ifdef SERIAL_DEBUG
    .dump_bytes       = false,
    .debug_str        = NULL,
#endif
  },
//  {
//    .msg_req_ptr      = msg_vac_trig_req,
//    .msg_res_ptr      = NULL,
//    .msg_req_cmp_size = sizeof(msg_vac_trig_req),
//    .msg_res_size     = 0,
//    .dest_state       = PROT_TX_TRIGGER_RESP,
//    #ifdef SERIAL_DEBUG
//    .dump_bytes       = false,
//    .debug_str        = NULL,
//    #endif
//  },
//  {
//    .msg_req_ptr      = msg_vac_trig_charging_req,
//    .msg_res_ptr      = NULL,
//    .msg_req_cmp_size = sizeof(msg_vac_trig_charging_req),
//    .msg_res_size     = 0,
//    .dest_state       = PROT_TX_TRIGGER_RESP,
//    #ifdef SERIAL_DEBUG
//    .dump_bytes       = false,
//    .debug_str        = "PROT:TRIG_CHARGING\r\n",
//    #endif
//  },
  {
    .msg_req_ptr      = msg_vac_trig_req,
    .msg_res_ptr      = NULL,
    .msg_req_cmp_size = 11u,
    .msg_res_size     = 0,
    .dest_state       = PROT_TX_TRIGGER_RESP,
#ifdef SERIAL_DEBUG
    .dump_bytes       = false,
    .debug_str        = NULL,
#endif
  },
  {
    .msg_req_ptr      = msg_vac_handshake_req,
    .msg_res_ptr      = msg_bms_handshake_res,
    .msg_req_cmp_size = sizeof(msg_vac_handshake_req),
    .msg_res_size     = sizeof(msg_bms_handshake_res),
    .dest_state       = PROT_TX_FRAME,
#ifdef SERIAL_DEBUG
    .dump_bytes       = false,
    .debug_str        = "PROT:HANDSHAKE_RECEIVED\r\n",
#endif
  },
};

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static prot_states prot_analyze_frame(prot_states current_state);
static void prot_assemble_data_frame(void);
static uint8_t prot_scramble_frame(uint8_t * dest, const uint8_t * src, size_t dest_len, size_t src_len);
static uint8_t prot_unscramble_frame(uint8_t * dest, const uint8_t * src, size_t dest_len, size_t src_len);

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
  PROT_PRINT("PROT:RESET TO PROT_INIT\r\n");
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
      PROT_PRINT("PROT_INIT\r\n");
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
    case PROT_TX_DATA_FRAME:
    {
      prot_assemble_data_frame();
    }
    // no break here
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
      {
        serial_send((uint8_t*)msg_bms_trig_on_res, sizeof(msg_bms_trig_on_res));
      }
      else
      {
        if(charger_connected == false)
        {
          serial_send((uint8_t*)msg_bms_trig_off_res, sizeof(msg_bms_trig_off_res));
        }
        else
        {
          serial_send((uint8_t*)msg_bms_trig_charging_res, sizeof(msg_bms_trig_charging_res));
        }
      }

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

  // end of frame detected, to be processed ...
  frame_flag           = ((ch == MSG_DELIM_CHAR) && (ch_prev != MSG_DELIM_CHAR));
  // new frame detected, reset buffer level
  serial_buffer_level *= (uint8_t)!((ch == MSG_DELIM_CHAR) && (ch_prev == MSG_DELIM_CHAR));

  if(serial_buffer_level < sizeof(serial_buffer_rx))
  {
    serial_buffer_rx[serial_buffer_level] = ch;
    serial_buffer_level++;
  }
  else
  {
    frame_flag = false;
  }

  // store current byte
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
  prot_states res = current_state;

  if(frame_flag != false)
  {
    for(uint8_t cfg_idx = 0; cfg_idx < (sizeof(prot_cfg) / sizeof(prot_cfg[0])); cfg_idx++)
    {
      if(serial_buffer_level > (MSG_DELIM_SIZE + 1) && serial_buffer_rx[serial_buffer_level - 1] == MSG_DELIM_CHAR)
      {
        if(0 == memcmp(serial_buffer_rx, prot_cfg[cfg_idx].msg_req_ptr, prot_cfg[cfg_idx].msg_req_cmp_size))
        {
          res        = prot_cfg[cfg_idx].dest_state;
          tx_msg_idx = cfg_idx;
          tx_length  = prot_cfg[tx_msg_idx].msg_res_size;
        }
      }
      
      if(res != current_state)
      {
#ifdef SERIAL_DEBUG
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
        // leave the loop for processing 
        break;
      }
    }

    frame_flag = false;
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
  uint16_t rolling_counter_u16 = (uint16_t)serial_buffer_rx[BMS_MSG_ROLLING_COUNTER_IDX] | ((uint16_t)serial_buffer_rx[BMS_MSG_ROLLING_COUNTER_IDX + 1] << 8);
  
  if(BYTE_0x12_REPLACEMENT == rolling_counter_u16)
  {
    rolling_counter = 0x12;
  }
  else if(BYTE_0xDB_REPLACEMENT == rolling_counter_u16)
  {
    rolling_counter = 0xDB;
  }
  else
  {
    rolling_counter = serial_buffer_rx[BMS_MSG_ROLLING_COUNTER_IDX];
  }

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
//- **************************************************************************
//! \brief
//- **************************************************************************
static uint8_t prot_unscramble_frame(uint8_t * dest, const uint8_t * src, size_t dest_len, size_t src_len)
{
  size_t length = 0;

  if(src_len > (MSG_DELIM_SIZE + 1) && dest_len >= (src_len - 1))
  {
    // copy first 6 bytes including frame delimiter, which are fixed
    memcpy(dest, src, 6);

    for (size_t i = length = 6; i < (src_len - 1); i++, length++)
    {
      if((i + 1) < (src_len - 1))
      {
        uint16_t word_u16 = (uint16_t)src[i] | ((uint16_t)src[i + 1] << 8);

        if(BYTE_0x12_REPLACEMENT == word_u16)
        {
          dest[length] = 0x12;
          i++;
        }
        else if(BYTE_0xDB_REPLACEMENT == word_u16)
        {
          dest[length] = 0xDB;
          i++;
        }
        else
        {
          dest[length] = src[i];
        }
      }
      else
      {
        dest[length] = src[i];
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