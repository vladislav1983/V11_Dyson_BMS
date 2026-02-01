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
#define BMS_MSG_ROLLING_COUNTER_IDX         8
#define BMS_MSG_TRIGGER_STATE_IDX           15
#define BMS_MSG_CHARGER_CONNECTED_IDX       22
#define BMS_MSG_SOC_LO_IDX                  29
#define BMS_MSG_SOC_HI_IDX                  30
#define BMS_MSG_BAT_RUNTIME_LO_IDX          38
#define BMS_MSG_BAT_RUNTIME_HI_IDX          39
#define BMS_MSG_SOC2_LO_IDX                 47
#define BMS_MSG_SOC2_HI_IDX                 48

#define MODE_BOOST                          0x1167
#define MODE_MED                            0xC80D
#define MODE_ECO                            0x2823

#define MSG_DELIM_CHAR                      0x12
#define MSG_DELIM_SIZE                      1
#define CRC_SIZE                            4
#define COMM_TIMEOUT                        (5 / SW_TIMER_TICK_MS)
/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/
typedef enum
{
  PROT_INIT,
  PROT_WAIT_FRAME,
  PROT_TX_HANDSHAKE,
  PROT_TX_DATA_FRAME,
  PROT_TX_CHAL_RESP,
  PROT_WAIT_STATE,
}prot_states;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
static sw_timer com_timeout_timer = 0;
static sw_timer wait_state_timer = 0;
static sw_timer wait_state_timer_ms = 0;
static sw_timer session_timer = 0;
static bool     prot_trigger_state = false;
static bool     prot_vacuum_connected = false;
uint8_t serial_buffer[64] = {0};
static uint8_t serial_buffer_level = 0;
static prot_states prot_state = PROT_WAIT_FRAME;
static prot_states prot_state_next = PROT_WAIT_FRAME;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/
static const uint8_t msg_vac_handshake_req_0[] = {0x12, 0x1A, 0x00, 0x31, 0x00, 0xC0, 0xFF, 0x03, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0xFF, 0x06, 0xDA, 0xEE, 0x2D, 0x12};
static const uint8_t msg_vac_handshake_req_1[] = {0x12, 0x22, 0x00, 0xE4, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0x13, 0x02, 0x10, 0x02, 0x81, 0x02, 0x10, 0x01, 0x02, 0x02, 0x10, 0x15, 0x81, 0x02, 0x10, 0x14, 0x81, 0x16, 0x82, 0x00, 0x00, 0x01, 0x00, 0x68, 0xCC, 0x78, 0x60, 0x12};
static const uint8_t msg_bms_handshake_res[]   = {0x12, 0x30, 0x00, 0x2B, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x13, 0x01, 0x10, 0x02, 0x81, 0x02, 0x00, 0xB8, 0x02, 0x01, 0x10, 0x01, 0x02, 0x04, 0x00, 0xC0, 0x27, 0x09, 0x00, 0x01, 0x10, 0x15, 0x81, 0x02, 0x00, 0xD8, 0x72, 0x01, 0x10, 0x14, 0x81, 0x02, 0x00, 0x76, 0x48, 0x01, 0x00, 0x21, 0x32, 0x21, 0xD1, 0x12};

// request/response between vacuum and bms (byte 9 is rolling counter)
static const uint8_t msg_vac_data_req[]    = {0x12, 0x21, 0x00, 0x53, 0x01, 0xC0, 0x01, 0x02, 0x04, 0x02, 0x10, 0x00, 0x81, 0x02, 0x10, 0x01, 0x21, 0x02, 0x10, 0x0D, 0x25, 0x02, 0x10, 0x02, 0x22, 0x02, 0x10, 0x05, 0x81, 0x02, 0x10, 0x06, 0x81, 0x96, 0xDD, 0x33, 0x81, 0x12};
// bms response of 0x21 is 0x38 (byte 9 is rolling counter, must be copied from 0x21)
static const uint8_t msg_bms_data_resp[]   = {0x12, 0x38, 0x00, 0xC1, 0x01, 0xC0, 0x02, 0x01, 0x04, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x01, 0x01, 0x10, 0x01, 0x21, 0x01, 0x00, 0x00, 0x01, 0x10, 0x0D, 0x25, 0x02, 0x00, 0xFA, 0x25, 0x01, 0x10, 0x02, 0x22, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x05, 0x81, 0x02, 0x00, 0x67, 0x11, 0x01, 0x10, 0x06, 0x81, 0x01, 0x00, 0x01, 0x7F, 0x52, 0x4C, 0xE7, 0x12};

// challenge/response message between vacuum and bms - data is not changed
static const uint8_t msg_vac_chal_req[]    = {0x12, 0x16, 0x00, 0xAE, 0x00, 0xC0, 0x01, 0x03, 0x01, 0x07, 0x10, 0x11, 0x00, 0x82, 0x40, 0x42, 0x0F, 0x00, 0x02, 0x10, 0x00, 0x81, 0xC5, 0x6A, 0xE2, 0x0B, 0x12};
// bms response of msg 0x16 is 0x1B
static const uint8_t msg_bms_chal_resp[]   = {0x12, 0x1B, 0x00, 0x5C, 0x01, 0xC0, 0x03, 0x01, 0x01, 0x08, 0x10, 0x11, 0x01, 0x82, 0x20, 0xDB, 0xDE, 0x0A, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x01, 0xA9, 0x82, 0xBE, 0x65, 0x12};

// unknown vacuum message
static const uint8_t vac_chal[]   = {0x12, 0x15, 0x00, 0x19, 0x01, 0xC0, 0x01, 0x02, 0x2D, 0x02, 0x10, 0x0B, 0x25, 0x02, 0x10, 0x0C, 0x25, 0x02, 0x10, 0x14, 0x81, 0xA6, 0x41, 0x73, 0x4D, 0x12};

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static bool prot_check_valid_handshake(const uint8_t* handshake_ptr, uint8_t handshake_size);
static prot_states prot_analyze_data_frame(uint8_t* frame_ptr, uint8_t frame_size, prot_states current_state);
static void prot_assemble_data_frame(void);

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief
//- **************************************************************************
void prot_init(void)
{
  prot_state = PROT_WAIT_FRAME;
  com_timeout_timer = 0;
  wait_state_timer = 0;
  wait_state_timer_ms = 0;
  serial_buffer_level = 0;
  session_timer = 0;
  prot_trigger_state = false;
  prot_vacuum_connected = false;
  sw_timer_start(&com_timeout_timer);
  sw_timer_start(&session_timer);

  if(sizeof(msg_bms_data_resp) > sizeof(serial_buffer) )
  {
    assert(false);
    while(1);
  }
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
void prot_mainloop(void)
{
  switch(prot_state)
  {
    //------------------------------------------------------------------------
    case PROT_INIT:
    {
      prot_state = PROT_WAIT_FRAME;
      com_timeout_timer = 0;
      wait_state_timer = 0;
      wait_state_timer_ms = 0;
      serial_buffer_level = 0;
      session_timer = 0;
      prot_trigger_state = false;
      
      sw_timer_start(&com_timeout_timer);
      sw_timer_start(&session_timer);

      if(prot_vacuum_connected)
      {
        leds_blink_error_led(500);
        leds_blink_error_led(500);
      }

      prot_vacuum_connected = false;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_WAIT_FRAME:
    {
      prot_states prot_state_req = prot_analyze_data_frame(serial_buffer, serial_buffer_level, prot_state);

      if(prot_state != prot_state_req)
      {
        // tx data after some time after request
        wait_state_timer_ms = 1;
        prot_state          = PROT_WAIT_STATE;
        prot_state_next     = prot_state_req;
        sw_timer_start(&session_timer);
      }
      else
      {
        if(sw_timer_is_elapsed(&session_timer, 500))
        {
          prot_state = PROT_INIT;
        }
      }
    }
    break;
    //------------------------------------------------------------------------
    case PROT_TX_HANDSHAKE:
    {
      serial_send((uint8_t*)msg_bms_handshake_res, sizeof(msg_bms_handshake_res));
      sw_timer_start(&wait_state_timer);
      serial_buffer_level = 0; 
      wait_state_timer_ms = 10;
      prot_state          = PROT_WAIT_STATE;
      prot_state_next     = PROT_WAIT_FRAME;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_TX_DATA_FRAME:
    {
      prot_assemble_data_frame();
      serial_send(serial_buffer, sizeof(msg_bms_data_resp));
      sw_timer_start(&wait_state_timer);
      wait_state_timer_ms = 1;
      prot_state          = PROT_WAIT_STATE;
      prot_state_next     = PROT_WAIT_FRAME;
      prot_vacuum_connected = true;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_TX_CHAL_RESP:
    {
      serial_send((uint8_t*)msg_bms_chal_resp, sizeof(msg_bms_chal_resp));
      sw_timer_start(&wait_state_timer);
      wait_state_timer_ms = 1;
      prot_state          = PROT_WAIT_STATE;
      prot_state_next     = PROT_WAIT_FRAME;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_WAIT_STATE:
    {
      if(false != sw_timer_is_elapsed(&wait_state_timer, wait_state_timer_ms))
      {
        prot_state = prot_state_next;
      }
    }
    break;
    //------------------------------------------------------------------------
    default: break;
  }
}

//- **************************************************************************
//! \brief 
//- **************************************************************************
void prot_serial_rx_callback(uint8_t ch)
{
  if(prot_state == PROT_WAIT_FRAME )
  {
    if(serial_buffer_level < sizeof(serial_buffer))
    {
      serial_buffer[serial_buffer_level] = ch;
      serial_buffer_level++;

      // reset sw timer on every rx
      sw_timer_start(&com_timeout_timer);
    }
  }
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief 
//- **************************************************************************
static bool prot_check_valid_handshake(const uint8_t* handshake_ptr, uint8_t handshake_size)
{
  bool res = false;

  if(false != sw_timer_is_elapsed(&com_timeout_timer, COMM_TIMEOUT))
  {
    if(MSG_DELIM_CHAR == serial_buffer[0] && serial_buffer_level > 0)
    {
      if(0 == memcmp(serial_buffer, handshake_ptr, serial_buffer_level))
      {
        res = true;
      }
      else
      {
        serial_buffer_level = 0;
      }
    }
    else
    {
      serial_buffer_level = 0;
    }
  }

  return res;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static prot_states prot_analyze_data_frame(uint8_t* frame_ptr, uint8_t frame_size, prot_states current_state)
{
  prot_states res = current_state;

  if(sw_timer_is_elapsed(&com_timeout_timer, COMM_TIMEOUT))
  {
    if(sizeof(msg_vac_data_req) == frame_size)
    {// could be data frame
      // check the beginning of 0x21 message, without rolling counter  
      if(0 == memcmp(&frame_ptr[0], &msg_vac_data_req[0], BMS_MSG_ROLLING_COUNTER_IDX))
      { 
        if(0 == memcmp(&frame_ptr[BMS_MSG_ROLLING_COUNTER_IDX + 1], &msg_vac_data_req[BMS_MSG_ROLLING_COUNTER_IDX + 1], (sizeof(msg_vac_data_req) - (BMS_MSG_ROLLING_COUNTER_IDX + 1) - (CRC_SIZE + MSG_DELIM_SIZE))))
        {// check the rest of the data, skipping rolling counter, CRC32 and frame delimiter
          // need to transmit data
          res = PROT_TX_DATA_FRAME;
        }
      }
    }
    else if(sizeof(msg_vac_handshake_req_1) == frame_size)
    { // could be initial handshake
      if(0 == memcmp(&frame_ptr[0], msg_vac_handshake_req_1, frame_size))
      { // it is handshake, bms should respond witj 0x12 0x30 ...
        // need to transmit data
        res = PROT_TX_HANDSHAKE;
      }
    }
    else if(sizeof(msg_vac_chal_req) == frame_size)
    {// could be challenge/response frame
      if(0 == memcmp(frame_ptr, msg_vac_chal_req, frame_size))
      {// it is challenge/response
        // need to respond to challenge
        res = PROT_TX_CHAL_RESP;
      }
    }
    else if(sizeof(vac_chal) == frame_size)
    { // could be vacuum challenge, unknown for now
      if(0 == memcmp(frame_ptr, vac_chal, 8)) // no need to compare whole message, it contains rolling counter at byte 9
      {
        // stay into the current state, we do not respond to this message, only session timer reset
      }
    }
    else
    {
      // no valid data frame
      serial_buffer_level = 0;
    }
  }

  return res;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static void prot_assemble_data_frame(void)
{
  uint16_t soc_percent_x10   = bms_get_soc_x10();
  uint16_t runtime_sec       = bms_get_runtime_seconds();
  bool     charger_connected = port_pin_get_input_level(CHARGER_CONNECTED_PIN);
  uint32_t crc;

  // first copy rolling counter from serial buffer
  uint8_t rolling_counter = serial_buffer[BMS_MSG_ROLLING_COUNTER_IDX];
  // copy data response to serial buffer
  memcpy(serial_buffer, msg_bms_data_resp, sizeof(msg_bms_data_resp));
  // copy back rolling counter to response
  serial_buffer[BMS_MSG_ROLLING_COUNTER_IDX] = rolling_counter;
  // fill charger connected state
  serial_buffer[BMS_MSG_CHARGER_CONNECTED_IDX] = charger_connected;

  if(charger_connected == false && prot_trigger_state != false)
  {
    // fill trigger state
    serial_buffer[BMS_MSG_TRIGGER_STATE_IDX] = prot_trigger_state;
    // fill estimated runtime in seconds
    serial_buffer[BMS_MSG_BAT_RUNTIME_LO_IDX] = (uint8_t)((runtime_sec >> 0) & 0x00FF); // lsb first
    serial_buffer[BMS_MSG_BAT_RUNTIME_HI_IDX] = (uint8_t)((runtime_sec >> 8) & 0x00FF);
  }
  else
  {
    // fill trigger state
    serial_buffer[BMS_MSG_TRIGGER_STATE_IDX] = 0;
    // fill estimated runtime in seconds
    serial_buffer[BMS_MSG_BAT_RUNTIME_LO_IDX] = 0;
    serial_buffer[BMS_MSG_BAT_RUNTIME_HI_IDX] = 0;
  }

  // fill state of charge
  serial_buffer[BMS_MSG_SOC_LO_IDX] = (uint8_t)((soc_percent_x10 >> 0) & 0x00FF); // lsb first
  serial_buffer[BMS_MSG_SOC_HI_IDX] = (uint8_t)((soc_percent_x10 >> 8) & 0x00FF);
  // set mode ... to be implemented 
  serial_buffer[BMS_MSG_SOC2_LO_IDX] = (uint8_t)((soc_percent_x10 >> 0) & 0x00FF); // lsb first
  serial_buffer[BMS_MSG_SOC2_HI_IDX] = (uint8_t)((soc_percent_x10 >> 8) & 0x00FF);
  
  // calculate crc
  crc = calc_crc32(serial_buffer, (sizeof(msg_bms_data_resp) - (CRC_SIZE + MSG_DELIM_SIZE)));
  // fill crc into the message
  serial_buffer[(sizeof(msg_bms_data_resp) - (CRC_SIZE + MSG_DELIM_SIZE)) + 0] = (uint8_t)((crc >> 0)  & 0x000000FFul);  // lsb first
  serial_buffer[(sizeof(msg_bms_data_resp) - (CRC_SIZE + MSG_DELIM_SIZE)) + 1] = (uint8_t)((crc >> 8)  & 0x000000FFul);
  serial_buffer[(sizeof(msg_bms_data_resp) - (CRC_SIZE + MSG_DELIM_SIZE)) + 2] = (uint8_t)((crc >> 16) & 0x000000FFul);
  serial_buffer[(sizeof(msg_bms_data_resp) - (CRC_SIZE + MSG_DELIM_SIZE)) + 3] = (uint8_t)((crc >> 24) & 0x000000FFul);
}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/