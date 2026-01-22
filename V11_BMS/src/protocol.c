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
#define MSG_ROLLING_COUNTER_IDX         9
#define MSG_DELIM_CHAR                  0x12
#define CRC_SIZE                        4
#define COMM_TIMEOUT                    (5 / SW_TIMER_TICK_MS)
/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/
typedef enum
{
  PROT_WAIT_FRAME,
  PROT_WAIT_HANDSHAKE_1,
  PROT_WAIT_HANDSHAKE_2,
  PROT_TX_HANDSHAKE,
  PROT_MAIN,
  PROT_WAIT_STATE,
}prot_states;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
static sw_timer com_timeout_timer = 0;
static sw_timer wait_state_timer = 0;
static sw_timer wait_state_timer_value = 0;
static sw_timer valid_data_frame_timeout_timer = 0;

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

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static bool prot_check_valid_handshake(uint8_t* frame_ptr, const uint8_t* handshake_ptr, uint8_t frame_size, uint8_t handshake_size);

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
  wait_state_timer_value = 0;
  valid_data_frame_timeout_timer = 0;
  serial_buffer_level = 0;
  sw_timer_start(&wait_state_timer);
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void prot_mainloop(void)
{
  //uint8_t tx[sizeof(msg_vac_handshake_req_1)] = {0xFF};

  if(false != sw_timer_is_elapsed(&wait_state_timer, 250))
  {
    //serial_send(tx, sizeof(tx));
    sw_timer_start(&wait_state_timer);
  }

  return;

  switch(prot_state)
  {
    //------------------------------------------------------------------------
    case PROT_WAIT_FRAME:
    {
      if(false != prot_check_valid_handshake(serial_buffer, msg_vac_handshake_req_0, serial_buffer_level, sizeof(msg_vac_handshake_req_0)))
      {
        prot_state = PROT_WAIT_HANDSHAKE_2;
      }

      serial_buffer_level = 0;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_WAIT_HANDSHAKE_2:
    {
      if(false != prot_check_valid_handshake(serial_buffer, msg_vac_handshake_req_1, serial_buffer_level, sizeof(msg_vac_handshake_req_1)))
      {
        prot_state = PROT_TX_HANDSHAKE;
      }

      serial_buffer_level = 0;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_TX_HANDSHAKE:
    {
      serial_send(msg_bms_handshake_res, sizeof(msg_bms_handshake_res));
      sw_timer_start(&wait_state_timer);
      wait_state_timer_value = 5;
      prot_state             = PROT_WAIT_STATE;
      prot_state_next        = PROT_MAIN;
    }
    break;
    //------------------------------------------------------------------------
    case PROT_MAIN:
    {
      if(false == sw_timer_is_elapsed(&valid_data_frame_timeout_timer, 200))
      {
        if(0 == memcmp(&serial_buffer[0], &msg_vac_data_req[0], (MSG_ROLLING_COUNTER_IDX - 1)))
        { // check the beginning of 0x21 message, without rolling counter
          if(0 == memcmp(&serial_buffer[MSG_ROLLING_COUNTER_IDX], &msg_vac_data_req[MSG_ROLLING_COUNTER_IDX], (sizeof(msg_vac_data_req) - MSG_ROLLING_COUNTER_IDX - (CRC_SIZE + 1))))
          { // check the rest of the data, skipping rolling counter, CRC32 and frame delimiter

          }
        }
      }
      else
      {
        // no valid data frame is received, reset state machine
        prot_state = PROT_WAIT_FRAME;
      }

    }
    break;
    //------------------------------------------------------------------------
    case PROT_WAIT_STATE:
    {
      if(false != sw_timer_is_elapsed(&wait_state_timer, wait_state_timer_value))
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
  if(serial_buffer_level < sizeof(serial_buffer))
  {
    serial_buffer[serial_buffer_level] = ch;
    serial_buffer_level++;
  }

  // reset sw timer on every rx
  if(PROT_WAIT_FRAME == prot_state)
  {
    sw_timer_start(&com_timeout_timer);
  }
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief 
//- **************************************************************************
static bool prot_check_valid_handshake(uint8_t* frame_ptr, const uint8_t* handshake_ptr, uint8_t frame_size, uint8_t handshake_size)
{
  bool res = false;

  if((frame_size > 0) && (false != sw_timer_is_elapsed(&com_timeout_timer, COMM_TIMEOUT)))
  {
    if(frame_size == handshake_size)
    {
      if(0 == memcmp(frame_ptr, handshake_ptr, frame_size))
      {
        res = true;
      }
    }
  }

  return res;
}
/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/