/*
 * serial_debug.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */ 

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "serial_debug.h"
#include "sw_timer.h"
#ifdef SERIAL_DEBUG
#include <string.h>
#endif
#include "eeprom_handler.h"

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
#define DEBUG_QUEUE_SIZE  256

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
static struct usart_module debug_usart;
static char debug_queue[DEBUG_QUEUE_SIZE];
static volatile uint16_t queue_head = 0;  // write index
static volatile uint16_t queue_tail = 0;  // read index
#endif

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/
extern volatile struct eeprom_data eeprom_data;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief 
//- **************************************************************************
void serial_debug_init()
{
#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
  struct usart_config config_usart;
  usart_get_config_defaults(&config_usart);
  
  //Load the necessary settings into the config struct.
  config_usart.baudrate    = 115200ul;
  config_usart.mux_setting = USART_RX_3_TX_2_XCK_3 ;
  config_usart.parity      = USART_PARITY_NONE;
  config_usart.pinmux_pad0 = PINMUX_UNUSED;
  config_usart.pinmux_pad1 = PINMUX_UNUSED;
  config_usart.pinmux_pad2 = PINMUX_PA10C_SERCOM0_PAD2;
  config_usart.pinmux_pad3 = PINMUX_PA11C_SERCOM0_PAD3;
  
  //Init the UART
  while (usart_init(&debug_usart,SERCOM0, &config_usart) != STATUS_OK) { }
  //Enable
  usart_enable(&debug_usart);
#endif
  queue_head = 0;  // write index
  queue_tail = 0;  // read index
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void serial_debug_send_message(const char *msg)
{
#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
  while (*msg)
  {
    uint16_t next_head = (queue_head + 1) % DEBUG_QUEUE_SIZE;
    if (next_head == queue_tail)
    {
      break;  // queue full, drop remaining characters
    }
    debug_queue[queue_head] = *msg++;
    queue_head = next_head;
  }
#endif
}

//- **************************************************************************
//! \brief Send one character from the queue. Call repeatedly from main loop.
//- **************************************************************************
void serial_debug_process(void)
{
#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
  if (queue_tail != queue_head)
  {
    usart_write_buffer_wait(&debug_usart, (const uint8_t*)&debug_queue[queue_tail], 1);
    queue_tail = (queue_tail + 1) % DEBUG_QUEUE_SIZE;
  }
#endif
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void serial_debug_send_cell_voltages(void)
{
#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
  char tmp[30];
  uint16_t *cell_voltages = bq7693_get_cell_voltages();
  
  serial_debug_send_message("V:");
  
  for (int i=0; i<7; ++i)
  {
    snprintf(tmp, sizeof(tmp), " %d ", cell_voltages[i]);
    serial_debug_send_message(tmp);
  }

  snprintf(tmp, sizeof(tmp), "P: %d\r\n", bq7693_get_pack_voltage());
  serial_debug_send_message(tmp);
#endif
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void serial_debug_send_pack_capacity(void)
{
#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
  char tmp[30];
  snprintf(tmp, sizeof(tmp), "C: %ld mAh\r\n", eeprom_data.current_charge_level/1000);
  serial_debug_send_message(tmp);
#endif
}


/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief 
//- **************************************************************************

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/






