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

/**
 * @brief Initialise the debug UART on SERCOM0 at 115200 baud.
 *
 * Configures PA10/PA11 as TX/RX and resets the transmit queue.
 */
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
    queue_head = 0;  // write index
    queue_tail = 0;  // read index
#endif
}

/**
 * @brief Queue a null-terminated string for debug output.
 *
 * Characters are appended to a ring buffer.  If the buffer is full
 * the remaining characters are silently dropped.
 *
 * @param msg  Null-terminated string to enqueue.
 */
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

/**
 * @brief Transmit one byte from the debug queue.
 *
 * Call repeatedly from the main loop to drain the queue without
 * blocking other tasks.
 */
void serial_debug_process(void)
{
#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
  if (queue_tail != queue_head)
  {
    SercomUsart *const hw = &(debug_usart.hw->USART);
    
    /* Check if USART is ready for new data */
    if (hw->INTFLAG.reg & SERCOM_USART_INTFLAG_DRE) 
    {
      /* Write data to USART module */
      hw->DATA.reg = (uint8_t)debug_queue[queue_tail];
      /* Update read index */
      queue_tail   = (queue_tail + 1) % DEBUG_QUEUE_SIZE;
    }
  }
#endif
}

/**
 * @brief Queue individual cell voltages and the pack voltage.
 *
 * Output format: "V: <c0> <c1> … <c6> P: <pack>\r\n"
 */
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

/**
 * @brief Queue the current pack charge level in mAh.
 *
 * Output format: "C: <mAh> mAh\r\n"
 */
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

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
