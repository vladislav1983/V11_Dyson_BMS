/*
 * serial.c
 *
 *  Author:  David Pye
 *  Contact: davidmpye@gmail.com
 *  Licence: GNU GPL v3 or later
 */ 
 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/

#include "asf.h"
#include "serial.h"
#include "protocol.h"
#include "sw_timer.h"

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

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
//Data is read into here via a callback triggered by usart_read_buffer_job
static uint8_t rx_char = 0;
struct usart_module usart_instance;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static void usart_read_callback(struct usart_module *const usart_module);

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief 
//- **************************************************************************
void serial_init()
{
  struct system_pinmux_config pin_conf;

  //Set up the pinmux settings for SERCOM2
  //pin_set_peripheral_function(PINMUX_PA14C_SERCOM2_PAD2);
  //pin_set_peripheral_function(PINMUX_PA15C_SERCOM2_PAD3);
  
  struct usart_config config_usart;
  usart_get_config_defaults(&config_usart);
  
  //Load the necessary settings into the config struct.
  config_usart.baudrate       = 115200;
  config_usart.mux_setting    = USART_RX_3_TX_2_XCK_3 ;
  config_usart.transfer_mode  = USART_TRANSFER_ASYNCHRONOUSLY;
  config_usart.parity         = USART_PARITY_NONE;
  config_usart.pinmux_pad0    = PINMUX_UNUSED;
  config_usart.pinmux_pad1    = PINMUX_UNUSED;
  config_usart.pinmux_pad2    = PINMUX_PA14C_SERCOM2_PAD2;
  config_usart.pinmux_pad3    = PINMUX_PA15C_SERCOM2_PAD3;
  config_usart.stopbits       = USART_STOPBITS_1;
  config_usart.character_size = USART_CHARACTER_SIZE_8BIT;

  system_pinmux_get_config_defaults(&pin_conf);
  pin_conf.mux_position = PINMUX_PA15C_SERCOM2_PAD3;
  system_pinmux_pin_set_config(PIN_PA15, &pin_conf);

  //Init the UART
  while (usart_init(&usart_instance,SERCOM2, &config_usart) != STATUS_OK) { }
  
  //Enable a callback for bytes received.
  usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
  usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);

  usart_enable(&usart_instance);
  //Start read job - the next one is kicked off by the above callback  usart_read_buffer_job(&usart_instance, &rx_char, sizeof(rx_char));  usart_disable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);  usart_write_buffer_wait(&usart_instance, 0, 1);  NVIC_SetPriority(SERCOM2_IRQn, 0);}

//- **************************************************************************
//! \brief
//- **************************************************************************
void serial_send(uint8_t* buff_ptr, uint8_t buff_size)
{
  NVIC_DisableIRQ(SERCOM2_IRQn);
  usart_disable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
  usart_disable_transceiver(&usart_instance, USART_TRANSCEIVER_RX);
  usart_enable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);

  usart_write_buffer_wait(&usart_instance, buff_ptr, buff_size);
  
  usart_disable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);
  usart_enable_transceiver(&usart_instance, USART_TRANSCEIVER_RX);
  usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
  NVIC_EnableIRQ(SERCOM2_IRQn);
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief Data is read into here via a callback triggered by usart_read_buffer_job
//- **************************************************************************
static void usart_read_callback(struct usart_module *const usart_module)
{
  prot_serial_rx_callback(rx_char);    //Queue up next read.*/  usart_read_buffer_job(&usart_instance, &rx_char, sizeof(rx_char));}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/











