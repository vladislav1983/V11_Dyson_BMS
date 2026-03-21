/*
 * serial.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */
 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/

#include "asf.h"
#include "serial.h"

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
struct usart_module usart_instance;

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**
 * @brief Initialize Dyson vacuum UART (SERCOM2, 115200 baud, RX only at start).
 */
void serial_init()
{
  struct system_pinmux_config pin_conf;

  struct usart_config config_usart;
  usart_get_config_defaults(&config_usart);

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

  while (usart_init(&usart_instance, SERCOM2, &config_usart) != STATUS_OK) { }

  usart_enable(&usart_instance);
  usart_disable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);
}

/**
 * @brief Polled UART read with error recovery.
 *
 * Clears framing/parity/overflow errors before checking for data.
 *
 * @param ch  Pointer to store the received byte.
 * @return    true if a byte was received, false otherwise.
 */
bool serial_rx_byte(uint8_t *ch)
{
  SercomUsart *const hw = &(usart_instance.hw->USART);

  /* Clear any UART errors to avoid blocking reception */
  if (hw->STATUS.reg & (SERCOM_USART_STATUS_FERR |
                        SERCOM_USART_STATUS_PERR |
                        SERCOM_USART_STATUS_BUFOVF))
  {
    hw->STATUS.reg = SERCOM_USART_STATUS_FERR |
                     SERCOM_USART_STATUS_PERR |
                     SERCOM_USART_STATUS_BUFOVF;
  }

  if (!(hw->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC))
    return false;

  *ch = (uint8_t)(hw->DATA.reg);
  return true;
}

/**
 * @brief Half-duplex TX: switch to transmit, send buffer, switch back to receive.
 *
 * @param buff_ptr   Pointer to transmit buffer.
 * @param buff_size  Number of bytes to send.
 */
void serial_send(uint8_t* buff_ptr, uint8_t buff_size)
{
  usart_disable_transceiver(&usart_instance, USART_TRANSCEIVER_RX);
  usart_enable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);

  usart_write_buffer_wait(&usart_instance, buff_ptr, buff_size);

  usart_disable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);
  usart_enable_transceiver(&usart_instance, USART_TRANSCEIVER_RX);
}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
