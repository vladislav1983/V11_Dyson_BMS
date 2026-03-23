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
    DEFINITION OF LOCAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
#define SERIAL_RX_BUF_SIZE  64
#define SERIAL_RX_BUF_MASK  (SERIAL_RX_BUF_SIZE - 1)

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
struct usart_module usart_instance;

static volatile uint8_t rx_ring[SERIAL_RX_BUF_SIZE];
static volatile uint8_t rx_ring_head;   // ISR writes here
static volatile uint8_t rx_ring_tail;   // serial_rx_byte reads here

/*-----------------------------------------------------------------------------
    DEFINITION OF INTERRUPT HANDLERS
-----------------------------------------------------------------------------*/

/**
 * @brief SERCOM2 RX interrupt handler — store received bytes into ring buffer.
 */
void SERCOM2_Handler_BMS(void)
{
  SercomUsart *const hw = &(SERCOM2->USART);

  if (hw->STATUS.reg & (SERCOM_USART_STATUS_FERR |
                         SERCOM_USART_STATUS_PERR |
                         SERCOM_USART_STATUS_BUFOVF))
  {
    hw->STATUS.reg = SERCOM_USART_STATUS_FERR |
                     SERCOM_USART_STATUS_PERR |
                     SERCOM_USART_STATUS_BUFOVF;
    // do not discard data
  }

  if (hw->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC)
  {
    uint8_t data = (uint8_t)hw->DATA.reg;
    uint8_t next = (rx_ring_head + 1) & SERIAL_RX_BUF_MASK;
    if (next != rx_ring_tail)
    {
      rx_ring[rx_ring_head] = data;
      rx_ring_head = next;
    }
  }
}

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
  pin_conf.mux_position = MUX_PA15C_SERCOM2_PAD3;
  system_pinmux_pin_set_config(PIN_PA15, &pin_conf);

  while (usart_init(&usart_instance, SERCOM2, &config_usart) != STATUS_OK) { }

  usart_enable(&usart_instance);
  usart_disable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);

  // Enable RXC interrupt for ring buffer reception
  rx_ring_head = 0;
  rx_ring_tail = 0;
  {
    SercomUsart *const hw = &(usart_instance.hw->USART);
    hw->INTENSET.reg = SERCOM_USART_INTENSET_RXC;
  }
  system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_SERCOM2);
}

/**
 * @brief Read one byte from the RX ring buffer (filled by SERCOM2 ISR).
 *
 * @param ch  Pointer to store the received byte.
 * @return    true if a byte was available, false if buffer empty.
 */
bool serial_rx_byte(uint8_t *ch)
{
  if (rx_ring_tail == rx_ring_head)
    return false;

  *ch = rx_ring[rx_ring_tail];
  rx_ring_tail = (rx_ring_tail + 1) & SERIAL_RX_BUF_MASK;
  return true;
}

/**
 * @brief Check if RX ring buffer has data available.
 * @return true if one or more bytes are buffered.
 */
bool serial_rx_available(void)
{
  return (rx_ring_tail != rx_ring_head);
}

/**
 * @brief Half-duplex TX: switch to transmit, send buffer, switch back to receive.
 *
 * @param buff_ptr   Pointer to transmit buffer.
 * @param buff_size  Number of bytes to send.
 */
void serial_send(uint8_t* buff_ptr, uint8_t buff_size)
{
  SercomUsart *const hw = &(usart_instance.hw->USART);

  /* Disable RX interrupt before switching to TX mode */
  hw->INTENCLR.reg = SERCOM_USART_INTENCLR_RXC;
  usart_disable_transceiver(&usart_instance, USART_TRANSCEIVER_RX);

  /* Flush stale RX data and errors before switching to TX */
  hw->STATUS.reg = SERCOM_USART_STATUS_FERR |
                   SERCOM_USART_STATUS_PERR |
                   SERCOM_USART_STATUS_BUFOVF;
  if (hw->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC)
    (void)hw->DATA.reg;

  usart_enable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);
  usart_write_buffer_wait(&usart_instance, buff_ptr, buff_size);
  usart_disable_transceiver(&usart_instance, USART_TRANSCEIVER_TX);

  /* Clear errors/noise from the TX period before re-enabling RX */
  hw->STATUS.reg = SERCOM_USART_STATUS_FERR |
                   SERCOM_USART_STATUS_PERR |
                   SERCOM_USART_STATUS_BUFOVF;
  if (hw->INTFLAG.reg & SERCOM_USART_INTFLAG_RXC)
    (void)hw->DATA.reg;

  usart_enable_transceiver(&usart_instance, USART_TRANSCEIVER_RX);

  /* Flush ring buffer (TX echoes noise) and re-enable RX interrupt */
  rx_ring_head = 0;
  rx_ring_tail = 0;
  hw->INTENSET.reg = SERCOM_USART_INTENSET_RXC;
}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
