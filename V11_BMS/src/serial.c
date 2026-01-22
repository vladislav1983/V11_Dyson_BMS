/*
 * serial.c
 *
 *  Author:  David Pye
 *  Contact: davidmpye@gmail.com
 *  Licence: GNU GPL v3 or later
 */ 

#include "asf.h"
#include "serial.h"
#include "protocol.h"

int serial_msgIndex = 0;
//Data is read into here via a callback triggered by usart_read_buffer_job
static uint8_t rx_char = SERIAL_MSG_DELIM_CHAR;
struct usart_module usart_instance;
uint8_t serial_rx_buffer[40] = {0};

void uart_error_callback(struct usart_module *const module)
{
  // Clear all USART error flags (FERR, BUFOVF, PERR)
  module->hw->USART.STATUS.reg =
  SERCOM_USART_STATUS_FERR |
  SERCOM_USART_STATUS_BUFOVF |
  SERCOM_USART_STATUS_PERR;

  // Flush data register
  (void)module->hw->USART.DATA.reg;

  // Re-arm RX
  usart_read_buffer_job(&usart_instance, serial_rx_buffer, sizeof(serial_rx_buffer));
}



static inline void pin_set_peripheral_function(uint32_t pinmux) 
{
	uint8_t port = (uint8_t)((pinmux >> 16)/32);
	PORT->Group[port].PINCFG[((pinmux >> 16) - (port*32))].bit.PMUXEN = 1;
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg &= ~(0xF << (4 * ((pinmux >>
	16) & 0x01u)));
	PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg |= (uint8_t)((pinmux &
	0x0000FFFF) << (4 * ((pinmux >> 16) & 0x01u)));
}

//Data is read into here via a callback triggered by usart_read_buffer_job
void usart_read_callback(struct usart_module *const usart_module) 
{
  prot_serial_rx_callback(rx_char);  //Queue up next read.*/  usart_read_buffer_job(&usart_instance, serial_rx_buffer, sizeof(serial_rx_buffer));}

void serial_init() 
{	
	//Set up the pinmux settings for SERCOM2
	//pin_set_peripheral_function(PINMUX_PA14C_SERCOM2_PAD2);
	//pin_set_peripheral_function(PINMUX_PA15C_SERCOM2_PAD3);
  struct system_pinmux_config pin_conf;
  system_pinmux_get_config_defaults(&pin_conf);

  pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_OUTPUT;
  pin_conf.mux_position = MUX_PA14C_SERCOM2_PAD2;
  system_pinmux_pin_set_config(PIN_PA14, &pin_conf);

  pin_conf.direction = SYSTEM_PINMUX_PIN_DIR_INPUT;
  pin_conf.mux_position = MUX_PA15C_SERCOM2_PAD3;
  pin_conf.input_pull   = SYSTEM_PINMUX_PIN_PULL_UP;
  system_pinmux_pin_set_config(PIN_PA15, &pin_conf);

	
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	
	//Load the necessary settings into the config struct.
	config_usart.baudrate    = 115200;
	config_usart.mux_setting =  USART_RX_3_TX_2_XCK_3 ;
	config_usart.parity      = USART_PARITY_NONE;
	config_usart.pinmux_pad2 = PINMUX_PA14C_SERCOM2_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PA15C_SERCOM2_PAD3;
	
	//Init the UART
	while (usart_init(&usart_instance,SERCOM2, &config_usart) != STATUS_OK) { }
	
	//Enable a callback for bytes received.
	usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
	
  usart_register_callback(&usart_instance, uart_error_callback, USART_CALLBACK_ERROR);
  usart_enable_callback(&usart_instance, USART_CALLBACK_ERROR);

	usart_enable(&usart_instance);
	//Start read job - the next one is kicked off by the above callback	usart_read_buffer_job(&usart_instance, serial_rx_buffer, sizeof(serial_rx_buffer));  system_interrupt_enable(SERCOM2_IRQn);}

void serial_send(uint8_t* buff_ptr, uint8_t buff_size)
{	
  usart_write_buffer_wait(&usart_instance, buff_ptr, buff_size);
}
