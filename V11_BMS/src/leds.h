/*
 * leds.h
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */ 

#ifndef LEDS_H_
#define LEDS_H_
/*-----------------------------------------------------------------------------
  INCLUDE FILES
---------------------------------------------------------------------------- */
#include "asf.h"
#include "config.h"

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL TYPES
-----------------------------------------------------------------------------*/
typedef enum
{
  LEDS_LED_ERR_LEFT,
  LEDS_LED_ERR_RIGHT,
  LEDS_NUM
}leds_t;

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
extern void leds_init(void);
extern void leds_sequence(void) ;
extern void leds_blink_leds(uint32_t);
extern void leds_blink_led(leds_t led, uint32_t ms);
extern void leds_blink_leds_num(leds_t led, uint32_t num, uint32_t ms);
extern void leds_off(void);
extern void leds_on(void);
extern void leds_set_led_duty(leds_t led, uint8_t duty_ppt);

/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#endif /* LEDS_H_ */