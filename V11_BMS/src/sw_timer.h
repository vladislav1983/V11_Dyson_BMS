/*
 * sw_timer.h
 *
 * Created: 21-Jan-26 16:31:23
 * Author : Vladislav Gyurov
 * License: GNU GPL v3 or later
 */ 
 #ifndef SW_TIMER_H_
#define SW_TIMER_H_
/*-----------------------------------------------------------------------------
  INCLUDE FILES
---------------------------------------------------------------------------- */
#include "asf.h"
#include "protocol.h"
#include "bms_adc.h"
#include "bms.h"
#include "dio.h"
#include "bms_wdt.h"
#include "serial_debug.h"

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL TYPES
-----------------------------------------------------------------------------*/
typedef uint32_t sw_timer;

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
#define SW_TIMER_TICK_MS      1
#define SW_TIMER_SERVICES()   \
{ \
  prot_mainloop(); \
  bms_interrupt_process(); \
  dio_mainloop(); \
  bms_wdt_mainloop();\
  serial_debug_process(); \
}

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
extern void sw_timer_init(void);
extern void sw_timer_start(sw_timer * sw_timer_ptr);
extern void sw_timer_stop(sw_timer * sw_timer_ptr);
extern bool sw_timer_is_started(sw_timer * sw_timer_ptr);
extern bool sw_timer_is_elapsed(sw_timer * sw_timer_ptr, uint32_t timeout);
extern sw_timer sw_timer_get_elapsed_time(sw_timer * sw_timer_ptr);
extern void sw_timer_delay_ms(uint32_t sw_timer_delay_ms);

/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/

#endif /* SW_TIMER_H_ */