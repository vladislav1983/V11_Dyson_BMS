/*
 * sw_timer.c
 *
 * Created: 21-Jan-26 16:31:01
 *  Author: Vladislav Gyurov
 */ 
 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
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
static volatile uint32_t sw_timer_clock = 0;
static volatile sw_timer delay_timer = 0;
struct tc_module tc_instance;


/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static uint32_t get_ticks_ms(void);
static void tc_callback_sw_timer(struct tc_module *const module_inst);

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief 
//- **************************************************************************
void sw_timer_init(void)
{
  struct tc_config config_tc;
  uint32_t cycles_per_ms = system_gclk_gen_get_hz(0);
  cycles_per_ms /= 1000;

  tc_get_config_defaults(&config_tc);
  config_tc.counter_size = TC_COUNTER_SIZE_16BIT;
  config_tc.clock_source = GCLK_GENERATOR_1;
  config_tc.clock_prescaler = TC_CLOCK_PRESCALER_DIV1;
  config_tc.wave_generation = TC_WAVE_GENERATION_MATCH_FREQ;
  config_tc.counter_16_bit.value = 0;
  config_tc.counter_16_bit.compare_capture_channel[0] = (uint16_t)cycles_per_ms;
  config_tc.counter_16_bit.compare_capture_channel[1] = (uint16_t)cycles_per_ms;
  tc_init(&tc_instance, TC0, &config_tc);
  tc_enable(&tc_instance);
  tc_register_callback(&tc_instance, tc_callback_sw_timer, TC_CALLBACK_CC_CHANNEL0);
  tc_enable_callback(&tc_instance, TC_CALLBACK_CC_CHANNEL0);
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void sw_timer_start(sw_timer * sw_timer_ptr)
{
  *sw_timer_ptr = get_ticks_ms();
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void sw_timer_stop(sw_timer * sw_timer_ptr)
{
  *sw_timer_ptr = 0;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
bool sw_timer_is_started(sw_timer * sw_timer_ptr)
{
  // A timer is never equal to 0
  // The 0 value is reserved to the timer stopped
  if (*sw_timer_ptr != 0)
    return(1);

  return(0);
}

//- **************************************************************************
//! \brief
//- **************************************************************************
bool sw_timer_is_elapsed(sw_timer * sw_timer_ptr, uint32_t timeout)
{
  uint32_t Delay;

  // A timer is never equal to 0
  // The 0 value is reserved to the timer stopped
  if (*sw_timer_ptr == 0)
  {
    return true;
  }
  else
  {
    uint32_t ticks = get_ticks_ms();

    Delay = (sw_timer)(ticks - *sw_timer_ptr);

    if(ticks < *sw_timer_ptr)
    {
      // The 0 value had been "jump" so we must substract 1 to the delay
      --Delay;
    }

    if ((Delay > timeout) || (timeout == 0))
    {
      // The timer is stopped or elapsed
      *sw_timer_ptr = 0;
      return true;
    }
  }

  return false;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
sw_timer sw_timer_get_elapsed_time(sw_timer * sw_timer_ptr)
{
  uint32_t Delay = 0;

  // A timer is never equal to 0
  // The 0 value is reserved to the timer stopped
  if ( *sw_timer_ptr == 0 )
  {
    // This function must not be call with a stopped tempo
    Assert(false);
  }
  else
  {
    uint32_t ticks = get_ticks_ms();

    Delay = (sw_timer)(ticks - *sw_timer_ptr);

    if(ticks < *sw_timer_ptr)
    {
      // The 0 value had been "jump" so we must substract 1 to the delay
      --Delay;
    }
  }

  return Delay;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void sw_timer_delay_ms(uint32_t sw_timer_delay_ms)
{
  sw_timer_start((sw_timer *)&delay_timer);

  do 
  {
    SW_TIMER_SERVICES();
  } while(false == sw_timer_is_elapsed((sw_timer *)&delay_timer, sw_timer_delay_ms));
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief
//- **************************************************************************
static void tc_callback_sw_timer(struct tc_module *const module_inst)
{
  sw_timer_clock++;

  // A timer is never equal to 0
  // The 0 value is reserved to the timer stopped
  if(sw_timer_clock == 0)
  {
    sw_timer_clock++;
  }
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static uint32_t get_ticks_ms(void)
{
  return sw_timer_clock;
}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/