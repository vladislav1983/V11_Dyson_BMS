/*
 * bms_wdt.c
 *
 * Created: 14/02/2026 15:06:58
 * Author : Vladislav Gyurov
 * License: GNU GPL v3 or later
 */
 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "bms_wdt.h"
#include "sw_timer.h"
#include "bq7693.h"
#include "bms.h"

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
#define WDT_TIMER_MS                  250

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
static sw_timer wdt_timer = 0;
/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static void bms_wdt_early_warning_callback(void);

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**
 * @brief initialise the hardware watchdog,
 *        timeout 1024 clk, early warning at 512 clk, registers the callback
 *        and starts the periodic kick timer
 */
void bms_wdt_init(void)
{
	struct wdt_conf config_wdt;

	wdt_get_config_defaults(&config_wdt);
    config_wdt.clock_source         = GCLK_GENERATOR_2;
	config_wdt.timeout_period       = WDT_PERIOD_1024CLK;
	config_wdt.early_warning_period = WDT_PERIOD_512CLK;
	wdt_set_config(&config_wdt);

  wdt_register_callback(bms_wdt_early_warning_callback, WDT_CALLBACK_EARLY_WARNING);
  wdt_enable_callback(WDT_CALLBACK_EARLY_WARNING);

  sw_timer_start(&wdt_timer);
}

/** @brief disable the watchdog, unregisters the callback and turns off the peripheral */
void bms_wdt_deinit(void)
{
  struct wdt_conf config_wdt;

  wdt_disable_callback(WDT_CALLBACK_EARLY_WARNING);
  wdt_get_config_defaults(&config_wdt);
  config_wdt.enable = false;
  wdt_set_config(&config_wdt);
}

/** @brief watchdog kick, call from the main loop, resets the counter every WDT_TIMER_MS */
void bms_wdt_mainloop(void)
{
  if(true == sw_timer_is_elapsed(&wdt_timer, WDT_TIMER_MS))
  {
    sw_timer_start(&wdt_timer);
    wdt_reset_count();
  }
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/** @brief early-warning callback, force a fault before the WDT resets the MCU */
static void bms_wdt_early_warning_callback(void)
{
  bms_force_fault(BMS_ERR_WDT);
}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
