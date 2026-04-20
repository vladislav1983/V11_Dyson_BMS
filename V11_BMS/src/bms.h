/*
 * bms.h
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */ 

#ifndef BMS_H_
#define BMS_H_

/*-----------------------------------------------------------------------------
  INCLUDE FILES
---------------------------------------------------------------------------- */
#include "asf.h"
#include "board.h"

#include "bq7693.h"
#include "serial.h"
#include "leds.h"
#include "eeprom_handler.h"
#include "serial_debug.h"
#include "config.h"

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL TYPES
-----------------------------------------------------------------------------*/
enum BMS_STATE
{
  BMS_INIT,
  BMS_IDLE,
  BMS_CHARGER_CONNECTED,
  BMS_CHARGING,
  BMS_CHARGER_CONNECTED_NOT_CHARGING,
  BMS_CHARGER_UNPLUGGED,
  BMS_VACUUM_RUNNING,
  BMS_FAULT, //error code should be logged to explain why!
  BMS_SLEEP,
};

enum BMS_ERROR_CODE
{
  BMS_ERR_NONE,            // 0  All good!
  BMS_ERR_PACK_DISCHARGED, // 1  Pack is flat - not really a bad error....
  BMS_ERR_UNDERVOLTAGE,    // 2  BMS IC undervoltage trip - flat pack, detected by BQ.
  BMS_ERR_PACK_UNDERTEMP,  // 3  Pack is below -40 if attempting discharge, or 0 if attempting charge.
  BMS_ERR_PACK_OVERTEMP,   // 4  Pack thermistor reading exceeded MAX_PACK_TEMPERATURE - default 60'C
  BMS_ERR_CELL_FAIL,       // 5  A cell voltage is below safe minimum.
  BMS_ERR_OVERVOLTAGE,     // 6  BMS IC overvoltage trip
  BMS_ERR_OVERCURRENT,     // 7  BMS IC overcurrent trip
  BMS_ERR_SHORTCIRCUIT,    // 8  BMS IC short circuit trip
  BMS_ERR_I2C_FAIL,        // 9  Unable to talk to the BQ7693 IC - very bad!
  BMS_ERR_WDT,             // 10 Watchdog early warning fired - main loop stalled!
};

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
 extern void bms_init(void);
 extern void bms_mainloop(void);
 extern void bms_force_fault(enum BMS_ERROR_CODE code);
 extern uint16_t bms_get_soc_x100(void);
 extern uint32_t bms_get_runtime_seconds(void);
 extern void bms_wakeup_interrupt_callback(void);
 extern void bms_interrupt_callback(void) ;
 extern void bms_interrupt_process(void);


/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#endif /* BMS_H_ */