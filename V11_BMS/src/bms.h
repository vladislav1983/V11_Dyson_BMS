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
  BMS_FAULT,                 // bms_error holds the cause
  BMS_SLEEP,
};

enum BMS_ERROR_CODE
{
  BMS_ERR_NONE,            // 0  no error
  BMS_ERR_PACK_DISCHARGED, // 1  pack flat
  BMS_ERR_UNDERVOLTAGE,    // 2  BQ7693 undervoltage trip
  BMS_ERR_PACK_UNDERTEMP,  // 3  below -40 °C (discharge) or 0 °C (charge)
  BMS_ERR_PACK_OVERTEMP,   // 4  pack temperature above MAX_PACK_TEMPERATURE
  BMS_ERR_CELL_FAIL,       // 5  cell below the safe minimum
  BMS_ERR_OVERVOLTAGE,     // 6  BQ7693 overvoltage trip
  BMS_ERR_OVERCURRENT,     // 7  BQ7693 overcurrent trip
  BMS_ERR_SHORTCIRCUIT,    // 8  BQ7693 short-circuit trip
  BMS_ERR_I2C_FAIL,        // 9  BQ7693 I²C communication failure
  BMS_ERR_WDT,             // 10 watchdog early warning fired (main loop stalled)
  BMS_ERR_CELL_IMBALANCE,  // 11 cell spread exceeded CELL_IMBALANCE_FAULT_MV
  BMS_ERR_SENSOR_FAIL,     // 12 invalid ADC/NTC measurement
  BMS_ERR_EEPROM_FAIL,     // 13 EEPROM read/write failure
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
extern uint32_t bms_get_full_charge_capacity_001mah(void);
extern void bms_wakeup_interrupt_callback(void);
extern void bms_interrupt_callback(void);
extern void bms_interrupt_process(void);

/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#endif /* BMS_H_ */
