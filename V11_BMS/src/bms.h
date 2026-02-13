/*
 * bms.h
 *
 *  Author:  David Pye
 *  Contact: davidmpye@gmail.com
 *  Licence: GNU GPL v3 or later
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
  BMS_TRIGGER_PULLED,
  BMS_DISCHARGING,
  BMS_FAULT, //error code should be logged to explain why!
  BMS_SLEEP,
};

enum BMS_ERROR_CODE
{
  BMS_ERR_NONE,            //All good!
  BMS_ERR_PACK_OVERTEMP,   //Pack thermistor reading exceeded MAX_PACK_TEMPERATURE - default 60'C
  BMS_ERR_PACK_UNDERTEMP,  //Pack is below -40 if attempting discharge, or -0 if attempting charge.
  BMS_ERR_CELL_FAIL,       //A cell voltage is below safe minimum.
  BMS_ERR_SHORTCIRCUIT,    //BMS detected short circuit
  BMS_ERR_OVERCURRENT,     //BMS detected overcurrent fault
  BMS_ERR_OVERVOLTAGE,     //BMS detected overvoltage state
  BMS_ERR_I2C_FAIL,        //Unable to talk to the BQ7693 IC - very bad!
  BMS_ERR_PACK_DISCHARGED, //Pack is flat - not really a bad error....
  BMS_ERR_UNDERVOLTAGE,    //BMS detected undervoltage state - flat pack, but detected by the BQ.
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
 extern uint16_t bms_get_soc_x100(void);
 extern uint32_t bms_get_runtime_seconds(void);
 extern void bms_wakeup_interrupt_callback(void);
 extern void bms_interrupt_callback(void) ;
 extern void bms_interrupt_process(void);

/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#endif /* BMS_H_ */