/*
 * serial_debug.h
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */ 
#ifndef SERIAL_DEBUG_H_
#define SERIAL_DEBUG_H_

/*-----------------------------------------------------------------------------
  INCLUDE FILES
---------------------------------------------------------------------------- */
#include "asf.h"
#include "config.h"
#include "bq7693.h"

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL TYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DEFINITION OF GLOBAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
#define DEBUG_MSG_BUFFER_SIZE  80

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
extern void serial_debug_init(void);
extern void serial_debug_send_message(const char *msg);
extern void serial_debug_process(void);
extern void serial_debug_send_cell_voltages(void);
extern void serial_debug_send_pack_capacity(void);

/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#endif /* SERIAL_DEBUG_H_ */
