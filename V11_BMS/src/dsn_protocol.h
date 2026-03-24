/*
 * dsn_protocol.h
 *
 * Dyson vacuum UART protocol handler — dynamic TLV-based implementation
 * matching the original firmware's pair dispatch architecture.
 *
 * Author : Vladislav Gyurov
 * License: GNU GPL v3 or later
 */

#ifndef DSN_PROTOCOL_H_
#define DSN_PROTOCOL_H_
/*-----------------------------------------------------------------------------
  INCLUDE FILES
---------------------------------------------------------------------------- */
#include "asf.h"

/*-----------------------------------------------------------------------------
  DECLARATION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
extern void dsn_prot_init(void);
extern void dsn_prot_set_trigger(bool trigger_state);
extern void dsn_prot_reset(void);
extern bool dsn_prot_get_sleep_flag(void);
extern bool dsn_prot_get_vacuum_connected(void);
extern void dsn_prot_mainloop(void);

/*-----------------------------------------------------------------------------
  END OF MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#endif /* DSN_PROTOCOL_H_ */
