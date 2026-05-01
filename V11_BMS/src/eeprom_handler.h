/*
 * eeprom_handler.h
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */ 


#ifndef EEPROM_H_
#define EEPROM_H_

#include <ctype.h>
#include <inttypes.h>
#include <string.h> //for memcpy
 
#include "eeprom.h"
#include "nvm.h"

#include "config.h"
#include "leds.h"
#include "serial_debug.h"
#include "crc.h"

// persisted BMS state, layout must stay stable: any resize invalidates the
// CRC on pages written by older firmware, reserved bytes let us add small
// fields later without a layout break
struct eeprom_data
{
  int32_t  total_pack_capacity;    // uAh
  int32_t  current_charge_level;   // uAh (coulomb counter)
  uint8_t  full_discharge_seen;    // 1 = capacity-learning anchor set
  uint8_t  imbalance_locked;       // 1 = cell imbalance latched, charge+discharge blocked
  uint8_t  reserved[6];
  uint32_t crc32;                  // CRC-32 over all preceding bytes
};

extern int eeprom_init(void);
extern int eeprom_read(void);
extern int eeprom_write(void);
extern int eeprom_fuses_set(void);
extern void eeprom_write_defaults(void);

#endif /* EEPROM_H_ */
