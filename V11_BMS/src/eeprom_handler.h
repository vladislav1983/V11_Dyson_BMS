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

//A struct to represent the stored eeprom data
struct eeprom_data
{
  int32_t total_pack_capacity;    //micro-amp-hours
  int32_t current_charge_level;   //micro-amp-hours
  uint8_t full_discharge_seen;    //capacity calibration flag
  uint32_t crc32;
} ;

extern int eeprom_init(void);
extern int eeprom_read(void);
extern int eeprom_write(void);
extern int eeprom_fuses_set(void);
extern void eeprom_write_defaults(void);

#endif /* EEPROM_H_ */
