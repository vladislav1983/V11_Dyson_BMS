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

//A struct to represent the stored eeprom data
struct eeprom_data 
{
  int32_t total_pack_capacity;      //micro-amp-hours - working capacity used for SOC/runtime
  int32_t current_charge_level;     //micro-amp-hours
  int32_t learned_pack_capacity;    //micro-amp-hours - IIR filtered capacity from coulomb counter
  int32_t cc_charge_counter_uah;    //micro-amp-hours - accumulated charge during a charge cycle
  int32_t cc_discharge_counter_uah; //micro-amp-hours - accumulated discharge during a discharge cycle
  uint16_t cycle_count;             //full charge/discharge cycle count
  uint8_t  first_cycle_done;        //0 = first cycle not yet completed, 1 = capacity learned
} ;

int eeprom_init(void);
int eeprom_read(void);
int eeprom_write(void);
int eeprom_fuses_set(void);

#endif /* EEPROM_H_ */
