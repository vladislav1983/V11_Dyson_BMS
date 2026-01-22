/*
 * config.h
 *
 *  Author:  David Pye
 *  Contact: davidmpye@gmail.com
 *  Licence: GNU GPL v3 or later
 */ 

#ifndef CONFIG_H_
#define CONFIG_H_

// Pin definitions
#define LED_BLOCKED PIN_PA00
#define LED_ERR PIN_PA19 // Works on V15, showing ERR

//- seems to go to Q3 on the charger inlet side of things, push it high to accept a charge
//#define ENABLE_CHARGE_PIN PIN_PA02 // For V10
#define ENABLE_CHARGE_PIN PIN_PA01 // For V15 & V11 charging confirmed

//PA28 appears to be ALERT pin from the BQ7693
#define BQ7693_ALERT_PIN PIN_PA28

 //NB Goes high when trigger pulled, but don't have it pulled up or down..
#define TRIGGER_PRESSED_PIN PIN_PA04
//Goes high when charger plugged in.
#define CHARGER_CONNECTED_PIN PIN_PA06
// vacuum modes button
#define MODE_BUTTON           PIN_PA31

//PA07 is attached to thermistor RT1
//PA08 is attached to thermistor RT2

//Some packs have Molicell INR18650P26a - datasheet https://www.molicel.com/wp-content/uploads/INR18650P26A-V2-80087.pdf
#define CELL_LOWEST_DISCHARGE_VOLTAGE 2500	//mV - wont allow pack to discharge if any cells lower than this
#define CELL_LOWEST_CHARGE_VOLTAGE 2000		//mV - won't try to charge the pack if any cells lower than this
#define CELL_FULL_CHARGE_VOLTAGE 4200		//mV - fully charged cell voltage.

#define CELL_OVERVOLTAGE_TRIP  4250		//BMS will trip out at this voltage - NB DO NOT set outside of 3150mV - 4700mV or it wont' work! 
#define CELL_UNDERVOLTAGE_TRIP 2450		//BMS will trip out at this voltage - NB DO NOT set outside of 1700mv - 3000mV or it wont' work!

//18650 cell temperature limits from Molicell datasheet.
#define MAX_PACK_TEMPERATURE 60				//'C - if pack temperature greater than this, no charge/discharge allowed.
//#define MIN_PACK_CHARGE_TEMP 0				//'C - if less than this, no charge.
//#define MIN_PACK_DISCHARGE_TEMP -40			//'C - if less than this, no discharge
#define MIN_PACK_CHARGE_TEMP -300				//'C - if less than this, no charge.
#define MIN_PACK_DISCHARGE_TEMP -400			//'C - if less than this, no discharge
// Limits disabled, as V15 & V11 have 2xRTDs, now unknown where are assigned, therefore even max temp doesnt work
// Do not charge battery when hot and not supervised! This is for Debug only for V15, battery pack outputs 24V

#define IDLE_TIME 60 * 1 // Idle time in seconds. Pack will go into SHIP/deep sleep mode if nothing happens in this duration

#define FULL_CHARGE_PAUSE_COUNT 10 //Once a cell reaches max charge volts, pause for 30 seconds and retry, this many times.

#define SERIAL_DEBUG 1 //Serial debug via the spare USART on the programming pins header

#endif /* CONFIG_H_ */
