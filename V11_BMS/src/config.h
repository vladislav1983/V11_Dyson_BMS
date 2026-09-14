/*
 * config.h
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */

#ifndef CONFIG_H_
#define CONFIG_H_

// pin assignments
#define LED_ERR_RIGHT                       PIN_PA00
#define LED_ERR_LEFT                        PIN_PA19

// drives Q3 on the charger inlet, pull high to accept charge, verified on V11 and V15
#define ENABLE_CHARGE_PIN                   PIN_PA01

// BQ7693 ALERT line
#define BQ7693_ALERT_PIN                    PIN_PA28

// high while trigger is pulled, do not pull up or down
#define TRIGGER_PRESSED_PIN                 PIN_PA04
// high while charger is plugged in
#define CHARGER_CONNECTED_PIN               PIN_PA06
#define MODE_BUTTON_PIN                     PIN_PA09
#define MODE_BUTTON_PULLUP_ENABLE_PIN       PIN_PA18
#define PRECHARGE_PIN                       PIN_PA24

#define PACK_CELL_COUNT                     7u
#define PACK_MAX_CAPACITY_MAH               3600
#define PACK_CAPACITY_UPPER_BOUND_UAH       (PACK_MAX_CAPACITY_MAH * 1200ul)
#define CELL_LOWEST_DISCHARGE_VOLTAGE       2500    // mV, below this no discharge
#define CELL_LOWEST_CHARGE_VOLTAGE          2000    // mV, below this no charge
#define CELL_FULL_CHARGE_VOLTAGE            4170    // mV, stock firmware target
#define CELL_FULL_CHARGE_RELEASE_VOLTAGE    4100    // mV, resume charging below this (70 mV hysteresis)

#define CELL_OVERVOLTAGE_TRIP               4250    // BQ7693 OV trip, valid range 3150-4700 mV
#define CELL_UNDERVOLTAGE_TRIP              2450    // BQ7693 UV trip, valid range 1700-3000 mV

// cell imbalance
#define CELL_IMBALANCE_FAULT_MV             500
#define CELL_IMBALANCE_DEBOUNCE             30       // debounce for 1.5 s at the 50 ms safety polling interval

#define FAULT_SLEEP_TIMEOUT_MS              120000   // sleep after fault display timeout

// 18650 temperature limits (Molicel datasheet)
#define MAX_PACK_TEMPERATURE                60       // °C, above this no charge or discharge
#define MIN_PACK_CHARGE_TEMP                0        // °C, below this no charge
#define MIN_PACK_DISCHARGE_TEMP             -40      // °C, below this no discharge
// V11/V15 packs use two RTDs and the assignment is unclear, so these
// limits are not currently enforced, do not charge a hot pack unattended,
// for V15 debug only — pack outputs 24 V

#define IDLE_TIME                           60 * 30 // seconds before SHIP/deep sleep when nothing happens

#define FULL_CHARGE_PAUSE_COUNT             3 // pause/retry passes after first reaching the full-charge threshold

#define SERIAL_DEBUG                        1 // debug UART on the programming-header USART
#define PROT_DEBUG_PRINT                    1

// trigger behaviour: 0 = momentary (hold to run), 1 = toggle (press to run/stop, hold ≥ 1 s to stop)
#define TRIGGER_TOGGLE_MODE                 0

#endif /* CONFIG_H_ */
