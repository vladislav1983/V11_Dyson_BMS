/*
 * bms.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */

 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "bms.h"
#include "bms_adc.h"
#include "ntc.h"
#include "crc.h"
#include "sw_timer.h"
#include "dsn_protocol.h"
#include "dio.h"
#include "bms_wdt.h"

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/
volatile bool     force_sleep = false;

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
static void bms_set_error(enum BMS_ERROR_CODE code);

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
#ifdef SERIAL_DEBUG
#define BMS_PRINT(...) \
{ \
  char _dbg_tmp[DEBUG_MSG_BUFFER_SIZE]; \
  snprintf(_dbg_tmp, sizeof(_dbg_tmp), __VA_ARGS__); \
  serial_debug_send_message(_dbg_tmp);  \
}
#else
#define BMS_PRINT(...)
#endif

#define ROUND(x) (((x) + 0.5))
#define PACK_CAPACITY_UPPER_BOUND_UAH       (PACK_MAX_CAPACITY_MAH * 1200ul)  // 120% of nominal, in uAh

// RTC standby wake timer: GCLK2 = ULP32K/32 (1024 Hz), RTC prescaler = DIV1024 → 1 Hz,
// N days = N * 86400 seconds × 1 tick/sec
#define RTC_STANDBY_WAKE_TICKS  ((uint32_t)2 * (24UL * 60UL * 60UL))

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
// we start off idle
static enum BMS_STATE bms_state      = BMS_INIT;
// if a fault occurs, it'll be lodged here
static enum BMS_ERROR_CODE bms_error = BMS_ERR_NONE;

static int32_t current_mA = 0;
static int32_t current_filt_sum_mA = 0;
static int32_t current_filt_mA = 0;

static uint16_t charge_pause_counter = 0;
static sw_timer bms_timer = 0;
static int16_t  pack_temperature = 0;
static bool process_bms_interrupt = false;
static volatile bool rtc_wakeup_flag = false;
static struct rtc_module rtc_instance;

extern volatile struct eeprom_data eeprom_data;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/
#ifdef SERIAL_DEBUG
const char *bms_state_names[] =
{
  "INIT",
  "IDLE",
  "CHARGER_CONNECTED",
  "CHARGING",
  "CHARGER_CONNECTED_NOT_CHARGING",
  "CHARGER_UNPLUGGED",
  "VACUUM_RUNNING",
  "FAULT",
  "SLEEP"
};
#endif

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/
static void    pins_init(void);
static void    pins_deinit(void);
static void    interrupts_init(void);
static int16_t bms_read_temperature(void);
static uint16_t bms_get_cell_spread_mv(void);
static bool    bms_trigger_active(void);
static bool    bms_factory_reset_check(uint8_t *count, bool *prev_level, sw_timer *timeout);
static bool    bms_is_safe_to_discharge(void);
static bool    bms_is_safe_to_charge(void);
static bool    bms_is_pack_full(void);
static void    bms_handle_idle(void);
static void    bms_handle_sleep(void);
static void    bms_handle_vacuum_running(void);
static void    bms_handle_fault(void);
static void    bms_handle_charger_connected(void);
static void    bms_handle_charger_connected_not_charging(void);
static void    bms_handle_charging(void);
static void    bms_handle_charger_unplugged(void);
static void    bms_enter_standby(void);
static void    bms_leave_standby(void);
static void    rtc_standby_timer_init(void);
static void    rtc_standby_timer_start(void);
static void    rtc_standby_timer_stop(void);

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
/** @brief initialise the BMS: clocks, peripherals, BQ7693, EEPROM, UART, RTC */
void bms_init(void)
{
  system_init();
  delay_init();
  sw_timer_init();
  dsn_prot_init();

  pins_init();
  dio_init();

  bms_adc_init();
  bq7693_init();

  leds_init();
  eeprom_init();
  eeprom_read();

  serial_init();

  interrupts_init();

  rtc_standby_timer_init();

#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
  serial_debug_init();
#endif
}

/** @brief wakeup EIC callback, unused and only needed for the wake event itself */
void bms_wakeup_interrupt_callback(void)
{

}

/** @brief BQ7693 ALERT line ISR, defers work to bms_interrupt_process() */
void bms_interrupt_callback(void)
{
  process_bms_interrupt = true;
}

/** @brief servicing for the BQ7693 ALERT, updates current and charge level */
void bms_interrupt_process(void)
{
  uint8_t sys_stat;

  if(true == process_bms_interrupt)
  {
    bq7693_read_register(SYS_STAT, 1, &sys_stat);

    if (sys_stat & 0x80)
    {
      // new coulomb counter sample ready
      int32_t ccVal = bq7693_read_cc();

      // convert raw CC value to mA, sense resistor = 1 mOhm
      current_mA = (ccVal * (uint16_t)(8.44f * 4096.0f)) / 4096;
      #define FILT_MS   (500ul)
      #define PERIOD_MS (250ul)
      current_filt_sum_mA += ( (int32_t)((65536.0 * PERIOD_MS) / FILT_MS) * (int16_t)(current_mA - (int16_t)(current_filt_sum_mA >> 16) ) );
      current_filt_mA = current_filt_sum_mA >> 16;

      {
        // convert CC sample to uAh and accumulate, 14.4 = (3600 s/h * 1000 mA/A) / (250 ms * 1000 mAh/Ah)
        int32_t cc_uah;
        cc_uah = ccVal * (int16_t)(((8.44f * 250.0f * 32768.0f) / (3600.0f)));
        cc_uah /= 32768;
        eeprom_data.current_charge_level += cc_uah;

        if (eeprom_data.full_discharge_seen)
        {
          if (eeprom_data.current_charge_level > (int32_t)(PACK_MAX_CAPACITY_MAH * 1200ul))
            eeprom_data.current_charge_level = (int32_t)(PACK_MAX_CAPACITY_MAH * 1200ul);
            
          if (eeprom_data.current_charge_level > eeprom_data.total_pack_capacity)
            eeprom_data.total_pack_capacity = eeprom_data.current_charge_level;
        }
        else if (eeprom_data.current_charge_level > eeprom_data.total_pack_capacity)
        {
          // normal operation: clamp to the learned ceiling
          eeprom_data.current_charge_level = eeprom_data.total_pack_capacity;
        }
        if (eeprom_data.current_charge_level < 0)
          eeprom_data.current_charge_level = 0;
      }
      // clear CC flag so it re-fires after the next 250 ms window
      bq7693_write_register(SYS_STAT, 0x80);
    }

    process_bms_interrupt = false;
  }
}

/**
 * @brief state of charge in 0.01 % units, for the vacuum protocol
 * @return SOC in [100, 10000], floored at 1 % to avoid the critical-battery screen
 */
uint16_t bms_get_soc_x100(void)
{
  uint16_t soc = 100;
  int32_t current_charge_level = eeprom_data.current_charge_level;
  int16_t total_pack_capacity  = eeprom_data.total_pack_capacity  >> 10;

  if(total_pack_capacity > 0 && current_charge_level > 0)
  {
    soc = (current_charge_level * (uint16_t)ROUND((100.0f * 100.0f) / 1024.0f)) / total_pack_capacity;
    soc = (soc > 10000) ? 10000 : ((soc == 0) ? 100 : soc);
  }

  return soc;
}

/**
 * @brief estimate remaining runtime from the filtered current
 * @return seconds, zero when not discharging and floored at 60 s otherwise
 */
uint32_t bms_get_runtime_seconds(void)
{
  int32_t current_filt_mA_abs = abs(current_filt_mA);
  int32_t current_charge_level;
  int32_t runtime = 0;

  // only estimate while the motor is running and pulling > 1 A
  if(    bms_state == BMS_VACUUM_RUNNING
      && current_filt_mA_abs > 1000)
  {
    current_charge_level = eeprom_data.current_charge_level < 0 ? 0
                         : eeprom_data.current_charge_level > (PACK_MAX_CAPACITY_MAH * 1000) ? (PACK_MAX_CAPACITY_MAH * 1000)
                         : eeprom_data.current_charge_level;

    runtime = ((current_charge_level / current_filt_mA_abs) * (uint16_t)((3600.0f / 1000.0f) * 1024.0f)) >> 10;

    runtime = runtime < 60 ? 60 : runtime;
  }

  return (uint32_t)runtime;
}

/** @brief main BMS state machine, never returns */
void bms_mainloop(void)
{
  bms_wdt_init();
  while (1)
  {
    BMS_PRINT("BMS_STATE: %s\r\n", bms_state_names[bms_state]);

    switch (bms_state)
    {
    //-----------------------------------------------------------------------
      case BMS_INIT:
#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
        serial_debug_send_message("Dyson V11/V15 BMS After market firmware\r\n");
#endif
        leds_sequence();
        wdt_reset_count();

#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
        serial_debug_send_cell_voltages();
        serial_debug_send_pack_capacity();
#endif
        wdt_reset_count();

        // surface a latched fault from EEPROM up front, before the user tries to use the pack
        if (eeprom_data.imbalance_locked)
        {
          bms_error = BMS_ERR_CELL_IMBALANCE;
          bms_state = BMS_FAULT;
        }
        else
        {
          bms_state = BMS_IDLE;
        }
      break;
      //-----------------------------------------------------------------------
      case BMS_IDLE:
        bms_handle_idle();
      break;
      //-----------------------------------------------------------------------
      case BMS_SLEEP:
        bms_handle_sleep();
      break;
      //-----------------------------------------------------------------------
      case BMS_CHARGER_CONNECTED:
        bms_handle_charger_connected();
      break;
      //-----------------------------------------------------------------------
      case BMS_CHARGING:
        bms_handle_charging();
      break;
      //-----------------------------------------------------------------------
      case BMS_CHARGER_CONNECTED_NOT_CHARGING:
        bms_handle_charger_connected_not_charging();
      break;
      //-----------------------------------------------------------------------
      case BMS_CHARGER_UNPLUGGED:
        bms_handle_charger_unplugged();
      break;
      //-----------------------------------------------------------------------
      case BMS_VACUUM_RUNNING:
        bms_handle_vacuum_running();
      break;
      //-----------------------------------------------------------------------
      case BMS_FAULT:
        bms_handle_fault();
      break;
      //-----------------------------------------------------------------------
      default:
      break;
    }


    dio_mainloop();
    dsn_prot_mainloop();
    bms_interrupt_process();
    bms_wdt_mainloop();
  }
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
/** @brief promote bms_error only if the new code is more severe than the current one */
static void bms_set_error(enum BMS_ERROR_CODE code)
{
  if (bms_error < code)
    bms_error = code;
}

/**
 * @brief read trigger state, returns true while the user wants the motor on,
 *        toggle mode (V12) flips a latch on each press and holding >= 1 s
 *        force-clears it, the latch is dropped on every call from a
 *        non-sampling state so non-sampling handlers can call this once to
 *        wipe stale state on the way out, momentary mode (V11/V15) just
 *        returns the raw pin level
 */
static bool bms_trigger_active(void)
{
#if TRIGGER_TOGGLE_MODE
  static bool     latched    = false;
  static bool     prev_level = false;
  static sw_timer held_timer = 0;

  bool level    = dio_read(DIO_TRIGGER_PRESSED);
  bool sampling = (bms_state == BMS_IDLE || bms_state == BMS_VACUUM_RUNNING);

  // drop the latch when the vacuum must not run: vacuum disconnected or
  // called from a non-sampling state, sync prev_level so a held trigger
  // doesn't read as a rising edge on the next sample
  if (!dsn_prot_get_vacuum_connected() || !sampling)
  {
    latched    = false;
    prev_level = level;
    return false;
  }

  if (level && !prev_level)            // rising edge: flip latch
  {
    latched = !latched;
    sw_timer_start(&held_timer);
  }
  else if (level && latched && sw_timer_is_elapsed(&held_timer, 1000))
  {
    latched = false;                   // hold >= 1 s clears the latch
  }
  prev_level = level;

  return latched;
#else
  return dio_read(DIO_TRIGGER_PRESSED);
#endif
}

/**
 * @brief factory-reset gesture: 20 raw trigger presses with gap < 5 s between
 *        them, reads the raw pin (not bms_trigger_active) so the V12 toggle
 *        latch can't mask presses, caller owns the counter so multiple call
 *        sites (fault, charging) can run independent gestures
 * @return true exactly once, on the 20th press
 */
static bool bms_factory_reset_check(uint8_t *count, bool *prev_level, sw_timer *timeout)
{
  const uint8_t  presses_required = 20;
  const uint32_t gap_ms           = 5000;

  bool level = dio_read(DIO_TRIGGER_PRESSED);

  if (level && !*prev_level)
  {
    sw_timer_start(timeout);   // rolling window, every press re-arms it
    (*count)++;
  }
  *prev_level = level;

  if (*count > 0 && sw_timer_is_elapsed(timeout, gap_ms))
    *count = 0;

  if (*count >= presses_required)
  {
    BMS_PRINT("BMS:FACTORY_RESET\r\n");
    *count = 0;
    return true;
  }
  return false;
}

/** @brief force a fault with the given code, ISR-safe */
void bms_force_fault(enum BMS_ERROR_CODE code)
{
  bms_error = code;
  bms_state = BMS_FAULT;
}

/** @brief configure GPIOs: charge enable, sense inputs, precharge, mode pull-up */
static void pins_init(void)
{
  // charge enable output
  struct port_config charge_pin_config;
  port_get_config_defaults(&charge_pin_config);
  charge_pin_config.direction = PORT_PIN_DIR_OUTPUT;
  port_pin_set_config(ENABLE_CHARGE_PIN, &charge_pin_config);
  port_pin_set_output_level(ENABLE_CHARGE_PIN, false);

  // sense inputs: charger present, trigger pressed
  struct port_config sense_pin_config;
  port_get_config_defaults(&sense_pin_config);
  sense_pin_config.direction = PORT_PIN_DIR_INPUT;
  sense_pin_config.input_pull = PORT_PIN_PULL_NONE;
  port_pin_set_config(CHARGER_CONNECTED_PIN, &sense_pin_config);
  port_pin_set_config(TRIGGER_PRESSED_PIN, &sense_pin_config);


  struct port_config io_pin_config;
  port_get_config_defaults(&io_pin_config);
  io_pin_config.direction = PORT_PIN_DIR_OUTPUT;

  // pack voltage feedback enable
  port_pin_set_config(PIN_PA03, &io_pin_config);
  port_pin_set_output_level(PIN_PA03, true);

  // mode-button pull-up supply
  port_pin_set_config(MODE_BUTTON_PULLUP_ENABLE_PIN, &io_pin_config);
  port_pin_set_output_level(MODE_BUTTON_PULLUP_ENABLE_PIN, true);

  // precharge
  port_pin_set_config(PRECHARGE_PIN, &io_pin_config);
  port_pin_set_output_level(PRECHARGE_PIN, false);

  // PA25: function unknown, currently unused
  //port_pin_set_config(PIN_PA25, &io_pin_config);
  //port_pin_set_output_level(PIN_PA25, true);

  // mode button input
  port_pin_set_config(MODE_BUTTON_PIN, &sense_pin_config);
}

/** @brief drive output GPIOs low before sleep */
static void pins_deinit(void)
{
  port_pin_set_output_level(PIN_PA03, false);
  port_pin_set_output_level(MODE_BUTTON_PULLUP_ENABLE_PIN, false);
  port_pin_set_output_level(PRECHARGE_PIN, false);
}

/** @brief configure EIC channels: BQ7693 ALERT, mode button, trigger, charger */
static void interrupts_init(void)
{
  // BQ7693 ALERT (PA28, EXTINT 8), either device may drive the line so no pull
  struct extint_chan_conf config_alert_pin;
  extint_chan_get_config_defaults(&config_alert_pin);
  config_alert_pin.wake_if_sleeping = false;
  config_alert_pin.gpio_pin     = PIN_PA28A_EIC_EXTINT8;
  config_alert_pin.gpio_pin_mux = MUX_PA28A_EIC_EXTINT8;
  config_alert_pin.gpio_pin_pull      = EXTINT_PULL_NONE;
  config_alert_pin.detection_criteria = EXTINT_DETECT_RISING;

  extint_chan_set_config(8, &config_alert_pin);
  extint_register_callback(bms_interrupt_callback, 8, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_enable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);

  //-----------------------------------------------------------------------------
  // mode button (PA09, EXTINT 9), rising edge wakes from standby
  struct extint_chan_conf config_mode_pin;
  extint_chan_get_config_defaults(&config_mode_pin);

  config_mode_pin.wake_if_sleeping    = true;
  config_mode_pin.filter_input_signal = true;
  config_mode_pin.gpio_pin            = PIN_PA09A_EIC_EXTINT9;
  config_mode_pin.gpio_pin_mux        = MUX_PA09A_EIC_EXTINT9;
  config_mode_pin.gpio_pin_pull       = EXTINT_PULL_NONE;
  config_mode_pin.detection_criteria  = EXTINT_DETECT_RISING;

  extint_chan_set_config(9, &config_mode_pin);
  extint_register_callback(bms_wakeup_interrupt_callback, 9, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_disable_callback(9, EXTINT_CALLBACK_TYPE_DETECT);

  //-----------------------------------------------------------------------------
  // trigger (PA04, EXTINT 4), rising edge wakes from standby
  struct extint_chan_conf config_trigger_pin;
  extint_chan_get_config_defaults(&config_trigger_pin);

  config_trigger_pin.wake_if_sleeping    = true;
  config_trigger_pin.filter_input_signal = true;
  config_trigger_pin.gpio_pin            = PIN_PA04A_EIC_EXTINT4;
  config_trigger_pin.gpio_pin_mux        = MUX_PA04A_EIC_EXTINT4;
  config_trigger_pin.gpio_pin_pull       = EXTINT_PULL_NONE;
  config_trigger_pin.detection_criteria  = EXTINT_DETECT_RISING;

  extint_chan_set_config(4, &config_trigger_pin);
  extint_register_callback(bms_wakeup_interrupt_callback, 4, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_disable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);

  //-----------------------------------------------------------------------------
  // charger present (PA06, EXTINT 6), both edges wake from standby
  struct extint_chan_conf config_charger_pin;
  extint_chan_get_config_defaults(&config_charger_pin);

  config_charger_pin.wake_if_sleeping    = true;
  config_charger_pin.filter_input_signal = true;
  config_charger_pin.gpio_pin            = PIN_PA06A_EIC_EXTINT6;
  config_charger_pin.gpio_pin_mux        = MUX_PA06A_EIC_EXTINT6;
  config_charger_pin.gpio_pin_pull       = EXTINT_PULL_NONE;
  config_charger_pin.detection_criteria  = EXTINT_DETECT_BOTH;

  extint_chan_set_config(6, &config_charger_pin);
  extint_register_callback(bms_wakeup_interrupt_callback, 6, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_disable_callback(6, EXTINT_CALLBACK_TYPE_DETECT);

  system_interrupt_enable_global();
}

/**
 * @brief read pack temperature from the NTC thermistor
 * @return temperature in 0.1 °C
 */
static int16_t bms_read_temperature(void)
{
  int16_t tc1_temp;
  uint16_t adc_value;

  // get tc1
  adc_value = adc_convert_channel(BMS_ADC_CH_TC1);
  tc1_temp  = NTC_ADC2Temperature(adc_value);

  return tc1_temp;
}

/** @brief highest minus lowest cell voltage, in mV */
static uint16_t bms_get_cell_spread_mv(void)
{
  uint16_t *cells = bq7693_get_cell_voltages();
  uint16_t lo = cells[0];
  uint16_t hi = cells[0];
  for (int i = 1; i < 7; ++i)
  {
    if (cells[i] < lo) lo = cells[i];
    if (cells[i] > hi) hi = cells[i];
  }
  return hi - lo;
}

/**
 * @brief check whether the pack is safe to discharge, sets bms_error on failure
 * @return true if safe
 */
static bool bms_is_safe_to_discharge(void)
{
  bms_error = BMS_ERR_NONE;

  uint16_t *cell_voltages = bq7693_get_cell_voltages();
  // cell undervoltage
  for (int i=0; i<7;++i)
  {
    if (cell_voltages[i] < CELL_LOWEST_DISCHARGE_VOLTAGE)
    {
      bms_set_error(BMS_ERR_PACK_DISCHARGED);
      BMS_PRINT("BMS:CELL_LOW c=%d v=%dmV\r\n", i, cell_voltages[i]);
    }
  }
  // pack temperature
  pack_temperature = bms_read_temperature();
  int temp = pack_temperature / 10;

  if (temp  > MAX_PACK_TEMPERATURE)
  {
    bms_set_error(BMS_ERR_PACK_OVERTEMP);
    BMS_PRINT("%s : Pack overtemp %d 'C, max %d\r\n",__FUNCTION__ ,  temp, MAX_PACK_TEMPERATURE);
  }
  else if (temp < MIN_PACK_DISCHARGE_TEMP)
  {
    bms_set_error(BMS_ERR_PACK_UNDERTEMP);
    BMS_PRINT("%s: Pack undertemp %d 'C, min %d\r\n", __FUNCTION__ , temp, MIN_PACK_DISCHARGE_TEMP);
  }

  // read SYS_STAT once, clear flags, then evaluate
  uint8_t sys_stat;
  bq7693_read_register(SYS_STAT, 1, &sys_stat);

  if (sys_stat & STAT_FLAGS)
  {
    BMS_PRINT("%s: SYS_STAT=0x%02X\r\n", __FUNCTION__, sys_stat);
    bq7693_write_register(SYS_STAT, sys_stat & STAT_FLAGS);
  }

  if (sys_stat & STAT_OCD)
  {
    bms_set_error(BMS_ERR_OVERCURRENT);
    BMS_PRINT("%s: BMS IC Overcurrent Trip\r\n", __FUNCTION__);
  }
  if (sys_stat & STAT_SCD)
  {
    bms_set_error(BMS_ERR_SHORTCIRCUIT);
    BMS_PRINT("%s: BMS IC Short Circuit Trip\r\n", __FUNCTION__);
  }
  if (sys_stat & STAT_UV)
  {
    bms_set_error(BMS_ERR_UNDERVOLTAGE);
    BMS_PRINT("%s: BMS IC Undervoltage Trip\r\n", __FUNCTION__);
  }
  if (sys_stat & STAT_OV)
  {
    bms_set_error(BMS_ERR_OVERVOLTAGE);
    BMS_PRINT("%s: BMS IC Overvoltage Trip\r\n", __FUNCTION__);
  }

  // imbalance lock persists across reboots, live detection runs in
  // bms_is_safe_to_charge near the top of charge
  if (eeprom_data.imbalance_locked)
    bms_set_error(BMS_ERR_CELL_IMBALANCE);

  return (bms_error == BMS_ERR_NONE);
}

/**
 * @brief check whether the pack is safe to charge, sets bms_error on failure
 * @return true if safe
 */
static bool bms_is_safe_to_charge(void)
{
  bms_error = BMS_ERR_NONE;

  uint16_t *cell_voltages = bq7693_get_cell_voltages();

  // cell too flat to charge
  for (int i=0; i<7;++i)
  {
    if ( cell_voltages[i] < CELL_LOWEST_CHARGE_VOLTAGE )
    {
      bms_set_error(BMS_ERR_CELL_FAIL);
      BMS_PRINT("%s: Cell %d below min charge voltage %d, min %d\r\n", __FUNCTION__, i, cell_voltages[i], CELL_LOWEST_CHARGE_VOLTAGE);
    }
  }

  // pack temperature
  pack_temperature = bms_read_temperature();
  int temp = pack_temperature / 10;

  if (temp  > MAX_PACK_TEMPERATURE)
  {
    bms_set_error(BMS_ERR_PACK_OVERTEMP);
  }
  else if (temp < MIN_PACK_CHARGE_TEMP)
  {
    bms_set_error(BMS_ERR_PACK_UNDERTEMP);
  }

  // read SYS_STAT once, clear flags, then evaluate
  uint8_t sys_stat;
  bq7693_read_register(SYS_STAT, 1, &sys_stat);

  if (sys_stat & STAT_FLAGS)
  {
    BMS_PRINT("%s: SYS_STAT=0x%02X\r\n", __FUNCTION__, sys_stat);
    bq7693_write_register(SYS_STAT, sys_stat & STAT_FLAGS);
  }

  if (sys_stat & STAT_OCD)
  {
    bms_set_error(BMS_ERR_OVERCURRENT);
    bq7693_write_register(SYS_STAT, 0x01);
  }

  if (sys_stat & STAT_OV)
  {
    bms_set_error(BMS_ERR_OVERVOLTAGE);
    bq7693_write_register(SYS_STAT, 0x04);
  }

  // persistent imbalance lock blocks operation until the factory-reset
  // gesture, live detection runs near the top of charge where cells should
  // have converged, a spread above the threshold at that SoC means real
  // capacity mismatch, not load sag or curve-knee divergence
  if (eeprom_data.imbalance_locked)
  {
    bms_set_error(BMS_ERR_CELL_IMBALANCE);
  }
  else
  {
    uint16_t lo = cell_voltages[0];
    uint16_t hi = cell_voltages[0];
    for (int i = 1; i < 7; ++i)
    {
      if (cell_voltages[i] < lo) lo = cell_voltages[i];
      if (cell_voltages[i] > hi) hi = cell_voltages[i];
    }
    if (hi >= CELL_IMBALANCE_NEAR_FULL_MV && (uint16_t)(hi - lo) >= CELL_IMBALANCE_FAULT_MV)
    {
      bms_set_error(BMS_ERR_CELL_IMBALANCE);
      BMS_PRINT("BMS:IMBALANCE hi=%umV lo=%umV\r\n", hi, lo);
    }
  }

  return (bms_error == BMS_ERR_NONE);
}

/**
 * @brief has any cell hit the full-charge threshold, uses hysteresis
 * @return true if any cell is at or above the threshold
 */
static bool bms_is_pack_full(void)
{
  uint16_t *cell_voltages = bq7693_get_cell_voltages();

  // while charging, trip on the higher threshold; once full, hold via the lower release
  uint16_t threshold = (bms_state == BMS_CHARGING)
                     ? CELL_FULL_CHARGE_VOLTAGE          // 4170 mV
                     : CELL_FULL_CHARGE_RELEASE_VOLTAGE; // 4100 mV

  for (int i=0; i<7; ++i)
  {
    if (cell_voltages[i] >= threshold)
    {
      return true;
    }
  }

  return false;
}

/** @brief idle: wait for trigger, charger, or sleep timeout */
static void bms_handle_idle(void)
{
  uint32_t sleep_time;
  bool vacuum_was_connected = false;
  bool trigger_was_pressed  = false;
  uint8_t imbalance_idle_count = 0;

  sw_timer_start(&bms_timer);

  do
  {
    bool vacuum_connected = dsn_prot_get_vacuum_connected();
    bool trigger_pressed  = bms_trigger_active();

    if (vacuum_connected && !vacuum_was_connected)
    {
      if (bms_is_safe_to_discharge())
      {
        sw_timer_delay_ms(300);
        bq7693_enable_discharge();
      }
    }
    vacuum_was_connected = vacuum_connected;

    // idle imbalance check, only valid above the SoC knee where the
    // OCV curve is flat and spread reflects real capacity mismatch
    // rather than curve shape
    {
      uint16_t *cells = bq7693_get_cell_voltages();
      uint16_t lo = cells[0];
      uint16_t hi = cells[0];
      for (int i = 1; i < 7; ++i)
      {
        if (cells[i] < lo) lo = cells[i];
        if (cells[i] > hi) hi = cells[i];
      }
      if ( // lo >= CELL_IMBALANCE_IDLE_MIN_MV && 
           (uint16_t)(hi - lo) >= CELL_IMBALANCE_FAULT_MV)
      {
        if (imbalance_idle_count < CELL_IMBALANCE_IDLE_DEBOUNCE)
          imbalance_idle_count++;
      }
      else
      {
        imbalance_idle_count = 0;
      }

      if (imbalance_idle_count >= CELL_IMBALANCE_IDLE_DEBOUNCE)
      {
        BMS_PRINT("BMS:IMBALANCE_AT_IDLE hi=%umV lo=%umV\r\n", hi, lo);
        bms_error = BMS_ERR_CELL_IMBALANCE;
        bms_state = BMS_FAULT;
        return;
      }
    }

    // longer idle window when the vacuum is attached, shorter when it isn't
    if(true == vacuum_connected)
      sleep_time = (IDLE_TIME * 1000ul);
    else
      sleep_time = (20 * 1000ul);

    if (dio_read(DIO_CHARGER_CONNECTED) == true)
    {
      bms_state = BMS_CHARGER_CONNECTED;
      return;
    }
    else if (trigger_pressed)
    {
      if (!trigger_was_pressed)
      {
        leds_blink_leds(10);
        sw_timer_start(&bms_timer);
      }

      if (vacuum_connected)
      {
        bms_state = BMS_VACUUM_RUNNING;
        return;
      }
    }
    else if(force_sleep == true)
      sw_timer_stop(&bms_timer);                     // sleep now
    else if(dsn_prot_get_sleep_flag() == true)
      sw_timer_stop(&bms_timer);                     // vacuum requested sleep

    trigger_was_pressed = trigger_pressed;

    sw_timer_delay_ms(50);
    wdt_reset_count();

  } while (false == sw_timer_is_elapsed(&bms_timer, sleep_time));

  // idle timeout reached without trigger or charger, go to sleep
  bms_state = BMS_SLEEP;
}

/** @brief sleep: save EEPROM, disable FETs, put BQ7693 into SHIP mode */
static void bms_handle_sleep(void)
{
  bms_wdt_deinit();
  serial_debug_send_message("BMS:GOING_TO_SLEEP\r\n");
  bq7693_disable_charge();
  bq7693_disable_discharge();

  leds_sequence();

  pins_deinit();

  delay_ms(1000);

  eeprom_write();

  bq7693_enter_sleep_mode();

  // we will be powered down before this returns
  while(1);
}

/** @brief vacuum running: monitor safety while the trigger is held and the vacuum is connected */
static void bms_handle_vacuum_running(void)
{
#ifdef SERIAL_DEBUG
  uint8_t debug_print_cnt = 0;
#endif

  if (!bms_is_safe_to_discharge())
  {
    dsn_prot_set_trigger(false);
    bms_state = BMS_FAULT;
    return;
  }
  dsn_prot_set_trigger(true);

  while (1)
  {
    if (!bms_is_safe_to_discharge())
    {
      dsn_prot_set_trigger(false);
      bms_state = BMS_FAULT;
      return;
    }

    if (!bms_trigger_active() || !dsn_prot_get_vacuum_connected())
    {
      dsn_prot_set_trigger(false);
      leds_off();
      bms_state = BMS_IDLE;
      return;
    }

#ifdef SERIAL_DEBUG
    if(++debug_print_cnt > 5)
    {
      BMS_PRINT("BMS:VACUUM_RUNNING I:%d mA @ %ld mAH, C:%ld mAH, T:%d 'C, P:%d mV\r\n", abs(current_filt_mA), (eeprom_data.current_charge_level / 1000), (eeprom_data.total_pack_capacity / 1000), (int16_t)(pack_temperature / 10), bq7693_get_pack_voltage());
      debug_print_cnt = 0;
    }
#endif


    sw_timer_delay_ms(60);
  }
}

/**
 * @brief fault: blink the error code on the LEDs and wait for user action,
 *        exit paths are charger plug-in -> BMS_CHARGER_CONNECTED,
 *        20-press gesture -> factory reset and BMS_IDLE,
 *        trigger rising edge (momentary-mode builds only) -> BMS_IDLE,
 *        auto-recover retry passes (transient faults only) -> BMS_IDLE,
 *        the LED pattern is a non-blocking state machine ticked every 20 ms
 *        so trigger presses are never missed, even on long codes like
 *        BMS_ERR_CELL_IMBALANCE (11 blinks)
 */
static void bms_handle_fault(void)
{
  const enum BMS_ERROR_CODE original_error = bms_error;
  const bool auto_recover = (original_error == BMS_ERR_PACK_UNDERTEMP
                          || original_error == BMS_ERR_PACK_OVERTEMP
                          || original_error == BMS_ERR_OVERCURRENT
                          || original_error == BMS_ERR_SHORTCIRCUIT);

  BMS_PRINT("BMS:FAULT err=%d auto_recover=%d\r\n", original_error, auto_recover);

  // persist fault-dependent state in a single eeprom_write(),
  // cell imbalance latches a flag that blocks charge/discharge until
  // the user performs the factory reset, pack-discharged / UV anchors
  // the coulomb counter to 0 so capacity learning starts clean on the
  // next full charge
  if (original_error == BMS_ERR_CELL_IMBALANCE)
  {
    eeprom_data.imbalance_locked = 1;
  }
  else if (original_error == BMS_ERR_PACK_DISCHARGED
        || original_error == BMS_ERR_UNDERVOLTAGE)
  {
    eeprom_data.current_charge_level = 0;
    eeprom_data.full_discharge_seen  = 1;
  }
  eeprom_write();

  leds_off();
  dsn_prot_set_trigger(false);
  bq7693_disable_discharge();
  port_pin_set_output_level(ENABLE_CHARGE_PIN, false);

  // LED pattern timing
  const uint32_t tick_ms     = 20;
  const uint32_t half_ms     = 250;    // one half-blink (on or off)
  const uint32_t pause_ms    = 2000;   // pause between groups
  const uint8_t  blink_total = bms_error;

  enum { PHASE_ON, PHASE_OFF, PHASE_PAUSE } phase = PHASE_ON;
  uint8_t  blink_idx = 0;
  uint32_t phase_ms  = 0;

  // factory-reset gesture state (raw trigger edges, safe in V12 toggle mode)
  uint8_t  reset_count   = 0;
  bool     reset_prev    = dio_read(DIO_TRIGGER_PRESSED);
  sw_timer reset_timeout = 0;

  sw_timer retry_timer = 0;
  if (auto_recover)
    sw_timer_start(&retry_timer);

#if !TRIGGER_TOGGLE_MODE
  // single-press exit, disabled in toggle mode where any press would flip
  // the latch and immediately leave the fault, toggle users exit via the
  // 20-press gesture or by plugging in the charger
  bool trigger_prev = bms_trigger_active();
#endif

  leds_on();

  while (1)
  {
    sw_timer_delay_ms(tick_ms);
    wdt_reset_count();
    phase_ms += tick_ms;

    if (dio_read(DIO_CHARGER_CONNECTED))
    {
      leds_off();
      bms_state = BMS_CHARGER_CONNECTED;
      return;
    }

    if (bms_factory_reset_check(&reset_count, &reset_prev, &reset_timeout))
    {
      // imbalance-lock recovery: clear only the lock flag and keep the
      // learned capacity and coulomb counter, other faults fall through
      // to a full factory reset (handled elsewhere)
      if (original_error == BMS_ERR_CELL_IMBALANCE)
      {
        eeprom_data.imbalance_locked = 0;
        eeprom_write();
      }

      leds_off();
      leds_blink_leds_num(LEDS_LED_ERR_LEFT, 10, 100);
      bms_state = BMS_IDLE;
      return;
    }

#if !TRIGGER_TOGGLE_MODE
    bool trigger_now = bms_trigger_active();
    if (trigger_now && !trigger_prev)
    {
      leds_off();
      bms_state = BMS_IDLE;
      return;
    }
    trigger_prev = trigger_now;
#endif

    if (auto_recover && sw_timer_is_elapsed(&retry_timer, 5000))
    {
      if (bms_is_safe_to_discharge())
      {
        BMS_PRINT("BMS:FAULT_RECOVERED err=%d\r\n", original_error);
        bms_error = BMS_ERR_NONE;
        leds_off();
        bms_state = BMS_IDLE;
        return;
      }
      bms_error = original_error;
      sw_timer_start(&retry_timer);
    }

    // LED pattern: ON -> OFF -> (next blink | PAUSE) -> ON ...
    switch (phase)
    {
      case PHASE_ON:
        if (phase_ms >= half_ms) { leds_off(); phase = PHASE_OFF; phase_ms = 0; }
        break;
      case PHASE_OFF:
        if (phase_ms >= half_ms)
        {
          if (++blink_idx < blink_total) { leds_on(); phase = PHASE_ON; }
          else                           {            phase = PHASE_PAUSE; }
          phase_ms = 0;
        }
        break;
      case PHASE_PAUSE:
        if (phase_ms >= pause_ms)
        {
          blink_idx = 0;
          leds_on();
          phase    = PHASE_ON;
          phase_ms = 0;
        }
        break;
    }
  }
}

/** @brief charger connected: evaluate the pack and start charging or report full */
static void bms_handle_charger_connected(void)
{
  // clear any pre-existing trigger intent on plug-in, without this the
  // dsn-protocol trigger_state or the V12 toggle latch could survive the
  // charge cycle and start the motor when the charger is later removed
  dsn_prot_set_trigger(false);
  bms_trigger_active();

  if (bms_is_pack_full())
  {
    bms_state = BMS_CHARGER_CONNECTED_NOT_CHARGING;
  }
  else if (bms_is_safe_to_charge())
  {
    bms_state = BMS_CHARGING;
  }
  else
  {
    bms_state = BMS_FAULT;
  }
}

/** @brief charger present but not charging: standby with periodic top-up checks */
static void bms_handle_charger_connected_not_charging(void)
{
  // drop the V12 toggle latch so the vacuum doesn't auto-start when the
  // charger is later removed, bms_trigger_active() returns early on a
  // non-sampling state and clears its internal latch
  bms_trigger_active();

  leds_blink_leds(2000);

  while(1)
  {
    if (!dio_read(DIO_CHARGER_CONNECTED))
    {
      bms_state = BMS_IDLE;
      return;
    }
    else if(dsn_prot_get_sleep_flag() == true)
    {
      rtc_standby_timer_start();
      bms_enter_standby();
      serial_debug_send_message("BMS_STANDBY\r\n");
      leds_blink_leds_num(LEDS_NUM, 4, 100);

      // sleep until RTC tick or external wake event
      system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
      system_sleep();

      bms_leave_standby();
      rtc_standby_timer_stop();

      if (rtc_wakeup_flag)
      {
        rtc_wakeup_flag = false;
        serial_debug_send_message("BMS:RTC_WAKE\r\n");
      }
      else
      {
        serial_debug_send_message("BMS:EIC_WAKE\r\n");
        dsn_prot_reset();
        leds_blink_leds_num(LEDS_NUM, 2, 100);
        // give the vacuum time to reconnect
        sw_timer_delay_ms(250);
      }

      // top up if the pack has dropped below the full threshold
      if (!bms_is_pack_full())
      {
        bms_state = BMS_CHARGER_CONNECTED;
        return;
      }
    }

    sw_timer_delay_ms(250);
  }
}

/** @brief charging: drive the charge cycle with pause/retry and capacity learning */
static void bms_handle_charging(void)
{
  uint8_t charging_leds_duty = 0;
#ifdef SERIAL_DEBUG
  uint8_t debug_print_cnt = 0;
#endif

  // 20 trigger presses while charging resets the learned pack capacity
  uint8_t  reset_count   = 0;
  bool     reset_prev    = false;
  sw_timer reset_timeout = 0;

  if (!bms_is_safe_to_charge())
  {
    bms_state = BMS_FAULT;
    return;
  }

  // disable the discharge FET, dsn_protocol keeps the precharge line asserted
  // so the vacuum keeps logic power
  bq7693_disable_discharge();

  // enable charging: external charge enable pin, then BQ7693 charge FET
  port_pin_set_output_level(ENABLE_CHARGE_PIN, true);
  bq7693_enable_charge();

  charge_pause_counter = 0;

  while (1)
  {
     #define DUTY_MAX    100
     uint8_t duty_loc = charging_leds_duty;

     if(charging_leds_duty > DUTY_MAX)
     {
       duty_loc = ((DUTY_MAX * 2) - charging_leds_duty);
     }

     (duty_loc < 10) ? duty_loc = 0 : (duty_loc);

     leds_set_led_duty(LEDS_LED_ERR_RIGHT, duty_loc);
     leds_set_led_duty(LEDS_LED_ERR_LEFT,  duty_loc);
     charging_leds_duty = (charging_leds_duty + ((charging_leds_duty > 20) ? 10 : 1)) % ((DUTY_MAX * 2) + 1);

    if (bms_factory_reset_check(&reset_count, &reset_prev, &reset_timeout))
    {
      // reset EEPROM and bail out of the charge cycle
      eeprom_write_defaults();
      eeprom_write();
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();
      leds_off();
      leds_blink_leds_num(LEDS_LED_ERR_LEFT, 10, 100);
      bms_state = BMS_CHARGER_CONNECTED;
      return;
    }

    if (!bms_is_safe_to_charge())
    {
      // safety error: stop charging and fault out
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      leds_off();
      bms_state = BMS_FAULT;
      return;
    }

    if ( !dio_read(DIO_CHARGER_CONNECTED))
    {
      // charger unplugged
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      // re-enable the discharge FET only if a vacuum is currently connected,
      // otherwise the idle loop's vacuum-connect edge will do it
      if (dsn_prot_get_vacuum_connected() && bms_is_safe_to_discharge())
      {
        bq7693_enable_discharge();
      }

      leds_off();
      bms_state = BMS_CHARGER_UNPLUGGED;
      return;
    }

    if (bms_is_pack_full())
    {
      charging_leds_duty = 0;
      leds_off();
#ifdef SERIAL_DEBUG
      BMS_PRINT("BMS:CHARGING Paused - full, attempt %d of %d\r\n", charge_pause_counter, FULL_CHARGE_PAUSE_COUNT);
      serial_debug_send_cell_voltages();
      debug_print_cnt = 0;
#endif
      // pause charging
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      // wait 30 s, then retry, bail early if the charger is unplugged
      for (int i=0; i<30; ++i)
      {
        sw_timer_delay_ms(1000);
        wdt_reset_count();
        if (!dio_read(DIO_CHARGER_CONNECTED))
        {
          if (dsn_prot_get_vacuum_connected() && bms_is_safe_to_discharge())
          {
            bq7693_enable_discharge();
          }
          leds_off();
          bms_state = BMS_CHARGER_UNPLUGGED;
          return;
        }
      }
      charge_pause_counter++;
      // resume charging
      port_pin_set_output_level(ENABLE_CHARGE_PIN, true);
      bq7693_enable_charge();
    }
    else
    {
#ifdef SERIAL_DEBUG
      if(++debug_print_cnt > 5)
      {
        BMS_PRINT("BMS:CHARGING I:%d mA @ %ld mAH, C:%ld mAH, T:%d 'C, P:%d mV\r\n", abs(current_filt_mA), (eeprom_data.current_charge_level / 1000), (eeprom_data.total_pack_capacity / 1000), (int16_t)(pack_temperature / 10), bq7693_get_pack_voltage());
        debug_print_cnt = 0;
      }
#endif
    }

    if (charge_pause_counter >= FULL_CHARGE_PAUSE_COUNT)
    {
      // full after FULL_CHARGE_PAUSE_COUNT pauses, disable charging
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      leds_off();

      bms_state = BMS_CHARGER_CONNECTED_NOT_CHARGING;

      // capacity learning, after a confirmed full discharge cycle snap
      // total capacity to the just-measured charge level, otherwise apply
      // a slow decay (never increase) to filter noise
      if (eeprom_data.full_discharge_seen)
      {
        eeprom_data.total_pack_capacity = eeprom_data.current_charge_level;
        eeprom_data.full_discharge_seen = 0;
      }
      else
      {
        int32_t gap = eeprom_data.total_pack_capacity - eeprom_data.current_charge_level;
        if (gap > 0)
          eeprom_data.total_pack_capacity -= gap >> 3;
      }

      // clamp to a sane upper bound
      if (eeprom_data.total_pack_capacity > (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH)
        eeprom_data.total_pack_capacity = (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH;

      // pack is now full
      eeprom_data.current_charge_level = eeprom_data.total_pack_capacity;

      BMS_PRINT("BMS:CHARGING Stopped\r\n");
#ifdef SERIAL_DEBUG
      serial_debug_send_pack_capacity();
#endif
      return;
    }


    sw_timer_delay_ms(50);
  }
}

/** @brief charger unplugged: blink an LED indication of cell spread, then go idle */
static void bms_handle_charger_unplugged(void)
{
  uint16_t spread = bms_get_cell_spread_mv();

  // 100 ms blink per 50 mV of spread
  for (int i = 0; i < (int)(spread / 50); ++i)
  {
    leds_blink_leds(100);
  }

#ifdef SERIAL_DEBUG
  BMS_PRINT("Charger unplugged\r\n");
  serial_debug_send_cell_voltages();
#endif

  bms_state = BMS_IDLE;
}

/** @brief RTC compare-match callback, sets the flag that wakes the MCU from standby */
static void rtc_wakeup_callback(void)
{
  rtc_wakeup_flag = true;
}

/** @brief one-time RTC setup: configure the peripheral and register the callback */
static void rtc_standby_timer_init(void)
{
  struct rtc_count_config config;

  rtc_count_get_config_defaults(&config);
  config.prescaler         = RTC_COUNT_PRESCALER_DIV_1024;
  config.mode              = RTC_COUNT_MODE_32BIT;
  config.clear_on_match    = true;
  config.compare_values[0] = RTC_STANDBY_WAKE_TICKS;

  rtc_count_init(&rtc_instance, RTC, &config);

  rtc_count_register_callback(&rtc_instance, rtc_wakeup_callback,
                              RTC_COUNT_CALLBACK_COMPARE_0);
}

/** @brief start the RTC standby wakeup timer, resets the count and enables it */
static void rtc_standby_timer_start(void)
{
  rtc_count_set_count(&rtc_instance, 0);
  rtc_wakeup_flag = false;

  // clear stale compare-match flag and pending NVIC IRQ, rtc_count_disable()
  // doesn't clear the NVIC pending bit so an old IRQ would fire as soon as
  // rtc_count_enable() re-enables interrupts and would set rtc_wakeup_flag
  // before we ever reach standby
  RTC->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_MASK;
  NVIC_ClearPendingIRQ(RTC_IRQn);

  rtc_count_enable_callback(&rtc_instance, RTC_COUNT_CALLBACK_COMPARE_0);
  rtc_count_enable(&rtc_instance);
  // wait for the ENABLE write to sync across clock domains before standby
  while (RTC->MODE0.STATUS.reg & RTC_STATUS_SYNCBUSY);
}

/** @brief stop and disable the RTC standby wakeup timer */
static void rtc_standby_timer_stop(void)
{
  rtc_count_disable_callback(&rtc_instance, RTC_COUNT_CALLBACK_COMPARE_0);
  rtc_count_disable(&rtc_instance);
}

/** @brief enter standby: switch EIC to the low-power oscillator and enable wake sources */
static void bms_enter_standby(void)
{
  struct system_gclk_chan_config gclk_chan_conf;

  bms_wdt_deinit();

  // reroute EIC to the 32 kHz low-power oscillator (GCLK3)
  _extint_disable();
  system_gclk_chan_disable(EIC_GCLK_ID);
  system_gclk_chan_get_config_defaults(&gclk_chan_conf);
  gclk_chan_conf.source_generator = GCLK_GENERATOR_3;
  system_gclk_chan_set_config(EIC_GCLK_ID, &gclk_chan_conf);
  system_gclk_chan_enable(EIC_GCLK_ID);
  _extint_enable();

  // enable wake sources: mode button, trigger, charger
  extint_chan_enable_callback(9, EXTINT_CALLBACK_TYPE_DETECT);  // MODE_BUTTON            (EXTINT 9, PA09)
  extint_chan_enable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);  // TRIGGER_PRESSED_PIN    (EXTINT 4, PA04)
  extint_chan_enable_callback(6, EXTINT_CALLBACK_TYPE_DETECT);  // CHARGER_CONNECTED_PIN  (EXTINT 6, PA06)
  // disable BQ7693 ALERT during standby
  extint_chan_disable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);
}

/** @brief leave standby: switch EIC back to the main clock and disable wake sources */
static void bms_leave_standby(void)
{
  struct system_gclk_chan_config gclk_chan_conf;

  // reroute EIC back to the main oscillator (GCLK0)
  _extint_disable();
  system_gclk_chan_disable(EIC_GCLK_ID);
  system_gclk_chan_get_config_defaults(&gclk_chan_conf);
  gclk_chan_conf.source_generator = GCLK_GENERATOR_0;
  system_gclk_chan_set_config(EIC_GCLK_ID, &gclk_chan_conf);
  system_gclk_chan_enable(EIC_GCLK_ID);
  _extint_enable();

  // disable wake sources, re-enable BQ7693 ALERT
  extint_chan_disable_callback(9, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_disable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_disable_callback(6, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_enable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);

  bms_wdt_init();
}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
