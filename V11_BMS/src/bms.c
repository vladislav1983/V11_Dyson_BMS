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

// RTC standby wake timer: GCLK2 = ULP32K/32 (1024 Hz), RTC prescaler = DIV1024 → 1 Hz
// N days = N * 86400 seconds × 1 tick/sec
#define RTC_STANDBY_WAKE_TICKS  ((uint32_t)2 * (24UL * 60UL * 60UL))

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
//We start off idle.
static enum BMS_STATE bms_state      = BMS_INIT;
//If a fault occurs, it'll be lodged here.
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
/** @brief Initialize all BMS hardware: clocks, timers, pins, ADC, BQ7693, LEDs, EEPROM, UART. */
void bms_init(void)
{
  //sets up clocks/IRQ handlers etc.
  system_init();
  //Initialise the sw_timer
  delay_init();
  sw_timer_init();
  dsn_prot_init();

  //Set up the pins
  pins_init();
  dio_init();

  bms_adc_init();
  //BQ7693 init
  bq7693_init();

  //Init the LEDs
  leds_init();
  //Init eeprom emulator
  eeprom_init();
  eeprom_read();

  //Initialise the USART we need to talk to the vacuum cleaner
  serial_init();

  //Enable interrupts
  interrupts_init();

  //Initialise RTC for standby wakeup (one-time config)
  rtc_standby_timer_init();

#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
  serial_debug_init();
#endif
}

/** @brief External interrupt callback for wakeup events (unused). */
void bms_wakeup_interrupt_callback(void)
{

}

/** @brief BQ7693 ALERT pin interrupt callback, sets processing flag. */
void bms_interrupt_callback(void)
{
  process_bms_interrupt = true;
}

/** @brief Process pending BQ7693 interrupt: read coulomb counter and update charge level. */
void bms_interrupt_process(void)
{
  uint8_t sys_stat;

  if(true == process_bms_interrupt)
  {
    bq7693_read_register(SYS_STAT, 1, &sys_stat);

    if (sys_stat & 0x80)
    {
      //Got a coulomb charger count ready.
      int32_t ccVal = bq7693_read_cc();

      //This needs better handling....
      current_mA = (ccVal * (uint16_t)(8.44f * 4096.0f)) / 4096;
      #define FILT_MS   (500ul)
      #define PERIOD_MS (250ul)
      current_filt_sum_mA += ( (int32_t)((65536.0 * PERIOD_MS) / FILT_MS) * (int16_t)(current_mA - (int16_t)(current_filt_sum_mA >> 16) ) );
      current_filt_mA = current_filt_sum_mA >> 16;

      //Ignore tiny values.
      //if ( (ccVal > 0 && ccVal > 2)  || (ccVal < 0 && ccVal < -2) )
      {
        int32_t cc_uah;
        //i = V/R
        //sense resistor = 1mOhm
        //microV / milliOhms gives current in mA.
        //so ccVal has current in mA.
        //Dividing by 14400 would give mAH. (number of 250mS periods in 1 hr.
        //Dividing by 14.4 will give microAH (what we want)
        // 14.4 = ((3600 * 1000) / 250ms) / 1000mAh
        cc_uah = ccVal * (int16_t)(((8.44f * 250.0f * 32768.0f) / (3600.0f)));
        cc_uah /= 32768;
        eeprom_data.current_charge_level += cc_uah;

        // Clamp charge level to valid range
        if (eeprom_data.current_charge_level > eeprom_data.total_pack_capacity)
          eeprom_data.current_charge_level = eeprom_data.total_pack_capacity;
        if (eeprom_data.current_charge_level < 0)
          eeprom_data.current_charge_level = 0;
      }
      //Update the CC bit so it'll refire in another 250mS as per datasheet.
      bq7693_write_register(SYS_STAT, 0x80);//Clear CC bit.
    }

    process_bms_interrupt = false;
  }
}

/**
 * @brief Get state of charge as percent * 100 for vacuum protocol.
 * @return SOC in 0.01% units (100-10000), minimum 1% to avoid critical battery screen.
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
 * @brief Estimate remaining runtime based on filtered current.
 * @return seconds, 0 when idle, minimum 60 during discharge.
 */
uint32_t bms_get_runtime_seconds(void)
{
  int32_t current_filt_mA_abs = abs(current_filt_mA);
  int32_t current_charge_level;
  int32_t runtime = 0;

  if(    bms_state == BMS_VACUUM_RUNNING // estimate only while the motor is actually running
      && current_filt_mA_abs > 1000)     // and current is > 1A, to keep the result bounded
  {
    // clamp to [0, PACK_MAX_CAPACITY_MAH * 1000]
    current_charge_level = eeprom_data.current_charge_level < 0 ? 0
                         : eeprom_data.current_charge_level > (PACK_MAX_CAPACITY_MAH * 1000) ? (PACK_MAX_CAPACITY_MAH * 1000)
                         : eeprom_data.current_charge_level;

    runtime = ((current_charge_level / current_filt_mA_abs) * (uint16_t)((3600.0f / 1000.0f) * 1024.0f)) >> 10;

    // always limit runtime to 1 minute
    runtime = runtime < 60 ? 60 : runtime;
  }

  return (uint32_t)runtime;
}

/** @brief Main BMS state machine loop (never returns). */
void bms_mainloop(void)
{
  bms_wdt_init();
  //Handle the state machinery.
  while (1)
  {
    BMS_PRINT("BMS_STATE: %s\r\n", bms_state_names[bms_state]);

    switch (bms_state)
    {
    //-----------------------------------------------------------------------
      case BMS_INIT:
        bms_state = BMS_IDLE;
#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
        //Initial debug blurb
        serial_debug_send_message("Dyson V11/V15 BMS After market firmware\r\n");
#endif
        leds_sequence();
        wdt_reset_count();

#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
        //Initial debug blurb
        serial_debug_send_cell_voltages();
        serial_debug_send_pack_capacity();
#endif
        wdt_reset_count();
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
/** @brief Set bms_error to code only if code is more severe than the current error. */
static void bms_set_error(enum BMS_ERROR_CODE code)
{
  if (bms_error < code)
    bms_error = code;
}

/** @brief Force the state machine into BMS_FAULT with the given error code. ISR-safe. */
void bms_force_fault(enum BMS_ERROR_CODE code)
{
  bms_error = code;
  bms_state = BMS_FAULT;
}

/** @brief Configure GPIO pins for charge control, sense inputs, and precharge. */
static void pins_init(void)
{
  //Set up the output charge pin

  struct port_config charge_pin_config;
  port_get_config_defaults(&charge_pin_config);
  charge_pin_config.direction = PORT_PIN_DIR_OUTPUT;
  port_pin_set_config(ENABLE_CHARGE_PIN, &charge_pin_config);
  port_pin_set_output_level(ENABLE_CHARGE_PIN, false);

  //Two input pins, CHARGER and TRIGGERs
  struct port_config sense_pin_config;
  port_get_config_defaults(&sense_pin_config);
  sense_pin_config.direction = PORT_PIN_DIR_INPUT;
  sense_pin_config.input_pull = PORT_PIN_PULL_NONE;
  port_pin_set_config(CHARGER_CONNECTED_PIN, &sense_pin_config);
  port_pin_set_config(TRIGGER_PRESSED_PIN, &sense_pin_config);


  struct port_config io_pin_config;
  port_get_config_defaults(&io_pin_config);
  io_pin_config.direction = PORT_PIN_DIR_OUTPUT;

  // pack voltage feedback
  port_pin_set_config(PIN_PA03, &io_pin_config);
  port_pin_set_output_level(PIN_PA03, true);

  // mode pin pullup voltage
  port_pin_set_config(MODE_BUTTON_PULLUP_ENABLE_PIN, &io_pin_config);
  port_pin_set_output_level(MODE_BUTTON_PULLUP_ENABLE_PIN, true);

  // precharge
  port_pin_set_config(PRECHARGE_PIN, &io_pin_config);
  port_pin_set_output_level(PRECHARGE_PIN, false);

  // unknown functionality pin
  //port_pin_set_config(PIN_PA25, &io_pin_config);
  //port_pin_set_output_level(PIN_PA25, true);

  // mode button
  port_pin_set_config(MODE_BUTTON_PIN, &sense_pin_config);
}

/** @brief De-initialize GPIO outputs before entering sleep. */
static void pins_deinit(void)
{
  port_pin_set_output_level(PIN_PA03, false);
  port_pin_set_output_level(MODE_BUTTON_PULLUP_ENABLE_PIN, false);
  port_pin_set_output_level(PRECHARGE_PIN, false);
}

/** @brief Configure EIC for BQ7693 ALERT, mode button, trigger, and charger pins. */
static void interrupts_init(void)
{
  //A single interrupt, focused on the BQ7693's alert line (PA28), which
  //is on EXTINT 8.
  struct extint_chan_conf config_alert_pin;
  extint_chan_get_config_defaults(&config_alert_pin);
  config_alert_pin.wake_if_sleeping = false;
  config_alert_pin.gpio_pin     = PIN_PA28A_EIC_EXTINT8;
  config_alert_pin.gpio_pin_mux = MUX_PA28A_EIC_EXTINT8;
  //This line is designed to be possible for either device to pull it up or down to indicate a fault condition, so
  //no pullups.
  config_alert_pin.gpio_pin_pull      = EXTINT_PULL_NONE;
  config_alert_pin.detection_criteria = EXTINT_DETECT_RISING;

  extint_chan_set_config(8, &config_alert_pin);
  extint_register_callback(bms_interrupt_callback, 8, EXTINT_CALLBACK_TYPE_DETECT);
  extint_chan_enable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);

  //-----------------------------------------------------------------------------
  // mode pin interrupt generation, used to exit from standby mode - rising edge
  //is on EXTINT 9 - PA09
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
  // trigger pin, can also wakeup the system, rising edge
  // is on EXTINT 4 - PA04
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
  // charger connected pin, can also wakeup the system, both edges will wakeup the system
  // is on EXTINT 6 - PA06
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

  //Enable interrupts.
  system_interrupt_enable_global();
}

/**
 * @brief Read pack temperature from NTC thermistor
 * @return temperature in 0.1 degC, 2560 on sensor disagreement.
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

/**
 * @brief Check if pack conditions allow discharge.
 * @return true if safe.
 */
static bool bms_is_safe_to_discharge(void)
{
  //Clear error status.
  bms_error = BMS_ERR_NONE;

  uint16_t *cell_voltages = bq7693_get_cell_voltages();
  //Check any cells undervolt.
  for (int i=0; i<7;++i)
  {
    if (cell_voltages[i] < CELL_LOWEST_DISCHARGE_VOLTAGE)
    {
      bms_set_error(BMS_ERR_PACK_DISCHARGED);
      BMS_PRINT("BMS:CELL_LOW c=%d v=%dmV\r\n", i, cell_voltages[i]);
    }
  }
  //Check pack temperature remains in acceptable range
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

  //Check sys_stat — read once, clear all fault flags, then evaluate.
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

  if (bms_error == BMS_ERR_NONE)
    return true;
  else
    return false;
}

/**
 * @brief Check if pack conditions allow charging.
 * @return true if safe.
 */
static bool bms_is_safe_to_charge(void)
{
  //Clear error status.
  bms_error = BMS_ERR_NONE;

  uint16_t *cell_voltages = bq7693_get_cell_voltages();

  //Check no cells are so flat they cannot be charged.
  for (int i=0; i<7;++i)
  {
    if ( cell_voltages[i] < CELL_LOWEST_CHARGE_VOLTAGE )
    {
      bms_set_error(BMS_ERR_CELL_FAIL);
      BMS_PRINT("%s: Cell %d below min charge voltage %d, min %d\r\n", __FUNCTION__, i, cell_voltages[i], CELL_LOWEST_CHARGE_VOLTAGE);
    }
  }

  //Check pack temperature acceptable (<=60'C)
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

  //Check sys_stat — read once, clear all fault flags, then evaluate.
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

  if (bms_error == BMS_ERR_NONE)
    return true;
  else
    return false;
}

/**
 * @brief Check if any cell reached full charge voltage (with hysteresis).
 * @return true if any cell is at/above threshold.
 */
static bool bms_is_pack_full(void)
{
  uint16_t *cell_voltages = bq7693_get_cell_voltages();

  // Use hysteresis: apply the lower release threshold when pack was already full
  uint16_t threshold = (bms_state == BMS_CHARGING)
                     ? CELL_FULL_CHARGE_VOLTAGE          // 4170mV
                     : CELL_FULL_CHARGE_RELEASE_VOLTAGE; // 4100mV

  for (int i=0; i<7; ++i)
  {
    if (cell_voltages[i] >= threshold)
    {
      return true;
    }
  }

  return false;
}

/** @brief Idle state: wait for trigger, charger, or sleep timeout. */
static void bms_handle_idle(void)
{
  uint32_t sleep_time;
  bool vacuum_was_connected = false;
  bool trigger_was_pressed  = false;

  sw_timer_start(&bms_timer);

  do
  {
    bool vacuum_connected = dsn_prot_get_vacuum_connected();
    bool trigger_pressed  = (dio_read(DIO_TRIGGER_PRESSED) == true);

    if (vacuum_connected && !vacuum_was_connected)
    {
      if (bms_is_safe_to_discharge())
      {
        sw_timer_delay_ms(300);
        bq7693_enable_discharge();
      }
    }
    vacuum_was_connected = vacuum_connected;

    if(true == vacuum_connected)
    {
      sleep_time = (IDLE_TIME * 1000ul);
    }
    else
    {
      sleep_time = (20 * 1000ul); // 20 sec
    }

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
    {
      sw_timer_stop(&bms_timer); // go to sleep
    }
    else if(dsn_prot_get_sleep_flag() == true)
    {
      sw_timer_stop(&bms_timer); // go to sleep requested by cleaner
    }
    trigger_was_pressed = trigger_pressed;


    sw_timer_delay_ms(50);
    wdt_reset_count();

  } while (false == sw_timer_is_elapsed(&bms_timer, sleep_time));

  //Reached the end of our wait loop, with nobody pulling the trigger, or plugging in charger.
  //Transit to sleep state
  bms_state = BMS_SLEEP;
}

/** @brief Sleep: save EEPROM, disable FETs, enter BQ7693 SHIP mode. */
static void bms_handle_sleep(void)
{
  bms_wdt_deinit();
  serial_debug_send_message("BMS:GOING_TO_SLEEP\r\n");
  bq7693_disable_charge();
  bq7693_disable_discharge();

  leds_sequence();

  pins_deinit();

  delay_ms(1000);

  //Store pack charge data to eeprom
  eeprom_write();

  bq7693_enter_sleep_mode();

  //We are about to get powered down.
  while(1);
}

/** @brief Vacuum running: monitor safety while trigger held and vacuum connected. */
static void bms_handle_vacuum_running(void)
{
#ifdef SERIAL_DEBUG
  uint8_t debug_print_cnt = 0;
#endif

  // Entry-side safety: if unsafe, route to FAULT immediately so the fault
  // handler can do capacity learning. Previously the unsafe verdict was
  // silently dropped and the in-loop recheck could see cells recover once
  // the motor coasted down.
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

    if (!dio_read(DIO_TRIGGER_PRESSED) || !dsn_prot_get_vacuum_connected())
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

/** @brief Fault: display error, retry safety for transient faults, keep protocol alive. */
static void bms_handle_fault(void)
{
  // Transient faults clear once the underlying condition resolves (pack cools,
  // overcurrent latch settles). Keep probing bms_is_safe_to_discharge and exit
  // back to idle when it passes; other codes wait for user action as before.
  const enum BMS_ERROR_CODE original_error = bms_error;
  const bool auto_recover = (original_error == BMS_ERR_PACK_UNDERTEMP
                          || original_error == BMS_ERR_PACK_OVERTEMP
                          || original_error == BMS_ERR_OVERCURRENT
                          || original_error == BMS_ERR_SHORTCIRCUIT);
  sw_timer retry_timer = 0;

  BMS_PRINT("BMS:FAULT err=%d auto_recover=%d\r\n", original_error, auto_recover);
  eeprom_write();

  leds_off();
  dsn_prot_set_trigger(false);
  bq7693_disable_discharge();
  port_pin_set_output_level(ENABLE_CHARGE_PIN, false);

  if (auto_recover)
    sw_timer_start(&retry_timer);

  // Persist the pack-discharged anchor once so capacity learning can run on the
  // next full charge, regardless of how long the user watches the fault blink.
  if (bms_error == BMS_ERR_PACK_DISCHARGED || bms_error == BMS_ERR_UNDERVOLTAGE)
  {
    eeprom_data.current_charge_level = 0;
    eeprom_data.full_discharge_seen = 1;
  }

  // Blink the error pattern until the user acknowledges by releasing the
  // trigger AND pressing it again (rising edge after fault entry), or until
  // the charger is plugged in, or an auto-recoverable fault clears on retry.
  bool trigger_state = dio_read(DIO_TRIGGER_PRESSED);

  while (1)
  {
    // Blink the error code N times (N = bms_error), then pause so the user
    // can count the pattern. Pack-discharged / undervoltage blink once each.
    for (int i = 0; i < bms_error; ++i)
    {
      leds_blink_leds(500);
      wdt_reset_count();
    }
    sw_timer_delay_ms(2000);
    wdt_reset_count();

    if (dio_read(DIO_CHARGER_CONNECTED))
    {
      bms_state = BMS_CHARGER_CONNECTED;
      return;
    }

    bool trigger_now = dio_read(DIO_TRIGGER_PRESSED);
    if (trigger_now && !trigger_state)
    {
      // Rising edge after fault entry — explicit acknowledgement.
      leds_off();
      bms_state = BMS_IDLE;
      return;
    }
    trigger_state = trigger_now;

    if (auto_recover && sw_timer_is_elapsed(&retry_timer, 5000))
    {
      // bms_is_safe_to_discharge() clobbers bms_error; restore the original
      // so the LED blink code stays stable if the retry fails.
      bool safe = bms_is_safe_to_discharge();
      if (safe)
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
  }
}

/** @brief Charger connected: evaluate pack and begin charging or report full. */
static void bms_handle_charger_connected(void)
{
  if (bms_is_pack_full())
  {
    bms_state = BMS_CHARGER_CONNECTED_NOT_CHARGING;
  }
  else if (bms_is_safe_to_charge())
  {
    // force trigger state
    dsn_prot_set_trigger(false);
    bms_state = BMS_CHARGING;
  }
  else
  {
    bms_state = BMS_FAULT;
  }
}

/** @brief Not charging: manage standby sleep while charger is connected. */
static void bms_handle_charger_connected_not_charging(void)
{
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

      // goto sleep
      system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
      system_sleep(); // WFI

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
        // add some time for vacuum to connect
        sw_timer_delay_ms(250);
      }

      // check if pack needs top-up charging on any wakeup
      if (!bms_is_pack_full())
      {
        bms_state = BMS_CHARGER_CONNECTED;
        return;
      }
    }

    sw_timer_delay_ms(250);
  }
}

/** @brief Charging: manage charge cycle with pause/retry and capacity learning. */
static void bms_handle_charging(void)
{
  uint8_t charging_leds_duty = 0;
#ifdef SERIAL_DEBUG
  uint8_t debug_print_cnt = 0;
#endif

  // First-cycle reset: 20 trigger pushes while charging resets learned capacity
  uint8_t trigger_push_count = 0;
  bool    trigger_was_pressed = false;
  sw_timer trigger_timeout_timer = 0;

  //Sanity check...
  if (!bms_is_safe_to_charge())
  {
    bms_state = BMS_FAULT;
    return;
  }

  // Disable the discharge FET while charging; precharge pin stays asserted
  // by dsn_protocol so the vacuum keeps logic power.
  bq7693_disable_discharge();

  //Enable charging.
  port_pin_set_output_level(ENABLE_CHARGE_PIN, true);
  //Enable the charge FET in the BQ7693.
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

    // Detect trigger pushes for eeprom reset (20 pushes = reset)
    {
      bool trigger_now = dio_read(DIO_TRIGGER_PRESSED);
      if (trigger_now && !trigger_was_pressed)
      {
        // Rising edge detected
        trigger_push_count++;
        sw_timer_start(&trigger_timeout_timer);

        if (trigger_push_count >= 20)
        {
          eeprom_write_defaults();
          trigger_push_count = 0;
          leds_off();
          leds_blink_leds_num(LEDS_LED_ERR_LEFT, 10, 100);
        }
      }
      trigger_was_pressed = trigger_now;

      // If trigger not pressed for >2 seconds, reset counter
      if (trigger_push_count > 0 && sw_timer_is_elapsed(&trigger_timeout_timer, 2000))
      {
        trigger_push_count = 0;
      }
    }

    if (!bms_is_safe_to_charge())
    {
      //Safety error.
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      leds_off();
      bms_state = BMS_FAULT;
      return;
    }

    if ( !dio_read(DIO_CHARGER_CONNECTED))
    {
      //Charger unplugged.
      //Turn off charging
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      // Re-enable discharge FET only if a vacuum is currently connected;
      // otherwise leave it to the idle loop's vacuum-connect edge.
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
      //Pause the charging.
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      //Delay for 30 seconds, then go and try again.
      for (int i=0; i<30; ++i)
      {
        sw_timer_delay_ms(1000);
        wdt_reset_count();
        //If it has, abandon the charge process and return to main loop
        if (!dio_read(DIO_CHARGER_CONNECTED))
        {
          //Charger's been unplugged.
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
      //Restart charging
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
      //After FULL_CHARGE_PAUSE_COUNT pauses, we are full.
      //Disable the charging
      port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
      bq7693_disable_charge();

      leds_off();

      bms_state = BMS_CHARGER_CONNECTED_NOT_CHARGING;

      if (eeprom_data.full_discharge_seen)
      {
        // assign total capacity to currrent charge level if we ar eseen full charge
        eeprom_data.total_pack_capacity = eeprom_data.current_charge_level;
        eeprom_data.full_discharge_seen = 0;
      }
      else
      {
        // filtration of total_pack_capacity, slow decay only (never increase)
        int32_t gap = eeprom_data.total_pack_capacity - eeprom_data.current_charge_level;
        if (gap > 0)
          eeprom_data.total_pack_capacity -= gap >> 3;
      }

      // Clamp to upper bound
      if (eeprom_data.total_pack_capacity > (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH)
        eeprom_data.total_pack_capacity = (int32_t)PACK_CAPACITY_UPPER_BOUND_UAH;

      // we are full
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

/** @brief Charger unplugged: show cell balance via LED blinks. */
static void bms_handle_charger_unplugged(void)
{
  //Do a little flash to show how out of sync the pack is, then go to idle.
  uint16_t *cell_voltages = bq7693_get_cell_voltages();

  uint8_t highest_cell = 0;
  uint8_t lowest_cell = 0;

  for (int i=0; i < 7; ++i)
  {
    if (cell_voltages[i] > cell_voltages[highest_cell])
    {
      highest_cell = i;
    }
    if (cell_voltages[i] < cell_voltages[lowest_cell])
    {
      lowest_cell = i;
    }
  }

  uint16_t spread = cell_voltages[highest_cell] - cell_voltages[lowest_cell];

  //Flash the error led for 100ms for each 50mV the pack is out of balance
  for (int i = 0; i < round(spread/50); ++i)
  {
    leds_blink_leds(100);
  }

#ifdef SERIAL_DEBUG
  BMS_PRINT("Charger unplugged\r\n");
  serial_debug_send_cell_voltages();
#endif

  bms_state = BMS_IDLE;
}

/** @brief RTC callback - sets wakeup flag and generates interrupt that wakes from standby. */
static void rtc_wakeup_callback(void)
{
  rtc_wakeup_flag = true;
}

/** @brief One-time RTC init: configure peripheral and register callback. */
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

/** @brief Start the RTC standby wakeup timer (reset count and enable). */
static void rtc_standby_timer_start(void)
{
  rtc_count_set_count(&rtc_instance, 0);
  rtc_wakeup_flag = false;

  // Clear any stale compare-match flag and NVIC pending bit.
  // rtc_count_disable() does NOT clear the NVIC pending bit, so a leftover
  // pending RTC IRQ would fire the moment system_interrupt_enable() runs
  // inside rtc_count_enable(), setting rtc_wakeup_flag before standby.
  RTC->MODE0.INTFLAG.reg = RTC_MODE0_INTFLAG_MASK;
  NVIC_ClearPendingIRQ(RTC_IRQn);

  rtc_count_enable_callback(&rtc_instance, RTC_COUNT_CALLBACK_COMPARE_0);
  rtc_count_enable(&rtc_instance);
  // wait for ENABLE write to sync across clock domains before entering standby
  while (RTC->MODE0.STATUS.reg & RTC_STATUS_SYNCBUSY);
}

/** @brief Stop and disable the RTC standby wakeup timer. */
static void rtc_standby_timer_stop(void)
{
  rtc_count_disable_callback(&rtc_instance, RTC_COUNT_CALLBACK_COMPARE_0);
  rtc_count_disable(&rtc_instance);
}

/** @brief Switch EIC to low-power oscillator and enable wakeup interrupts for standby. */
static void bms_enter_standby(void)
{
  struct system_gclk_chan_config gclk_chan_conf;

  bms_wdt_deinit();

  /* 1) Stop EIC while changing its clock */
  _extint_disable();
  /* 2) Disable the generic clock channel feeding EIC */
  system_gclk_chan_disable(EIC_GCLK_ID);
  /* 3) Route a new generator to EIC */
  system_gclk_chan_get_config_defaults(&gclk_chan_conf);
  gclk_chan_conf.source_generator = GCLK_GENERATOR_3; // low power 32khz oscillator
  system_gclk_chan_set_config(EIC_GCLK_ID, &gclk_chan_conf);
  /* 4) Enable it again */
  system_gclk_chan_enable(EIC_GCLK_ID);
  /* 5) Start EIC again */
  _extint_enable();

  // enable callbacks, need to wakeup the mcu
  extint_chan_enable_callback(9, EXTINT_CALLBACK_TYPE_DETECT);  // MODE_BUTTON            EXTINT 9 - PA09
  extint_chan_enable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);  // TRIGGER_PRESSED_PIN    EXTINT 4 - PA04
  extint_chan_enable_callback(6, EXTINT_CALLBACK_TYPE_DETECT);  // CHARGER_CONNECTED_PIN  EXTINT 6 - PA06
  // disable alert callback
  extint_chan_disable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);
}

/** @brief Restore EIC to main oscillator and disable wakeup interrupts. */
static void bms_leave_standby(void)
{
  struct system_gclk_chan_config gclk_chan_conf;

  /* 1) Stop EIC while changing its clock */
  _extint_disable();
  /* 2) Disable the generic clock channel feeding EIC */
  system_gclk_chan_disable(EIC_GCLK_ID);
  /* 3) Route a new generator to EIC */
  system_gclk_chan_get_config_defaults(&gclk_chan_conf);
  gclk_chan_conf.source_generator = GCLK_GENERATOR_0;
  system_gclk_chan_set_config(EIC_GCLK_ID, &gclk_chan_conf);
  /* 4) Enable it again */
  system_gclk_chan_enable(EIC_GCLK_ID);
  /* 5) Start EIC again */
  _extint_enable();

  // return from sleep, disable external interrupts
  extint_chan_disable_callback(9, EXTINT_CALLBACK_TYPE_DETECT);  // MODE_BUTTON            EXTINT 9 - PA09
  extint_chan_disable_callback(4, EXTINT_CALLBACK_TYPE_DETECT);  // TRIGGER_PRESSED_PIN    EXTINT 4 - PA04
  extint_chan_disable_callback(6, EXTINT_CALLBACK_TYPE_DETECT);  // CHARGER_CONNECTED_PIN  EXTINT 6 - PA06
  // enable alert callback
  extint_chan_enable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);

  bms_wdt_init();
}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
