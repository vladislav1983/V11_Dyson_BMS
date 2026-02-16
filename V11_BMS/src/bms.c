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
#include "protocol.h"
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

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/
#ifdef SERIAL_DEBUG
#define BMS_PRINT(...) \
{ \
  sprintf(debug_msg_buffer, __VA_ARGS__); \
  serial_debug_send_message(debug_msg_buffer);  \
}
#else
#define BMS_PRINT(...)
#endif

#define ROUND(x) (((x) + 0.5))

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
  "TRIGGER_PULLED",
  "DISCHARGING",
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
static void    bms_handle_trigger_pulled(void);
static void    bms_handle_sleep(void);
static void    bms_handle_discharging(void);
static void    bms_handle_fault(void);
static void    bms_handle_charger_connected(void);
static void    bms_handle_charger_connected_not_charging(void);
static void    bms_handle_charging(void);
static void    bms_handle_charger_unplugged(void);
static void    bms_enter_standby(void);
static void    bms_leave_standby(void);

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief
//- **************************************************************************
void bms_init(void)
{
  //sets up clocks/IRQ handlers etc.
  system_init();
  //Initialise the sw_timer
  delay_init();
  sw_timer_init();
  prot_init();

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

#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
  serial_debug_init();
#endif
  bms_wdt_init();
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void bms_wakeup_interrupt_callback(void)
{

}

//- **************************************************************************
//! \brief 
//- **************************************************************************
void bms_interrupt_callback(void)
{
  process_bms_interrupt = true;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
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
        
        //We thought the pack was full, but it's still charging, so we need to update its' size.
        if (eeprom_data.current_charge_level > eeprom_data.total_pack_capacity)
        {
          eeprom_data.current_charge_level = eeprom_data.total_pack_capacity;
        }
        
        //We thought the pack was empty, but it isn't, so again, we need to update our estimate of what it can hold!
        if (eeprom_data.current_charge_level < 0)
        {
          //subtracting negative numbers will increment the pack capacity.
          eeprom_data.total_pack_capacity -= eeprom_data.current_charge_level;
          eeprom_data.current_charge_level = 0;
        }
      }
      //Update the CC bit so it'll refire in another 250mS as per datasheet.
      bq7693_write_register(SYS_STAT, 0x80);//Clear CC bit.
    }

    process_bms_interrupt = false;
  }
}

//- **************************************************************************
//! \brief return state of charge to vacuum in % * 100, required by protocol
//         do not return 0, to prevent vacuum critical battery level screen
//- **************************************************************************
uint16_t bms_get_soc_x100(void)
{
  uint16_t soc = 100;

  if(eeprom_data.total_pack_capacity > 0 && eeprom_data.current_charge_level > 0)
  {
    uint32_t current_charge_level = eeprom_data.current_charge_level;
    uint16_t total_pack_capacity  = eeprom_data.total_pack_capacity  >> 10;
    
    soc = (current_charge_level * (uint16_t)ROUND((100.0f * 100.0f) / 1024.0f)) / total_pack_capacity;
    soc = (soc > 10000) ? 10000 : ((soc == 0) ? 100 : soc);
  }

  return soc;
}

//- **************************************************************************
//! \brief get runtime in seconds, required by vacuum
//         Prevent excessively large runtime display during pack capacity learning
//         Always limit to the maximum pack capacity
//         Do not display less than minute to avoid annoying vacuum messages 
//- **************************************************************************
uint32_t bms_get_runtime_seconds(void)
{
  uint32_t current_filt_mA_abs = abs(current_filt_mA);
  int32_t  current_charge_level;
  int32_t  runtime = 0;
  uint16_t current;

  if(    bms_state == BMS_DISCHARGING // fill the runtime only when vacuum is working
      && current_filt_mA_abs > 1000)  // and current is > 1A, to prevent too big numbers
  {
    current_charge_level = eeprom_data.current_charge_level > (PACK_MAX_CAPACITY_MAH * 1000) ? (PACK_MAX_CAPACITY_MAH * 1000) : eeprom_data.current_charge_level; // limit pack capacity
    current              = (current_filt_mA == 0) ? 1 : abs(current_filt_mA);  // check current is not zero

    runtime = ((current_charge_level / current) * (uint16_t)((3600.0f / 1000.0f) * 1024.0f)) >> 10;  // compute and scale down
   
    // always limit runtime to 1 minute 
    runtime = runtime < 60 ? 60 : runtime;
  }

  return (uint32_t)runtime;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void bms_mainloop(void)
{
  //Handle the state machinery.
  while (1)
  {
    BMS_PRINT("BMS: %s\r\n", bms_state_names[bms_state]);

    switch (bms_state)
    {
    //-----------------------------------------------------------------------
      case BMS_INIT:
        bms_state = BMS_IDLE;
        port_pin_set_output_level(PRECHARGE_PIN, true);

#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
        //Initial debug blurb
        serial_debug_send_message("Dyson V11/V15 BMS After market firmware\r\n");
#endif

        leds_sequence();

#if defined(SERIAL_DEBUG) || defined(PROT_DEBUG_PRINT)
        //Initial debug blurb
        serial_debug_send_cell_voltages();
        serial_debug_send_pack_capacity();
#endif
        
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
      case BMS_TRIGGER_PULLED:
        bms_handle_trigger_pulled();
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
      case BMS_DISCHARGING:
        bms_handle_discharging();
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
    prot_mainloop();
    bms_interrupt_process();
    bms_wdt_mainloop();
  }
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief 
//- **************************************************************************
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

//- **************************************************************************
//! \brief
//- **************************************************************************
static void pins_deinit(void)
{
  port_pin_set_output_level(PIN_PA03, false);
  port_pin_set_output_level(MODE_BUTTON_PULLUP_ENABLE_PIN, false);
  port_pin_set_output_level(PRECHARGE_PIN, false);
}

//- **************************************************************************
//! \brief
//- **************************************************************************
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

//- **************************************************************************
//! \brief
//- **************************************************************************
static int16_t bms_read_temperature(void)
{
  int16_t tc1_temp;
  int16_t tc2_temp;
  uint16_t adc_value;
  int16_t temp_diff;;

  // get tc1
  adc_value = adc_convert_channel(BMS_ADC_CH_TC1);
  tc1_temp  = NTC_ADC2Temperature(adc_value);
  // get tc2
  adc_value = adc_convert_channel(BMS_ADC_CH_TC2);
  tc2_temp  = NTC_ADC2Temperature(adc_value);

  // return temperature point between two thermistors if they have plausible values - no more than 5 degree difference
  temp_diff = tc1_temp - tc2_temp;

  if(abs(temp_diff) < 50)
  {
    return (tc1_temp + tc2_temp) >> 1;
  }
  else
  {
    BMS_PRINT("BMS:TEMP ERROR: %d\r\n", temp_diff);
    return 256 * 10;
  }
}

//- **************************************************************************
//! \brief
//- **************************************************************************
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
      bms_error = BMS_ERR_PACK_DISCHARGED;
      
#ifdef SERIAL_DEBUG
      BMS_PRINT("%s: Cell voltages too low\r\n", __FUNCTION__);
      
      for (int cell=0; cell<7; ++cell)
      {
        BMS_PRINT("Cell %d: %d mV, min %d mV\r\n", cell, cell_voltages[cell], CELL_LOWEST_DISCHARGE_VOLTAGE);
      }
#endif
    }
  }
  //Check pack temperature remains in acceptable range
  pack_temperature = bms_read_temperature();
  int temp = pack_temperature / 10;

  if (temp  > MAX_PACK_TEMPERATURE)
  {
    bms_error = BMS_ERR_PACK_OVERTEMP;
    BMS_PRINT("%s : Pack overtemp %d 'C, max %d\r\n",__FUNCTION__ ,  temp, MAX_PACK_TEMPERATURE);
  }
  else if (temp < MIN_PACK_DISCHARGE_TEMP)
  {
    bms_error = BMS_ERR_PACK_UNDERTEMP;
    BMS_PRINT("%s: Pack undertemp %d 'C, min %d\r\n", __FUNCTION__ , temp, MIN_PACK_DISCHARGE_TEMP);
  }
  
  //Check sys_stat
  uint8_t sys_stat;
  bq7693_read_register(SYS_STAT, 1, &sys_stat);

  if (sys_stat & 0x01)
  {
    bms_error = BMS_ERR_OVERCURRENT;
    bq7693_write_register(SYS_STAT, 0x01);

    BMS_PRINT("%s: BMS IC Overcurrent Trip\r\n", __FUNCTION__);
  }
  else if (sys_stat & 0x02)
  {
    bms_error = BMS_ERR_SHORTCIRCUIT;
    bq7693_write_register(SYS_STAT, 0x02);

    BMS_PRINT("%s: BMS IC Short Circuit Trip\r\n", __FUNCTION__);
  }
  else if (sys_stat & 0x08)
  {
    bms_error = BMS_ERR_UNDERVOLTAGE;
    bq7693_write_register(SYS_STAT, 0x08);

    BMS_PRINT("%s: BMS IC Undervoltage Trip\r\n", __FUNCTION__);
  }

  if (bms_error == BMS_ERR_NONE)
    return true;
  else
    return false;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
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
      bms_error = BMS_ERR_CELL_FAIL;
      BMS_PRINT("%s: Cell %d below min charge voltage %d, min %d\r\n", __FUNCTION__, i, cell_voltages[i], CELL_LOWEST_CHARGE_VOLTAGE);
    }
  }

  //Check pack temperature acceptable (<=60'C)
  pack_temperature = bms_read_temperature();
  int temp = pack_temperature / 10;

  if (temp  > MAX_PACK_TEMPERATURE)
  {
    bms_error = BMS_ERR_PACK_OVERTEMP;
  }
  else if (temp < MIN_PACK_CHARGE_TEMP)
  {
    bms_error = BMS_ERR_PACK_UNDERTEMP;
  }
  
  //Check sys_stat
  uint8_t sys_stat;
  bq7693_read_register(SYS_STAT, 1, &sys_stat);

  if (sys_stat & 0x01)
  {
    bms_error = BMS_ERR_OVERCURRENT;
    bq7693_write_register(SYS_STAT, 0x01);
  }
  else if (sys_stat & 0x04)
  {
    bms_error = BMS_ERR_OVERVOLTAGE;
    bq7693_write_register(SYS_STAT, 0x04);
  }
  
  if (bms_error == BMS_ERR_NONE)
    return true;
  else
    return false;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static bool bms_is_pack_full(void)
{
  uint16_t *cell_voltages = bq7693_get_cell_voltages();

  //If any cells are at their full charge voltage, we are full.
  for (int i=0; i<7; ++i)
  {
    if (cell_voltages[i] >= CELL_FULL_CHARGE_VOLTAGE )
    {
      return true;
    }
  }

  return false;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static void bms_handle_idle(void)
{
  uint32_t sleep_time;
  sw_timer_start(&bms_timer);

  do 
  {
    if(true == prot_get_vacuum_connected())
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
    else if (dio_read(DIO_TRIGGER_PRESSED) == true)
    {
      leds_blink_leds(10);
      bms_state = BMS_TRIGGER_PULLED;
      return;
    }
    else if(force_sleep == true)
    {
      sw_timer_stop(&bms_timer); // go to sleep
    }
    else if(prot_get_sleep_flag() == true)
    {
      sw_timer_stop(&bms_timer); // go to sleep requested by cleaner
    }

    sw_timer_delay_ms(50);

  } while (false == sw_timer_is_elapsed(&bms_timer, sleep_time));
  	
  //Reached the end of our wait loop, with nobody pulling the trigger, or plugging in charger.
  //Transit to sleep state
  bms_state = BMS_SLEEP;
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static void bms_handle_trigger_pulled(void)
{
	//Check if it's safe to discharge or not.
	if (bms_is_safe_to_discharge()) 
  {
		//All go - unleash the power!
		bms_state = BMS_DISCHARGING;
	}
	else 
  {
		bms_state = BMS_FAULT;
	}
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static void bms_handle_sleep(void)
{
  bms_wdt_deinit();
  serial_debug_send_message("BMS_SLEEP\r\n");
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

//- **************************************************************************
//! \brief
//- **************************************************************************
static void bms_handle_discharging(void)
{
#ifdef SERIAL_DEBUG
  uint8_t debug_print_cnt = 0;
#endif

	if (bms_is_safe_to_discharge()) 
  {
		//Sanity check, hopefully already checked prior to here!
		bq7693_enable_discharge();
    sw_timer_delay_ms(300);
    prot_set_trigger(true);
	}

  while (1)
  {
    if (!dio_read(DIO_TRIGGER_PRESSED)) 
    {
      prot_set_trigger(false);
      sw_timer_delay_ms(300);
      //Trigger released.
      bq7693_disable_discharge();
      //Clear the battery status etc.
      leds_off();
      bms_state = BMS_IDLE;
      return;
    }

    if (!bms_is_safe_to_discharge()) 
    {
      //A fault has occurred.
      bq7693_disable_discharge();
      bms_state = BMS_FAULT;
      return;
    }

#ifdef SERIAL_DEBUG
    if(++debug_print_cnt > 5)
    {
      BMS_PRINT("BMS:DISCHARGING I:%d mA @ %ld mAH, C:%ld mAH, T:%d'C\r\n", abs(current_filt_mA), (eeprom_data.current_charge_level / 1000), (eeprom_data.total_pack_capacity / 1000), (int16_t)(pack_temperature / 10));
      debug_print_cnt = 0;
    }
#endif

    sw_timer_delay_ms(60);
  }
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static void bms_handle_fault(void)
{
  //Turn all the LEDs off.
	leds_off();
	prot_set_trigger(false);
  bq7693_disable_discharge();
  port_pin_set_output_level(ENABLE_CHARGE_PIN, false);

	//Show the error status and continue to show it, until trigger released and charger unplugged.
	do 
  {
		if (bms_error == BMS_ERR_PACK_DISCHARGED || bms_error == BMS_ERR_UNDERVOLTAGE ) 
    {
			//If the problem is just a flat pack, blink
			leds_blink_leds(50);

			//We also need to update the pack capacity as it's flat at this point.
			if (eeprom_data.current_charge_level > 0 && eeprom_data.total_pack_capacity > eeprom_data.current_charge_level) 
      {
				eeprom_data.total_pack_capacity -= eeprom_data.current_charge_level;
				eeprom_data.current_charge_level = 0;				
			}
		}
		else 
    {
			//Flash the red error led the number of times indicated by the fault code.
			for (int i=0; i < bms_error; ++i)
      {
				leds_blink_leds(500);
			}

			sw_timer_delay_ms(2000);
		}
	} 
	while (dio_read(DIO_TRIGGER_PRESSED) || dio_read(DIO_CHARGER_CONNECTED));
		
	//Return to idle
	bms_state = BMS_IDLE;	
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static void bms_handle_charger_connected(void)
{
  if (bms_is_pack_full())
  {
    //If the pack is full, transit to idle.
    bms_state = BMS_IDLE;
  }
  else if (bms_is_safe_to_charge())
  {
    // force trigger state
    prot_set_trigger(false);
    bms_state = BMS_CHARGING;
  }
  else
  {
    bms_state = BMS_FAULT;
  }
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static void bms_handle_charger_connected_not_charging(void)
{
  while(1)
  {
    if(prot_get_sleep_flag() == true)
    {
      bms_enter_standby();
      serial_debug_send_message("BMS_STANDBY\r\n");
      leds_blink_leds_num(LEDS_NUM, 4, 100);

      // goto sleep
      system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
      system_sleep(); // WFI

      bms_leave_standby();

      // reset protocol state machine, wait for handshake 
      prot_reset();
      leds_blink_leds_num(LEDS_NUM, 2, 100);
    }
    else if (!dio_read(DIO_CHARGER_CONNECTED))
    {
      bms_state = BMS_IDLE;
      return;
    }
    sw_timer_delay_ms(100);
  }
}

//- **************************************************************************
//! \brief
//- **************************************************************************
static void bms_handle_charging(void)
{
  uint8_t charging_leds_duty = 0;
#ifdef SERIAL_DEBUG
  uint8_t debug_print_cnt = 0;
#endif

  //Sanity check...
  if (!bms_is_safe_to_charge()) 
  {
    bms_state = BMS_FAULT;
    return;
  }
  
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
        //If it has, abandon the charge process and return to main loop
        if (!dio_read(DIO_CHARGER_CONNECTED)) 
        {
          //Charger's been unplugged.
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
        BMS_PRINT("BMS:CHARGING I:%d mA @ %ld mAH, C:%ld mAH, T:%d'C\r\n", abs(current_filt_mA), (eeprom_data.current_charge_level / 1000), (eeprom_data.total_pack_capacity / 1000), (int16_t)(pack_temperature / 10));
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

      //Set charge level to equal capacity.
      eeprom_data.total_pack_capacity = eeprom_data.current_charge_level;

      BMS_PRINT("BMS:CHARGING Stopped\r\n");
#ifdef SERIAL_DEBUG
      serial_debug_send_pack_capacity();
#endif
      return;
    }

    sw_timer_delay_ms(50);
  }
}

//- **************************************************************************
//! \brief
//- **************************************************************************
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

//- **************************************************************************
//! \brief
//- **************************************************************************
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

//- **************************************************************************
//! \brief
//- **************************************************************************
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
