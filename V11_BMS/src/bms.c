/*
 * bms.c
 *
 *  Author:  David Pye
 *  Contact: davidmpye@gmail.com
 *  Licence: GNU GPL v3 or later
 */ 

#include "bms.h"
#include "bms_adc.h"
#include "ntc.h"
#include "crc32.h"
#include "sw_timer.h"
#include "protocol.h"

//We start off idle.
enum BMS_STATE bms_state = BMS_IDLE;

//If a fault occurs, it'll be lodged here.
enum BMS_ERROR_CODE bms_error = BMS_ERR_NONE;

struct adc_module adc_instance;

#ifdef SERIAL_DEBUG
const char *bms_state_names[] = 
{
	"BMS_IDLE",
	"BMS_CHARGER_CONNECTED",
	"BMS_CHARGING",
	"BMS_CHARGER_CONNECTED_NOT_CHARGING",
	"BMS_CHARGER_UNPLUGGED",
	"BMS_TRIGGER_PULLED",
	"BMS_DISCHARGING",
	"BMS_FAULT",
	"BMS_SLEEP"
};
#endif

void pins_init() 
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

  // setup serial communication pullup enable pin
  struct port_config io_port_config;
  port_get_config_defaults(&io_port_config);
  io_port_config.direction = PORT_PIN_DIR_OUTPUT;
  port_pin_set_config(PIN_PA25, &io_port_config);
  port_pin_set_output_level(PIN_PA25, true );

  // setup mode button pullup voltage
  port_pin_set_config(PIN_PA18, &io_port_config);
  port_pin_set_output_level(PIN_PA18, true );
}

volatile int32_t currentmA;
	
void bms_interrupt_callback(void) {
	uint8_t sys_stat;
	bq7693_read_register(SYS_STAT, 1, &sys_stat);
	if (sys_stat & 0x80) {
		//Got a coulomb charger count ready.
		int32_t ccVal = bq7693_read_cc();
		
		//This needs better handling....
		currentmA = ccVal*8.44f;
			
		//Ignore tiny values.
		if ( (ccVal > 0 && ccVal > 2)  || (ccVal < 0 && ccVal < -2) )  {
			ccVal *= 8.44f; //8.44microVolts per LSB.
			//i = V/R
			//sense resistor = 1mOhm
			//microV / milliOhms gives current in mA.   
			//so ccVal has current in mA.
			//Dividing by 14400 would give mAH. (number of 250mS periods in 1 hr.
			//Dividing by 14.4 will give microAH (what we want)
			ccVal /= 14.4f;
		
			eeprom_data.current_charge_level += ccVal;
						
			//We thought the pack was full, but it's still charging, so we need to update its' size.		
			if (eeprom_data.current_charge_level > eeprom_data.total_pack_capacity) {
				eeprom_data.total_pack_capacity = eeprom_data.current_charge_level;
			}
		
			//We thought the pack was empty, but it isn't, so again, we need to update our estimate of what it can hold!
			if (eeprom_data.current_charge_level < 0) {
				//subtracting negative numbers will increment the pack capacity.
				eeprom_data.total_pack_capacity -= eeprom_data.current_charge_level;
				eeprom_data.current_charge_level = 0;
			}
		}
		//Update the CC bit so it'll refire in another 250mS as per datasheet.
		bq7693_write_register(SYS_STAT, 0x80);//Clear CC bit.
	}	
}
	
void interrupts_init(void) {
	//A single interrupt, focused on the BQ7693's alert line (PA28), which
	//is on EXTINT 8.
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);	
	config_extint_chan.gpio_pin        = 	PIN_PA28A_EIC_EXTINT8;
	config_extint_chan.gpio_pin_mux =       MUX_PA28A_EIC_EXTINT8;
	//This line is designed to be possible for either device to pull it up or down to indicate a fault condition, so
	//no pullups.
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_NONE;
	config_extint_chan.detection_criteria = EXTINT_DETECT_RISING;
	
	extint_chan_set_config(8, &config_extint_chan);
	extint_register_callback(bms_interrupt_callback, 8, EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(8, EXTINT_CALLBACK_TYPE_DETECT);
	//Enable interrupts.	
	system_interrupt_enable_global();
}

int16_t bms_read_temperature(void)
{
  int16_t tc1_temp;
  int16_t tc2_temp;
  uint16_t adc_value;

  adc_convert_channels();

  // get tc1
  adc_value = bms_adc_read_ch(BMS_ADC_CH_TC1);
  tc1_temp = NTC_ADC2Temperature(adc_value);
  // get tc2
  adc_value = bms_adc_read_ch(BMS_ADC_CH_TC2);
  tc2_temp = NTC_ADC2Temperature(adc_value);

  // return temperature point between two thermistors if they have plausible values - no more than 5 degree difference
  if(abs(tc1_temp - tc2_temp) < 50)
  {
    return (tc1_temp + tc2_temp) >> 1;
  }
  else
  {
    return 256;
  }
}

void bms_init() 
{
	//sets up clocks/IRQ handlers etc.
	system_init();
	//Initialise the sw_timer
	sw_timer_init();
	//Set up the pins
	pins_init();
	
  bms_adc_init();

	//BQ7693 init
	bq7693_init();
	
#ifdef SERIAL_DEBUG
	serial_debug_init();
#endif
	
	//Init the LEDs
	leds_init();
	//Init eeprom emulator
	eeprom_init();
	eeprom_read();
	
	//Initialise the USART we need to talk to the vacuum cleaner
	serial_init();	
	
  prot_init();

	//Enable interrupts
	interrupts_init();
}
	
bool bms_is_safe_to_discharge() {
	//Clear error status.
	bms_error = BMS_ERR_NONE;
	
	uint16_t *cell_voltages = bq7693_get_cell_voltages();
	//Check any cells undervolt.
	for (int i=0; i<7;++i) {
		if (cell_voltages[i] < CELL_LOWEST_DISCHARGE_VOLTAGE) {
			bms_error = BMS_ERR_PACK_DISCHARGED;
			
#ifdef SERIAL_DEBUG
			sprintf(debug_msg_buffer, "%s: Cell voltages too low\r\n", __FUNCTION__);
			serial_debug_send_message(debug_msg_buffer);
				
			for (int cell=0; cell<7; ++cell) 
      {
				sprintf(debug_msg_buffer, "Cell %d: %d mV, min %d mV\r\n", cell, cell_voltages[cell], CELL_LOWEST_DISCHARGE_VOLTAGE);
				serial_debug_send_message(debug_msg_buffer);
			}
#endif		
		}
	}
	//Check pack temperature remains in acceptable range	
	int temp = bms_read_temperature();
	if (temp/10  > MAX_PACK_TEMPERATURE) {
		bms_error = BMS_ERR_PACK_OVERTEMP;
		
#ifdef SERIAL_DEBUG
		sprintf(debug_msg_buffer, "%s : Pack overtemp %d 'C, max %d\r\n",__FUNCTION__ ,  temp/10, MAX_PACK_TEMPERATURE);
		serial_debug_send_message(debug_msg_buffer);
#endif

	}
	else if (temp/10 < MIN_PACK_DISCHARGE_TEMP) {
		bms_error = BMS_ERR_PACK_UNDERTEMP;

#ifdef SERIAL_DEBUG
		sprintf(debug_msg_buffer, "%s: Pack undertemp %d 'C, min %d\r\n", __FUNCTION__ , temp/10, MIN_PACK_DISCHARGE_TEMP);
		serial_debug_send_message(debug_msg_buffer);
#endif
	}
	
	//Check sys_stat	
	uint8_t sys_stat;
	bq7693_read_register(SYS_STAT, 1, &sys_stat);

	if (sys_stat & 0x01) 	{
		bms_error = BMS_ERR_OVERCURRENT;
		bq7693_write_register(SYS_STAT, 0x01);

#ifdef SERIAL_DEBUG
		sprintf(debug_msg_buffer, "%s: BMS IC Overcurrent Trip\r\n", __FUNCTION__);
		serial_debug_send_message(debug_msg_buffer);
#endif

	}
	else if (sys_stat & 0x02) {
		bms_error = BMS_ERR_SHORTCIRCUIT;
		bq7693_write_register(SYS_STAT, 0x02);

#ifdef SERIAL_DEBUG
		sprintf(debug_msg_buffer, "%s: BMS IC Short Circuit Trip\r\n", __FUNCTION__);
		serial_debug_send_message(debug_msg_buffer);
#endif	

	}
	else if (sys_stat & 0x08) {
		bms_error = BMS_ERR_UNDERVOLTAGE;
		bq7693_write_register(SYS_STAT, 0x08);

#ifdef SERIAL_DEBUG
	sprintf(debug_msg_buffer, "%s: BMS IC Undervoltage Trip\r\n", __FUNCTION__);
	serial_debug_send_message(debug_msg_buffer);
#endif

	}	

	if (bms_error == BMS_ERR_NONE) {
		return true;
	}
	else return false;
	
}

bool bms_is_safe_to_charge() {
	//Clear error status.
	bms_error = BMS_ERR_NONE;
	
	uint16_t *cell_voltages = bq7693_get_cell_voltages();
	
	//Check no cells are so flat they cannot be charged.
	for (int i=0; i<7;++i) {
		if ( cell_voltages[i] < CELL_LOWEST_CHARGE_VOLTAGE ) {
			bms_error = BMS_ERR_CELL_FAIL;	

#ifdef SERIAL_DEBUG
		sprintf(debug_msg_buffer, "%s: Cell %d below min charge voltage %d, min %d\r\n", __FUNCTION__, i, cell_voltages[i], CELL_LOWEST_CHARGE_VOLTAGE);
		serial_debug_send_message(debug_msg_buffer);
#endif

		}
	}

	//Check pack temperature acceptable (<=60'C)	
	int temp = bms_read_temperature();
	if (temp/10  > MAX_PACK_TEMPERATURE) {
		bms_error = BMS_ERR_PACK_OVERTEMP;
	}
	else if (temp/10 < MIN_PACK_CHARGE_TEMP) {
		bms_error = BMS_ERR_PACK_UNDERTEMP;
	}
	
	//Check sys_stat
	uint8_t sys_stat;
	bq7693_read_register(SYS_STAT, 1, &sys_stat);
	if (sys_stat & 0x01) 	{
		bms_error = BMS_ERR_OVERCURRENT;
		bq7693_write_register(SYS_STAT, 0x01);
	}
	else if (sys_stat & 0x04) {
		bms_error = BMS_ERR_OVERVOLTAGE;
		bq7693_write_register(SYS_STAT, 0x04);
	}
	
	if (bms_error == BMS_ERR_NONE) {
		return true;
	}
	else return false;
}

bool bms_is_pack_full() {
	uint16_t *cell_voltages = bq7693_get_cell_voltages();

#ifdef SERIAL_DEBUG
	for (int i=0; i<7; ++i) {
		char message[40];
		sprintf(message, "Cell %d: %d mV, target %d mV\r\n", i, cell_voltages[i], CELL_FULL_CHARGE_VOLTAGE);
	}
#endif

	//If any cells are at their full charge voltage, we are full.
	for (int i=0; i<7;++i) {
		if (cell_voltages[i] >= CELL_FULL_CHARGE_VOLTAGE ) {
			return true;
		}
	}
	return false;
}


void bms_handle_idle() 
{
	//Three potential ways out of this state - someone pulls the trigger, plugs in a charger, or the IDLE_TIME is exceeded and we go to sleep.
	for (int i=0; i<  (IDLE_TIME * 1000) / 50; ++i) 
  {
		if (port_pin_get_input_level(CHARGER_CONNECTED_PIN) == true) 
    {
			bms_state = BMS_CHARGER_CONNECTED;
			return;
		}
		else if (port_pin_get_input_level(TRIGGER_PRESSED_PIN) == true) 
    {
			bms_state = BMS_TRIGGER_PULLED;
			return;
		}
		sw_timer_delay_ms(50);
	}	
	//Reached the end of our wait loop, with nobody pulling the trigger, or plugging in charger.
	//Transit to sleep state
	bms_state = BMS_SLEEP;
}

void bms_handle_trigger_pulled() 
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

void bms_handle_sleep() 
{
	//Store pack charge data to eeprom
	eeprom_write();
	
	bq7693_enter_sleep_mode();
	
	//We are about to get powered down.
	while(1);
}

static sw_timer bms_timer = 0;

#ifdef SERIAL_DEBUG
static sw_timer bms_debug_print_timer = 0;
#endif

void bms_handle_discharging(void) 
{		
#ifdef SERIAL_DEBUG
	serial_debug_send_message("Starting discharge\r\n");
#endif
	
	if (bms_is_safe_to_discharge())
   {
		//Sanity check, hopefully already checked prior to here!
		bq7693_enable_discharge();
	}
	
	while (1) 
  {
#ifdef SERIAL_DEBUG
    if(false != sw_timer_is_elapsed(&bms_debug_print_timer, 250))
    {
      sw_timer_start(&bms_debug_print_timer);

      sprintf(debug_msg_buffer,"Discharging at %ld mA, %ld mAH, capacity %ld mAH, Temp %d'C\r\n", currentmA*-1, eeprom_data.current_charge_level/1000, eeprom_data.total_pack_capacity/1000,
      bms_read_temperature()/10);
      serial_debug_send_message(debug_msg_buffer);
    }
#endif

    if(false != sw_timer_is_elapsed(&bms_timer, 50))
    {
      sw_timer_start(&bms_timer);

		  if (!port_pin_get_input_level(TRIGGER_PRESSED_PIN)) 
      {
			  //Trigger released.
			  bq7693_disable_discharge();
			  //Clear the battery status etc.
			  leds_off();
			  bms_state = BMS_IDLE;
			  return;
		  }
		
      if(!bms_is_safe_to_discharge()) 
      {
			  //A fault has occurred.
			  bq7693_disable_discharge();
			  bms_state = BMS_FAULT;
			  return;
		  }
    }

    prot_mainloop();
	}
}

void bms_handle_fault() {
	//Turn all the LEDs off.
	leds_off();
	
	//Show the error status and continue to show it, until trigger released and charger unplugged.
	do {
		if (bms_error == BMS_ERR_PACK_DISCHARGED || bms_error == BMS_ERR_UNDERVOLTAGE ) {
			//If the problem is just a flat pack, blink the lowest battery segment three times.
			leds_show_pack_flat();

			//We also need to update the pack capacity as it's flat at this point.
			if (eeprom_data.current_charge_level > 0) {
				eeprom_data.total_pack_capacity -= eeprom_data.current_charge_level;
				eeprom_data.current_charge_level = 0;				
			}
		}
		else {
			//Flash the red error led the number of times indicated by the fault code.
			for (int i=0; i<bms_error; ++i) {
				leds_blink_error_led(500);
			}
			sw_timer_delay_ms(2000);
		}
	} 
	while (port_pin_get_input_level(TRIGGER_PRESSED_PIN) || port_pin_get_input_level(CHARGER_CONNECTED_PIN));
		
	//Return to idle
	bms_state = BMS_IDLE;	
}

void bms_handle_charger_connected() {
	if (bms_is_pack_full()) {
		//If the pack is full, transit to idle.
		bms_state = BMS_IDLE;
	}
	else if (bms_is_safe_to_charge()) {
		bms_state = BMS_CHARGING;
	}
	else {
		bms_state = BMS_FAULT;
	}
}

void bms_handle_charger_connected_not_charging() {
	//Wait up to 30 seconds to see if someone unplugs the charger.
	//If so, to idle.
	//If not, to sleep.
	for (int i=0; i<30; ++i) {
		if (!port_pin_get_input_level(CHARGER_CONNECTED_PIN)) {
			bms_state = BMS_IDLE;
			return;
		}		
		sw_timer_delay_ms(1000);
	}
	//Sleep then!
	bms_state = BMS_SLEEP;
}

void bms_handle_charging() {
	//Sanity check...
	if (!bms_is_safe_to_charge()) {
		bms_state = BMS_FAULT;
		return;
	}
	//Enable charging.
	port_pin_set_output_level(ENABLE_CHARGE_PIN, true);
	 //Enable the charge FET in the BQ7693.
	bq7693_enable_charge();
	
	int charge_pause_counter = 0;
	while (1) {
		//Charging now in progress.		
		//Show flashing LED segment to indicate we are charging.
		// TODO: leds_flash_charging_segment((eeprom_data.current_charge_level*100) / eeprom_data.total_pack_capacity);
	
#ifdef SERIAL_DEBUG
		sprintf(debug_msg_buffer,"Charging at %ld mA, %ld mAH, capacity %ld mAH, Temp %d'C\r\n", currentmA, eeprom_data.current_charge_level/1000, eeprom_data.total_pack_capacity/1000, 
		bms_read_temperature()/10);
		serial_debug_send_message(debug_msg_buffer);	
#endif
		if (!bms_is_safe_to_charge()) 
    {
			//Safety error.
			port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
			bq7693_disable_charge();

			leds_off();
			bms_state = BMS_FAULT;
			return;
		}
				
		if ( !port_pin_get_input_level(CHARGER_CONNECTED_PIN))
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
#ifdef SERIAL_DEBUG
			sprintf(debug_msg_buffer, "Charging paused - cell full, attempt %d of %d\r\n", charge_pause_counter, FULL_CHARGE_PAUSE_COUNT);
			serial_debug_send_message(debug_msg_buffer);			
			serial_debug_send_cell_voltages();
#endif
			//Pause the charging.
			port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
			bq7693_disable_charge();
		
			//Delay for 30 seconds, then go and try again.	
			for (int i=0; i<30; ++i) 
      {
				//Check the charger hasn't been unplugged while we're waiting
				//If it has, abandon the charge process and return to main loop
				if (!port_pin_get_input_level(CHARGER_CONNECTED_PIN)) {
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
		
		if (charge_pause_counter == FULL_CHARGE_PAUSE_COUNT) {
			//After FULL_CHARGE_PAUSE_COUNT pauses, we are full.
			//Disable the charging
			port_pin_set_output_level(ENABLE_CHARGE_PIN, false);
			bq7693_disable_charge();
			
			leds_off();

			bms_state = BMS_CHARGER_CONNECTED_NOT_CHARGING;

			//Set charge level to equal capacity.
			eeprom_data.total_pack_capacity = eeprom_data.current_charge_level;

#ifdef SERIAL_DEBUG
			serial_debug_send_message("Charging stopped - cells at capacity\r\n");
			char message[40];
			sprintf(message, "Total pack capacity %ldmAh\r\n", eeprom_data.total_pack_capacity/1000);
			serial_debug_send_message(message);
#endif
			return;	
		}			
	}
}

void bms_handle_charger_unplugged() {
	//Do a little flash to show how out of sync the pack is, then go to idle.
	uint16_t *cell_voltages = bq7693_get_cell_voltages();
		
	uint8_t highest_cell = 0;
	uint8_t lowest_cell = 0;
		
	for (int i=0; i<7;++i) {
		if (cell_voltages[i] > cell_voltages[highest_cell]) {
			highest_cell = i;
		}
		if (cell_voltages[i] < cell_voltages[lowest_cell]) {
			lowest_cell = i;
		}
	}
	
	uint16_t spread = cell_voltages[highest_cell] - cell_voltages[lowest_cell];
	
	//Flash the error led for 100ms for each 50mV the pack is out of balance
	for (int i=0; i<round(spread/50); ++i) {
		leds_blink_error_led(100);	
	}

#ifdef SERIAL_DEBUG
	serial_debug_send_message("Charger unplugged\r\n");
	serial_debug_send_cell_voltages();
#endif

	bms_state = BMS_IDLE;
}

volatile uint32_t crc32;
void bms_mainloop() 
{
	//Handle the state machinery.
	while (1) 
  {
#ifdef SERIAL_DEBUG
		sprintf(debug_msg_buffer, "%s: Entering state %s\r\n", __FUNCTION__, bms_state_names[bms_state]);
		serial_debug_send_message(debug_msg_buffer);
#endif
		switch (bms_state) 
    {
			case BMS_IDLE:
				bms_handle_idle();
				break;
			case BMS_SLEEP:
				bms_handle_sleep();
				break;	
			case BMS_TRIGGER_PULLED:
				bms_handle_trigger_pulled();
				break;
			case BMS_CHARGER_CONNECTED:
				bms_handle_charger_connected();
				break;	
			case BMS_CHARGING:
				bms_handle_charging();
				break;
			case BMS_CHARGER_CONNECTED_NOT_CHARGING:
				bms_handle_charger_connected_not_charging();
				break;
			case BMS_CHARGER_UNPLUGGED:
				bms_handle_charger_unplugged();
				break;
			case BMS_DISCHARGING:
        prot_init();
				bms_handle_discharging();
				leds_blink_error_led(500);
        const uint8_t tst_vector[] = {0x12, 0x38, 0x00, 0xC1, 0x01, 0xC0, 0x02, 0x01, 0x07, 0x01, 0x10, 0x00, 0x81, 0x01, 0x00, 0x00, 0x01, 0x10, 0x01, 0x21, 0x01, 0x00, 0x00, 0x01, 0x10, 0x0D, 0x25, 0x02, 0x00, 0x03, 0x26, 0x01, 0x10, 0x02, 0x22, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x10, 0x05, 0x81, 0x02, 0x00, 0x67, 0x11, 0x01, 0x10, 0x06, 0x81, 0x01, 0x00, 0x01};
        crc32 = calc_crc32(tst_vector, sizeof(tst_vector));

        sprintf(debug_msg_buffer,"CRC32 at 0x%X \r\n", crc32);
        serial_debug_send_message(debug_msg_buffer);

				break;
			case BMS_FAULT:
				bms_handle_fault();
				break;
		}
	}
}

