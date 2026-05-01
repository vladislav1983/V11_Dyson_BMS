/*
 * bq7693.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */


#include "bq7693.h"

void bq7693_i2c_init(void);

// "internal" function primitives
int bq7693_read_block(uint8_t start_addr, size_t len, uint8_t* buf);
int bq7693_write_block(uint8_t start_addr, size_t len, uint8_t *buf);
uint8_t bq7693_calc_checksum(uint8_t inCrc, uint8_t data);

uint16_t bq7693_cell_voltages[7];

volatile int bq7693_adc_gain = 0;   // in uV/LSB
volatile int8_t bq7693_adc_offset = 0; //in mV

// maps for settings in chip protection registers

const int SCD_delay_setting [4] =
{ 70, 100, 200, 400 };

const int SCD_threshold_setting [8] =
{ 44, 67, 89, 111, 133, 155, 178, 200 }; // mV

const int OCD_delay_setting [8] =
{ 8, 20, 40, 80, 160, 320, 640, 1280 }; // ms
const int OCD_threshold_setting [16] =
{ 17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94, 100 };  // mV

const uint8_t UV_delay_setting [4] = { 1, 4, 8, 16 }; // s
const uint8_t OV_delay_setting [4] = { 1, 2, 4, 8 }; // s

struct i2c_master_module i2c_master_instance;

/**
 * @brief set a pin's peripheral mux via direct register access
 * @param pinmux  pin multiplexer configuration value
 */
static inline void pin_set_peripheral_function(uint32_t pinmux)
{
  uint8_t port = (uint8_t)((pinmux >> 16)/32);
  PORT->Group[port].PINCFG[((pinmux >> 16) - (port*32))].bit.PMUXEN = 1;
  PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg &= ~(0xF << (4 * ((pinmux >>16) & 0x01u)));
  PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg |= (uint8_t)((pinmux & 0x0000FFFF) << (4 * ((pinmux >> 16) & 0x01u)));
}

/** @brief bring up I²C master on SERCOM1 for BQ7693 traffic */
void bq7693_i2c_init()
 {
  struct i2c_master_config config_i2c_master;

  i2c_master_get_config_defaults(&config_i2c_master);
  config_i2c_master.buffer_timeout            = BQ7693_TIMEOUT;
  config_i2c_master.unknown_bus_state_timeout = BQ7693_TIMEOUT;
  config_i2c_master.inactive_timeout          = BQ7693_TIMEOUT;
  config_i2c_master.pinmux_pad0               = PINMUX_PA16C_SERCOM1_PAD0;
  config_i2c_master.pinmux_pad1               = PINMUX_PA17C_SERCOM1_PAD1;
  config_i2c_master.scl_low_timeout           = true;

  i2c_master_init(&i2c_master_instance, SERCOM1, &config_i2c_master);
  i2c_master_enable(&i2c_master_instance);
}

/** @brief configure the BQ7693: ADC calibration, protection, OV/UV trips, coulomb counter */
void bq7693_init()
{
  bq7693_i2c_init();
  bq7693_write_register(SYS_CTRL2, 0x00); // both FETs off — pack safe before configuring

  // read ADC offset and gain, in two's complement
  uint8_t scratch1, scratch2;
  bq7693_read_register(ADCOFFSET, 1, &scratch1);
  bq7693_adc_offset = (int8_t)scratch1;
  bq7693_read_register(ADCGAIN1, 1, &scratch1);
  bq7693_read_register(ADCGAIN2, 1, &scratch2);
  bq7693_adc_gain = 365 + ((( scratch1 & 0x0C) << 1) | (( scratch2 & 0xE0) >> 5)); // µV/LSB

  bq7693_write_register(PROTECT1, 0x82);
  bq7693_write_register(PROTECT2, 0x04);

  // OV/UV delays = 1 s
  bq7693_write_register(PROTECT3, 0x00);

  // translate the configured trip voltages into BQ7693 raw units
  scratch1 = (((((long)CELL_OVERVOLTAGE_TRIP - bq7693_adc_offset)*1000)/ bq7693_adc_gain) >> 4) & 0xFF;
  bq7693_write_register(OV_TRIP, scratch1);

  scratch1 = (((((long)CELL_UNDERVOLTAGE_TRIP - bq7693_adc_offset) * 1000) / bq7693_adc_gain) >> 4) & 0xFF;
  bq7693_write_register(UV_TRIP, scratch1);

  bq7693_write_register(CELLBAL1, 0x00);  // cell balancing off
  bq7693_write_register(CELLBAL2, 0x00);

  bq7693_write_register(CC_CFG, 0x19);    // datasheet-mandated value
  bq7693_write_register(SYS_CTRL2, 0x40); // CC_EN: continuous coulomb counter

  bq7693_write_register(SYS_CTRL1, 0x10); // ADC_EN

  // clear any latched SYS_STAT bits by writing them back
  bq7693_read_register(SYS_STAT, 1, &scratch1);
  bq7693_write_register(SYS_STAT, scratch1);
}

/**
 * @brief I²C read from a BQ7693 register
 * @param addr  register address
 * @param len   number of bytes to read
 * @param buf   destination buffer
 * @return      true on success
 */
bool bq7693_read_register(uint8_t addr, size_t len, uint8_t *buf)
 {
  // mask EIC during the I²C transaction so the BQ7693 ALERT ISR can't
  // reentrantly read the coulomb counter mid-transfer
  system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_EIC);

  uint16_t timeout = 0;
  bool result = true;

  // address phase
  struct i2c_master_packet packet =
  {
    .address = BQ7693_ADDR,
    .data_length = 1,
    .data = &addr
  };

  while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK)
  {
    if (timeout++ >= BQ7693_TIMEOUT)
    {
      break;
    }
  }
  // data phase
  packet.data_length = len;
  packet.data = buf;
  timeout = 0;

  while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) != STATUS_OK)
  {
    if (timeout++ >= BQ7693_TIMEOUT)
    {
      result = false;
      break;
    }
  }

  system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_EIC);
  return result;
}

/**
 * @brief I²C write of a single register byte, with CRC
 * @param addr   register address
 * @param value  byte to write
 * @return       true on success
 */
bool bq7693_write_register(uint8_t addr, uint8_t value)
{
  // mask EIC during the I²C transaction (see bq7693_read_register)
  system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_EIC);

  uint16_t timeout = 0;
  bool result = true;

  uint8_t buf[3];
  buf[0] = addr;
  buf[1] = value;

  // CRC over slave address + R/W bit, then register address, then data
  uint8_t crc = bq7693_calc_checksum(0x00, (BQ7693_ADDR <<1) | 0);
  crc = bq7693_calc_checksum(crc, buf[0]);
  crc = bq7693_calc_checksum(crc, buf[1]);
  buf[2] = crc;

  struct i2c_master_packet packet =
  {
    .address = BQ7693_ADDR,
    .data_length = 3,
    .data = buf
  };

  while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK)
  {
    if (timeout++ == BQ7693_TIMEOUT)
    {
      result = false;
      break;
    }
  }
  system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_EIC);
  return result;
}

/**
 * @brief BQ7693 I²C CRC (polynomial 0x07)
 * @param inCrc   running CRC
 * @param inData  next byte
 * @return        updated CRC
 */
uint8_t bq7693_calc_checksum(uint8_t inCrc, uint8_t inData)
{
  uint8_t i;
  uint8_t data;
  data = inCrc ^ inData;
  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else data <<= 1;
  }
  return data;
}

// SYS_CTRL2 bits
#define SYS_CTRL2_CC_EN   0x40
#define SYS_CTRL2_DSG_ON  0x02
#define SYS_CTRL2_CHG_ON  0x01

/** @brief clear SYS_STAT errors and turn the charge FET on */
void bq7693_enable_charge(void)
{
  uint8_t scratch;
  bq7693_read_register(SYS_STAT, 1, &scratch);
  bq7693_write_register(SYS_STAT, scratch);    // clear latched bits

  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 | SYS_CTRL2_CC_EN | SYS_CTRL2_CHG_ON);
}

/** @brief turn the charge FET off, discharge FET state preserved */
void bq7693_disable_charge(void)
{
  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 & ~SYS_CTRL2_CHG_ON);
}

/** @brief configure protection, clear errors and turn the discharge FET on */
void bq7693_enable_discharge(void)
{
  bq7693_write_register(SYS_CTRL1, 0x10);  // ADC_EN=1

  bq7693_write_register(PROTECT1, 0x9F);
  bq7693_write_register(PROTECT2, 0x04);

  uint8_t scratch;
  bq7693_read_register(SYS_STAT, 1, &scratch);
  bq7693_write_register(SYS_STAT, scratch);    // clear latched bits

  // set DSG_ON, preserve CHG_ON so charging is unaffected
  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 | SYS_CTRL2_CC_EN | SYS_CTRL2_DSG_ON);

  bq7693_write_register(PROTECT2, 0x04);
  bq7693_write_register(PROTECT1, 0x82);
}

/** @brief turn the discharge FET off, charge FET state preserved */
void bq7693_disable_discharge(void)
{
  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 & ~SYS_CTRL2_DSG_ON);
}

/**
 * @brief read the seven cell voltages and apply ADC calibration
 * @return pointer to the static cell-voltage array, in mV
 */
uint16_t *bq7693_get_cell_voltages(void)
{
  uint8_t scratch[3];
  uint16_t tempval;
  // V11/V15 wiring: only these BQ7693 channels are populated
  int cellsToRead[] = { 0,1,2,3,5,6,9};

  for (int i=0; i< 7; ++i)
  {
    // CRC mode returns 3 bytes per read: HI, CRC, LO, CRC byte is ignored
    bq7693_read_register((VC1_HI_BYTE + 2*cellsToRead[i]), 3, scratch);
    tempval = ((scratch[0] & 0x3F) <<8) | scratch[2];
    bq7693_cell_voltages[i] = tempval * bq7693_adc_gain/1000 + bq7693_adc_offset;
  }

  return bq7693_cell_voltages;
}

/**
 * @brief read pack voltage from the BAT register
 * @return pack voltage in mV
 */
int bq7693_get_pack_voltage(void)
{
  uint8_t scratch[3];
  uint16_t tempval;
  bq7693_read_register(BAT_HI_BYTE, 3, scratch);
  tempval = scratch[0] <<8 | scratch[2];
  int bq7693_pack_voltage = 4 * bq7693_adc_gain * tempval / 1000 + ( 7 * bq7693_adc_offset);
  return bq7693_pack_voltage;
}

/** @brief put the BQ7693 into SHIP (deep sleep) mode */
void bq7693_enter_sleep_mode(void)
{
  bq7693_write_register(SYS_CTRL1, 0x00);
  bq7693_write_register(SYS_CTRL1, 0x01);
  bq7693_write_register(SYS_CTRL1, 0x02);
}

/**
 * @brief read the raw coulomb-counter value
 * @return signed 16-bit CC reading
 */
int16_t bq7693_read_cc(void)
{
  int16_t tempCC;

  uint8_t scratch[3];
  bq7693_read_register(CC_HI_BYTE, 3, scratch);
  tempCC =  ((scratch[0])<<8);
  tempCC |= scratch[2];   // skip CRC byte

  return tempCC;
}
