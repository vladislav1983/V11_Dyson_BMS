/*
 * bq7693.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */


#include "bq7693.h"

void bq7693_i2c_init(void);

//"internal" function primitives
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
 * @brief Set peripheral function for a pin via direct register access.
 *
 * @param pinmux  Pin multiplexer configuration value.
 */
static inline void pin_set_peripheral_function(uint32_t pinmux)
{
  uint8_t port = (uint8_t)((pinmux >> 16)/32);
  PORT->Group[port].PINCFG[((pinmux >> 16) - (port*32))].bit.PMUXEN = 1;
  PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg &= ~(0xF << (4 * ((pinmux >>16) & 0x01u)));
  PORT->Group[port].PMUX[((pinmux >> 16) - (port*32))/2].reg |= (uint8_t)((pinmux & 0x0000FFFF) << (4 * ((pinmux >> 16) & 0x01u)));
}

/** @brief Initialize I2C master on SERCOM1 for BQ7693 communication. */
void bq7693_i2c_init()
 {
  //I2C peripheral init
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

/** @brief Initialize BQ7693 protector IC: read ADC cal, set protection thresholds, enable CC. */
void bq7693_init()
{
  bq7693_i2c_init();
  bq7693_write_register(SYS_CTRL2, 0x00); //Ensure that charge/discharge FETs are off so pack is safe.

  //Read the ADC offset and gain values and store
  uint8_t scratch1, scratch2;
  bq7693_read_register(ADCOFFSET, 1, &scratch1);  // convert from 2's complement
  bq7693_adc_offset = (int8_t)scratch1;
  bq7693_read_register(ADCGAIN1, 1, &scratch1);
  bq7693_read_register(ADCGAIN2, 1, &scratch2);
  bq7693_adc_gain = 365 + ((( scratch1 & 0x0C) << 1) | (( scratch2 & 0xE0) >> 5)); // uV/LSB

  bq7693_write_register(PROTECT1, 0x82);
  bq7693_write_register(PROTECT2, 0x04);

  //This sets overvolt and undervolt delays to 1 second.
  bq7693_write_register(PROTECT3, 0x00);

  //Calculate OV and UV trip voltages.
  scratch1 = (((((long)CELL_OVERVOLTAGE_TRIP - bq7693_adc_offset)*1000)/ bq7693_adc_gain) >> 4) & 0xFF;
  bq7693_write_register(OV_TRIP, scratch1);

  scratch1 = (((((long)CELL_UNDERVOLTAGE_TRIP - bq7693_adc_offset) * 1000) / bq7693_adc_gain) >> 4) & 0xFF;
  bq7693_write_register(UV_TRIP, scratch1);

  bq7693_write_register(CELLBAL1, 0x00); //Disable cell balancing 1
  bq7693_write_register(CELLBAL2, 0x00); //Disable cell balancing 2

  bq7693_write_register(CC_CFG, 0x19); //'magic' value as per datasheet.
  bq7693_write_register(SYS_CTRL2, 0x40); //CC_EN - enable continuous operation of coulomb counter

  bq7693_write_register(SYS_CTRL1, 0x10); //ADC_EN

  bq7693_read_register(SYS_STAT, 1, &scratch1);
  bq7693_write_register(SYS_STAT, scratch1); //Explicitly clear any set bits in the SYS_STAT register by writing them back.
}

/**
 * @brief Read one or more bytes from a BQ7693 register via I2C.
 *
 * @param addr  Register address to read from.
 * @param len   Number of bytes to read.
 * @param buf   Buffer to store the read data.
 * @return      true on success.
 */
bool bq7693_read_register(uint8_t addr, size_t len, uint8_t *buf)
 {
  //Disable interrupts from the EIC - we don't want to end up trying to read the
  //charge counter half way through an existing i2c op. Re-enable at the end.
  system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_EIC);

  uint16_t timeout = 0;
  bool result = true;

  //Initial write to set register address.
  struct i2c_master_packet packet =
  {
    .address = BQ7693_ADDR,
    .data_length = 1,
    .data = &addr
  };

  //Tx address
  while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK)
  {
    /* Increment timeout counter and check if timed out. */
    if (timeout++ >= BQ7693_TIMEOUT)
    {
      break;
    }
  }
  //Rx value
  packet.data_length = len;
  packet.data = buf;
  timeout = 0;

  while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) != STATUS_OK)
  {
    /* Increment timeout counter and check if timed out. */
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
 * @brief Write a single byte to a BQ7693 register with CRC.
 *
 * @param addr   Register address to write to.
 * @param value  Byte value to write.
 * @return       true on success.
 */
bool bq7693_write_register(uint8_t addr, uint8_t value)
{
  //Disable interrupts from the EIC - we don't want to end up trying to read the
  //charge counter half way through an existing i2c op. Re-enable at the end.
  system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_EIC);

  uint16_t timeout = 0;
  bool result = true;

  uint8_t buf[3];
  buf[0] = addr;
  buf[1] = value;

  //Calculate CRC (over slave address + rw bit, reg address + data.
  uint8_t crc = bq7693_calc_checksum(0x00, (BQ7693_ADDR <<1) | 0);
  crc = bq7693_calc_checksum(crc, buf[0]);
  crc = bq7693_calc_checksum(crc, buf[1]);
  buf[2] = crc;

  //Initial write to set register address.
  struct i2c_master_packet packet =
  {
    .address = BQ7693_ADDR,
    .data_length = 3,
    .data = buf
  };

  while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK)
  {
    /* Increment timeout counter and check if timed out. */
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
 * @brief Compute BQ7693 I2C CRC (poly 0x07).
 *
 * @param inCrc   Current CRC accumulator value.
 * @param inData  Next data byte to fold in.
 * @return        Updated CRC byte.
 */
uint8_t bq7693_calc_checksum(uint8_t inCrc, uint8_t inData)
{
  // CRC is calculated over the slave address (including R/W bit), register address, and data.
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

/** @brief Clear SYS_STAT errors and enable the charge FET. */
void bq7693_enable_charge(void)
{
  uint8_t scratch;
  //Clear any bits in the SYS_STAT error register
  bq7693_read_register(SYS_STAT, 1, &scratch);
  bq7693_write_register(SYS_STAT, scratch); //Explicitly clear any set bits in the SYS_STAT register by writing them back.

  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 | SYS_CTRL2_CC_EN | SYS_CTRL2_CHG_ON);
}

/** @brief Disable the charge FET, preserving DSG state. */
void bq7693_disable_charge(void)
{
  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 & ~SYS_CTRL2_CHG_ON);
}

/** @brief Configure protection, clear errors, and enable the discharge FET. */
void bq7693_enable_discharge(void)
{
  bq7693_write_register(SYS_CTRL1, 0x10);  //ADC_EN=1

  bq7693_write_register(PROTECT1, 0x9F);
  bq7693_write_register(PROTECT2, 0x04);

  uint8_t scratch;
  bq7693_read_register(SYS_STAT, 1, &scratch);
  bq7693_write_register(SYS_STAT, scratch); //Explicitly clear any set bits in the SYS_STAT register by writing them back.

  //DSG_ON turns the discharge FET on. Preserve CHG_ON so charging is not affected.
  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 | SYS_CTRL2_CC_EN | SYS_CTRL2_DSG_ON);

  bq7693_write_register(PROTECT2, 0x04);
  bq7693_write_register(PROTECT1, 0x82);
}

/** @brief Disable the discharge FET, preserving CHG state. */
void bq7693_disable_discharge(void)
{
  uint8_t ctrl2;
  bq7693_read_register(SYS_CTRL2, 1, &ctrl2);
  bq7693_write_register(SYS_CTRL2, ctrl2 & ~SYS_CTRL2_DSG_ON);
}

/**
 * @brief Read all 7 cell voltages from BQ7693 and apply ADC calibration.
 *
 * @return  Pointer to static array of 7 cell voltages in mV.
 */
uint16_t *bq7693_get_cell_voltages(void)
{
  uint8_t scratch[3];
  uint16_t tempval;
  //Voltages for each cell
  //The cells are connected as below on these packs...
  int cellsToRead[] = { 0,1,2,3,5,6,9};

  for (int i=0; i< 7; ++i)
  {
    //Because CRC is enabled, we need to read 3 bytes (VCx_HI, the CRC byte (ignore), then VCx_Lo)
    bq7693_read_register((VC1_HI_BYTE + 2*cellsToRead[i]), 3, scratch);
    tempval = ((scratch[0] & 0x3F) <<8) | scratch[2];
    bq7693_cell_voltages[i] = tempval * bq7693_adc_gain/1000 + bq7693_adc_offset;
  }

  return bq7693_cell_voltages;
}

/**
 * @brief Read total pack voltage from BQ7693 BAT register.
 *
 * @return  Pack voltage in mV.
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

/** @brief Put BQ7693 into SHIP mode (deep sleep). */
void bq7693_enter_sleep_mode(void)
{
  bq7693_write_register(SYS_CTRL1, 0x00);
  bq7693_write_register(SYS_CTRL1, 0x01);
  bq7693_write_register(SYS_CTRL1, 0x02);
}

/**
 * @brief Read raw coulomb counter value from BQ7693.
 *
 * @return  Signed 16-bit CC value.
 */
int16_t bq7693_read_cc(void)
{
  int16_t tempCC;

  uint8_t scratch[3];
  bq7693_read_register(CC_HI_BYTE, 3, scratch);
  tempCC =  ((scratch[0])<<8);
  tempCC |= scratch[2]; //ignore the unwanted CRC byte.

  return tempCC;
}
