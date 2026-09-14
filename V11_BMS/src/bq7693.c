/*
 * bq7693.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */


#include "bq7693.h"
#include <string.h>

#define BQ7693_MAX_READ_LENGTH  16u

static bool bq7693_i2c_init(void);
static uint8_t bq7693_calc_checksum(uint8_t inCrc, uint8_t data);

static uint16_t bq7693_cell_voltages[PACK_CELL_COUNT];

static int bq7693_adc_gain = 0;       // in uV/LSB
static int8_t bq7693_adc_offset = 0;  // in mV

static struct i2c_master_module i2c_master_instance;

/** @brief bring up I²C master on SERCOM1 for BQ7693 traffic */
static bool bq7693_i2c_init(void)
{
  struct i2c_master_config config_i2c_master;

  i2c_master_get_config_defaults(&config_i2c_master);
  config_i2c_master.buffer_timeout            = BQ7693_TIMEOUT;
  config_i2c_master.unknown_bus_state_timeout = BQ7693_TIMEOUT;
  config_i2c_master.inactive_timeout          = BQ7693_TIMEOUT;
  config_i2c_master.pinmux_pad0               = PINMUX_PA16C_SERCOM1_PAD0;
  config_i2c_master.pinmux_pad1               = PINMUX_PA17C_SERCOM1_PAD1;
  config_i2c_master.scl_low_timeout           = true;

  if (i2c_master_init(&i2c_master_instance, SERCOM1, &config_i2c_master) != STATUS_OK)
    return false;

  i2c_master_enable(&i2c_master_instance);
  return true;
}

/** @brief configure the BQ7693: ADC calibration, protection, OV/UV trips, coulomb counter */
bool bq7693_init(void)
{
  uint8_t scratch1 = 0;
  uint8_t scratch2 = 0;

  if (!bq7693_i2c_init() || !bq7693_write_register(SYS_CTRL2, 0x00))
    return false;

  if (!bq7693_read_register(ADCOFFSET, 1, &scratch1))
    return false;

  bq7693_adc_offset = (int8_t)scratch1;

  if (!bq7693_read_register(ADCGAIN1, 1, &scratch1) || !bq7693_read_register(ADCGAIN2, 1, &scratch2))
    return false;
  bq7693_adc_gain = 365 + (((scratch1 & 0x0C) << 1) | ((scratch2 & 0xE0) >> 5)); // µV/LSB

  if (!bq7693_write_register(PROTECT1, 0x82) || !bq7693_write_register(PROTECT2, 0x04) || !bq7693_write_register(PROTECT3, 0x00))
    return false;

  scratch1 = (((((long)CELL_OVERVOLTAGE_TRIP - bq7693_adc_offset) * 1000) / bq7693_adc_gain) >> 4) & 0xFF;

  if (!bq7693_write_register(OV_TRIP, scratch1))
    return false;

  scratch1 = (((((long)CELL_UNDERVOLTAGE_TRIP - bq7693_adc_offset) * 1000) / bq7693_adc_gain) >> 4) & 0xFF;

  if (!bq7693_write_register(UV_TRIP, scratch1))
    return false;

  if (!bq7693_write_register(CELLBAL1, 0x00) || !bq7693_write_register(CELLBAL2, 0x00))
    return false;

  if (!bq7693_write_register(CC_CFG, 0x19) || !bq7693_write_register(SYS_CTRL2, 0x40) || !bq7693_write_register(SYS_CTRL1, 0x10))
    return false;

  return bq7693_read_register(SYS_STAT, 1, &scratch1) && bq7693_write_register(SYS_STAT, scratch1);
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
  uint8_t raw[BQ7693_MAX_READ_LENGTH * 2u];
  uint16_t timeout = 0;
  bool result = false;
  bool eic_was_enabled = false;

  if (buf != NULL && len > 0 && len <= BQ7693_MAX_READ_LENGTH)
  {
    struct i2c_master_packet packet = { .address = BQ7693_ADDR, .data_length = 1, .data = &addr };
    eic_was_enabled = system_interrupt_is_enabled(SYSTEM_INTERRUPT_MODULE_EIC);

    if (eic_was_enabled)
      system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_EIC);

    while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK && timeout++ < BQ7693_TIMEOUT);
    result = (timeout <= BQ7693_TIMEOUT);

    if (result)
    {
      packet.data_length = len * 2u;
      packet.data = raw;
      timeout = 0;

      while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) != STATUS_OK && timeout++ < BQ7693_TIMEOUT);
      result = (timeout <= BQ7693_TIMEOUT);
    }

    for (size_t i = 0; result && i < len; ++i)
    {
      uint8_t crc = 0;

      if (i == 0)
        crc = bq7693_calc_checksum(crc, (BQ7693_ADDR << 1) | 1u);

      crc = bq7693_calc_checksum(crc, raw[i * 2u]);
      result = (crc == raw[i * 2u + 1u]);

      if (result)
        buf[i] = raw[i * 2u];
    }
  }

  if (eic_was_enabled)
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
  uint16_t timeout = 0;
  bool result = true;
  bool eic_was_enabled = system_interrupt_is_enabled(SYSTEM_INTERRUPT_MODULE_EIC);

  if (eic_was_enabled)
    system_interrupt_disable(SYSTEM_INTERRUPT_MODULE_EIC);

  uint8_t buf[3];
  buf[0] = addr;
  buf[1] = value;

  // CRC over slave address + R/W bit, then register address, then data
  uint8_t crc = bq7693_calc_checksum(0x00, (BQ7693_ADDR << 1) | 0);
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

  if (eic_was_enabled)
    system_interrupt_enable(SYSTEM_INTERRUPT_MODULE_EIC);

  return result;
}

/**
 * @brief BQ7693 I²C CRC (polynomial 0x07)
 * @param inCrc   running CRC
 * @param inData  next byte
 * @return        updated CRC
 */
static uint8_t bq7693_calc_checksum(uint8_t inCrc, uint8_t inData)
{
  uint8_t i;
  uint8_t data;

  data = inCrc ^ inData;

  for (i = 0; i < 8; i++)
  {
    if ((data & 0x80) != 0)
    {
      data <<= 1;
      data ^= 0x07;
    }
    else
    {
      data <<= 1;
    }
  }

  return data;
}

// SYS_CTRL2 bits
#define SYS_CTRL2_CC_EN   0x40
#define SYS_CTRL2_DSG_ON  0x02
#define SYS_CTRL2_CHG_ON  0x01

/** @brief clear SYS_STAT errors and turn the charge FET on */
bool bq7693_enable_charge(void)
{
  uint8_t scratch = 0;
  uint8_t ctrl2 = 0;

  if (!bq7693_read_register(SYS_STAT, 1, &scratch) || !bq7693_write_register(SYS_STAT, scratch & STAT_FLAGS))
    return false;

  if (!bq7693_read_register(SYS_CTRL2, 1, &ctrl2))
    return false;

  return bq7693_write_register(SYS_CTRL2, ctrl2 | SYS_CTRL2_CC_EN | SYS_CTRL2_CHG_ON);
}

/** @brief turn the charge FET off, discharge FET state preserved */
bool bq7693_disable_charge(void)
{
  uint8_t ctrl2 = 0;

  if (!bq7693_read_register(SYS_CTRL2, 1, &ctrl2))
    return false;

  return bq7693_write_register(SYS_CTRL2, ctrl2 & ~SYS_CTRL2_CHG_ON);
}

/** @brief configure protection, clear errors and turn the discharge FET on */
bool bq7693_enable_discharge(void)
{
  uint8_t scratch = 0;
  uint8_t ctrl2 = 0;
  bool result = bq7693_write_register(SYS_CTRL1, 0x10);
  bool protect2_restored;
  bool protect1_restored;

  if (result)
    result = bq7693_write_register(PROTECT1, 0x9F);

  if (result)
    result = bq7693_write_register(PROTECT2, 0x04);

  if (result)
    result = bq7693_read_register(SYS_STAT, 1, &scratch);

  if (result)
    result = bq7693_write_register(SYS_STAT, scratch & STAT_FLAGS);

  if (result)
    result = bq7693_read_register(SYS_CTRL2, 1, &ctrl2);

  if (result)
    result = bq7693_write_register(SYS_CTRL2, ctrl2 | SYS_CTRL2_CC_EN | SYS_CTRL2_DSG_ON);

  protect2_restored = bq7693_write_register(PROTECT2, 0x04);
  protect1_restored = bq7693_write_register(PROTECT1, 0x82);
  result = result && protect2_restored && protect1_restored;

  return result;
}

/** @brief turn the discharge FET off, charge FET state preserved */
bool bq7693_disable_discharge(void)
{
  uint8_t ctrl2 = 0;

  if (!bq7693_read_register(SYS_CTRL2, 1, &ctrl2))
    return false;

  return bq7693_write_register(SYS_CTRL2, ctrl2 & ~SYS_CTRL2_DSG_ON);
}

/**
 * @brief read the seven cell voltages and apply ADC calibration
 * @return pointer to the static cell-voltage array, in mV
 */
uint16_t *bq7693_get_cell_voltages(void)
{
  uint8_t scratch[2];
  uint16_t values[PACK_CELL_COUNT];
  // V11/V15 wiring: only these BQ7693 channels are populated
  static const uint8_t cells_to_read[PACK_CELL_COUNT] = { 0, 1, 2, 3, 5, 6, 9 };

  for (uint8_t i = 0; i < PACK_CELL_COUNT; ++i)
  {
    if (!bq7693_read_register(VC1_HI_BYTE + 2u * cells_to_read[i], 2, scratch))
      return NULL;

    uint16_t raw = ((uint16_t)(scratch[0] & 0x3F) << 8) | scratch[1];
    int32_t calibrated = (int32_t)raw * bq7693_adc_gain / 1000 + bq7693_adc_offset;

    if (calibrated < 0 || calibrated > 6000)
      return NULL;
    values[i] = (uint16_t)calibrated;
  }

  memcpy(bq7693_cell_voltages, values, sizeof(values));

  return bq7693_cell_voltages;
}

/**
 * @brief read and sum all seven calibrated cell voltages
 * @return pack voltage in mV, or -1 when a cell snapshot cannot be read
 */
int bq7693_get_pack_voltage(void)
{
  uint16_t *cell_voltages = bq7693_get_cell_voltages();
  int32_t pack_mv = 0;

  if (cell_voltages == NULL)
    return -1;

  for (uint8_t i = 0; i < PACK_CELL_COUNT; ++i)
    pack_mv += cell_voltages[i];

  return (pack_mv <= 60000) ? (int)pack_mv : -1;
}

/** @brief put the BQ7693 into SHIP (deep sleep) mode */
bool bq7693_enter_sleep_mode(void)
{
  return bq7693_write_register(SYS_CTRL1, 0x00) && bq7693_write_register(SYS_CTRL1, 0x01) && bq7693_write_register(SYS_CTRL1, 0x02);
}

/**
 * @brief read the raw coulomb-counter value
 * @return signed 16-bit CC reading
 */
bool bq7693_read_cc(int16_t *value)
{
  uint8_t scratch[2];

  if (value == NULL || !bq7693_read_register(CC_HI_BYTE, 2, scratch))
    return false;

  *value = (int16_t)(((uint16_t)scratch[0] << 8) | scratch[1]);
  return true;
}
