/*
 *  eeprom_handler.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */

#include "eeprom_handler.h"
volatile struct eeprom_data eeprom_data;

/**
 * @brief reset EEPROM contents to factory defaults and commit,
 *        zeros the whole struct then sets the non-zero fields
 */
void eeprom_write_defaults(void)
{
  memset((void *)&eeprom_data, 0, sizeof(eeprom_data));
  eeprom_data.total_pack_capacity  = (PACK_MAX_CAPACITY_MAH     * 1200ul);
  eeprom_data.current_charge_level = ((PACK_MAX_CAPACITY_MAH/2) * 1000ul);
  eeprom_write();
}

/**
 * @brief bring up the EEPROM emulator, programs fuses on first use and
 *        rewrites defaults on a CRC mismatch or invalid fields
 * @return ASF status from eeprom_emulator_init()
 */
int eeprom_init(void)
{
  enum status_code error_code = eeprom_emulator_init();

  if (error_code == STATUS_ERR_NO_MEMORY)
  {
    // fuses are still 0x07, EEPROM is disabled, flash a few slow blinks so
    // the user knows we're doing something, then program the fuses (which
    // resets the MCU)
    for (int i=0; i<4; ++i)
    {
      leds_blink_leds(2000);
    }
    eeprom_fuses_set();                    // does not return
  }
  else if (error_code != STATUS_OK)
  {
    // wipe and reformat
    eeprom_emulator_erase_memory();
    error_code = eeprom_emulator_init();
    eeprom_write_defaults();
  }
  else
  {
    // emulator is happy, load and validate
    if (eeprom_read() != 0)
    {
      eeprom_write_defaults();             // CRC mismatch
    }
    else if (eeprom_data.full_discharge_seen > 1 || eeprom_data.imbalance_locked > 1)
    {
      // a boolean byte outside {0,1} means uninitialised (erased flash = 0xFF)
      // or written by a firmware whose struct layout didn't cover this byte,
      // treat as corrupt and rewrite
      eeprom_write_defaults();
    }
  }

  return error_code;
}

/**
 * @brief read EEPROM page 0 and verify the CRC
 * @return 0 on success, -1 on CRC mismatch
 */
int eeprom_read(void)
{
  uint8_t buffer[EEPROM_PAGE_SIZE];
  eeprom_emulator_read_page(0, buffer);
  memcpy((void*)&eeprom_data, buffer, sizeof(eeprom_data));

  // CRC covers every byte before the crc32 field
  uint32_t calc = calc_crc32((const uint8_t *)&eeprom_data,
      sizeof(eeprom_data) - sizeof(eeprom_data.crc32));
  if (calc != eeprom_data.crc32) {
    return -1;
  }
  return 0;
}

/**
 * @brief compute CRC and write EEPROM page 0
 * @return always 0
 */
int eeprom_write(void)
{
  eeprom_data.crc32 = calc_crc32((const uint8_t *)&eeprom_data,
      sizeof(eeprom_data) - sizeof(eeprom_data.crc32));

  uint8_t buffer[EEPROM_PAGE_SIZE];
  memcpy(buffer, (const void*)&eeprom_data, sizeof(eeprom_data));
  eeprom_emulator_write_page(0, buffer);
  eeprom_emulator_commit_page_buffer();
  return 0;
}

/**
 * @brief program the NVM fuses to enable a 1024-byte EEPROM region, then reset
 * @return does not return — triggers NVIC_SystemReset()
 */
int eeprom_fuses_set(void)
{
  struct nvm_config config_nvm;
  nvm_get_config_defaults(&config_nvm);
  nvm_set_config(&config_nvm);

  uint32_t temp;
  uint32_t data[2];

  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY));

  // read existing 64-bit user-row fuse word
  data[0] = *((uint32_t *)NVMCTRL_AUX0_ADDRESS);
  data[1] = *(((uint32_t *)NVMCTRL_AUX0_ADDRESS) + 1);

  // EEPROM size lives in bits 4-6, clear then set 0b100 = 1024 bytes / 4 rows
  data[0] &= ~0x00000070;
  data[0] |=  0x00000040;

  // writeback sequence per Microchip KB:
  // https://microchip.my.site.com/s/article/SAMD20-SAMD21-Programming-the-fuses-from-application-code

  // disable cache during the operation
  temp = NVMCTRL->CTRLB.reg;
  NVMCTRL->CTRLB.reg = temp | NVMCTRL_CTRLB_CACHEDIS;

  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;
  NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS/2;

  // erase the user page
  NVMCTRL->CTRLA.reg = NVM_COMMAND_ERASE_AUX_ROW | NVMCTRL_CTRLA_CMDEX_KEY;
  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY));

  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;
  NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS/2;

  // clear the page buffer before staging new data
  NVMCTRL->CTRLA.reg = NVM_COMMAND_PAGE_BUFFER_CLEAR | NVMCTRL_CTRLA_CMDEX_KEY;
  while (!(NVMCTRL->INTFLAG.reg & NVMCTRL_INTFLAG_READY));

  NVMCTRL->STATUS.reg |= NVMCTRL_STATUS_MASK;
  NVMCTRL->ADDR.reg = NVMCTRL_AUX0_ADDRESS/2;

  // stage updated fuse bits
  *((uint32_t *)NVMCTRL_AUX0_ADDRESS) = data[0];
  *(((uint32_t *)NVMCTRL_AUX0_ADDRESS) + 1) = data[1];

  // commit the user-row write
  NVMCTRL->CTRLA.reg = NVM_COMMAND_WRITE_AUX_ROW | NVMCTRL_CTRLA_CMDEX_KEY;

  // restore cache config
  NVMCTRL->CTRLB.reg = temp;

  NVIC_SystemReset();
}
