/*
 * crc.c
 *
 * Created: 21-Jan-26 12:27:18
 * Author : Vladislav Gyurov
 * License: GNU GPL v3 or later
 */
 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "crc.h"

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL VARIABLES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL CONSTANTS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DECLARATION OF LOCAL MACROS/#DEFINES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/
static const uint32_t c_wCRC32Table[16] =
{
  0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
  0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
  0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
  0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C,
};

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**
 * @brief CRC32 matching the stock Dyson firmware
 *        poly 0x04C11DB7 (reflected 0xEDB88320), init 0xFFFFFFFF,
 *        RefIn/Out true, XorOut 0xFFFFFFFF, pads to 4-byte alignment with zeros
 * @param data  input bytes
 * @param len   byte count
 * @return      CRC32
 */
uint32_t calc_crc32(const uint8_t *data, uint16_t len)
{
  uint16_t i;
  uint32_t crc = 0xFFFFFFFF;
  uint16_t padded_len = (len + 3u) & ~3u;

  for (i = 0; i < padded_len; i++)
  {
    uint8_t byte = (i < len) ? data[i] : 0x00;
    crc = (crc >> 4) ^ c_wCRC32Table[(crc & 0x0F) ^ (byte & 0x0F)];
    crc = (crc >> 4) ^ c_wCRC32Table[(crc & 0x0F) ^ (byte >> 4)];
  }

  return ~crc;
}

/**
 * @brief CRC8 matching the stock Dyson header checksum
 *        poly 0xE0 (reflected), init 0xFF, RefIn/Out true, XorOut 0xFF,
 *        used over the 2-byte SIZE field
 * @param data  input bytes
 * @param len   byte count
 * @return      CRC8
 */
uint8_t calc_crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0xFF;

  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x01)
        crc = (crc >> 1) ^ 0xE0;
      else
        crc >>= 1;
    }
  }

  return crc ^ 0xFF;
}

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
