/**
 * @file main.c
 * @brief Entry point for Dyson V11/V15 BMS firmware.
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */

#include "bms.h"

/**
 * @brief Application entry point. Initializes BMS and enters main loop.
 * @return Never returns.
 */
int main(void)
{
  bms_init();
  bms_mainloop();
  //never returns.
}
