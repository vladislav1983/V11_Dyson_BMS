/**
 * @file main.c
 * @brief entry point for the Dyson V11/V15 BMS firmware
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */

#include "bms.h"

/** @brief initialise the BMS and enter the main loop, never returns */
int main(void)
{
  bms_init();
  bms_mainloop();
}
