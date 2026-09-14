/*
 * bms_adc.c
 *
 * Created: 21-Jan-26 10:09:58
 * Author : Vladislav Gyurov
 * License: GNU GPL v3 or later
 */
 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "bms_adc.h"

#define BMS_ADC_BUSY_LIMIT  65535u

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
static struct adc_module adc_instance;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/
static const enum adc_positive_input adc_ch_map_cfg[BMS_ADC_CH_NUM] =
{
  [BMS_ADC_CH_TC1]      = ADC_POSITIVE_INPUT_PIN7,  /* PA07 */
};

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**
 * @brief initialise the ADC, 12-bit single-shot, internal VCC/1.48 reference,
 *        ADC interrupts are hard-disabled
 */
bool bms_adc_init(void)
{
  struct adc_config config_adc;

  adc_get_config_defaults(&config_adc);

  config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
  config_adc.reference       = ADC_REFERENCE_INTVCC0;
  config_adc.resolution      = ADC_RESOLUTION_12BIT;
  config_adc.freerunning     = false;
  config_adc.negative_input  = ADC_NEGATIVE_INPUT_GND;

  // any channel works as the initial mux, the first conversion picks one
  config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN7;

  if (adc_init(&adc_instance, ADC, &config_adc) != STATUS_OK)
    return false;

  // force-disable ADC interrupts
  ADC->INTENCLR.reg = ADC_INTENCLR_MASK;
  ADC->INTFLAG.reg  = ADC_INTFLAG_MASK;

  adc_enable(&adc_instance);
  return true;
}

/**
 * @brief one-shot conversion on a single channel, blocks until done
 * @param ch  ADC channel
 * @return    12-bit result, or 0xFFFF on error
 */
uint16_t adc_convert_channel(bms_adc_ch_t ch)
{
  enum status_code status;
  uint16_t result = 0xFFFF;
  uint16_t busy_count = 0;

  if ((uint32_t)ch >= (uint32_t)BMS_ADC_CH_NUM)
    return 0xFFFF;

  enum adc_positive_input ch_mux = adc_ch_map_cfg[ch];

  adc_set_positive_input(&adc_instance, ch_mux);
  adc_start_conversion(&adc_instance);

  do
  {
    status = adc_read(&adc_instance, &result);
    busy_count++;
  } while (status == STATUS_BUSY && busy_count < BMS_ADC_BUSY_LIMIT);

  if (status != STATUS_OK)
    return 0xFFFF;

  return result;
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
