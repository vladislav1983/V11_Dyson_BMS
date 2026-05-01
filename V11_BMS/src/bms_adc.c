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
static uint16_t adc_result[BMS_ADC_CH_NUM] = {0};
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
void bms_adc_init(void)
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

  adc_init(&adc_instance, ADC, &config_adc);

  // force-disable ADC interrupts
  ADC->INTENCLR.reg = ADC_INTENCLR_MASK;
  ADC->INTFLAG.reg  = ADC_INTFLAG_MASK;

  adc_enable(&adc_instance);
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

  if(ch < BMS_ADC_CH_NUM)
  {
    enum adc_positive_input ch_mux = adc_ch_map_cfg[ch];

    adc_set_positive_input(&adc_instance, ch_mux);
    adc_start_conversion(&adc_instance);

    do
    {
      status = adc_read(&adc_instance, &result);
    } while (status == STATUS_BUSY);

    if(status != STATUS_OK)
    {
      result = 0xFFFF;
    }

    adc_result[ch] = result;
  }

  return result;
}

/**
 * @brief convert every configured channel and cache the results,
 *        read them back with bms_adc_read_ch()
 */
void adc_convert_channels(void)
{
  enum status_code status;
  uint16_t result;

  for(uint16_t i = 0; i < (uint16_t)BMS_ADC_CH_NUM; i++)
  {
    enum adc_positive_input ch_mux = adc_ch_map_cfg[i];

    adc_set_positive_input(&adc_instance, ch_mux);
    adc_start_conversion(&adc_instance);

    do
    {
      status = adc_read(&adc_instance, &result);
    } while (status == STATUS_BUSY);

    if(status == STATUS_OK)
    {
      adc_result[i] = result;
    }
    else
    {
      adc_result[i] = 0xFFFF;
    }
  }
}

/**
 * @brief read the last cached ADC result for a channel
 * @param ch  ADC channel
 * @return    cached 12-bit value, or 0xFFFF on invalid channel
 */
uint16_t bms_adc_read_ch(bms_adc_ch_t ch)
{
  uint16_t adc_ch_value = 0xFFFF;

  if(ch < BMS_ADC_CH_NUM)
  {
    adc_ch_value = adc_result[ch];
  }

  return adc_ch_value;
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
