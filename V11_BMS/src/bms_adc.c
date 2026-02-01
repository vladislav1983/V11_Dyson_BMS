/*
 * bms_adc.c
 *
 * Created: 21-Jan-26 10:09:58
 *  Author: GYV1SF4
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
  [BMS_ADC_CH_TC1] = ADC_POSITIVE_INPUT_PIN7,  /* PA07 */
  [BMS_ADC_CH_TC2] = ADC_POSITIVE_INPUT_PIN16  /* PA08 */
};

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/
//- **************************************************************************
//! \brief
//- **************************************************************************
void bms_adc_init(void)
{
  struct adc_config config_adc;

  adc_get_config_defaults(&config_adc);

  config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV16;
  config_adc.reference       = ADC_REFERENCE_INTVCC0; /** 1/1.48V<SUB>CC</SUB> reference */
  config_adc.resolution      = ADC_RESOLUTION_12BIT;
  config_adc.freerunning     = false;
  config_adc.negative_input  = ADC_NEGATIVE_INPUT_GND;

  /* Initial channel does not matter */
  config_adc.positive_input  = ADC_POSITIVE_INPUT_PIN7;

  adc_init(&adc_instance, ADC, &config_adc);

  /* HARD disable ADC interrupts */
  ADC->INTENCLR.reg = ADC_INTENCLR_MASK;
  ADC->INTFLAG.reg  = ADC_INTFLAG_MASK;

  adc_enable(&adc_instance);
}

//- **************************************************************************
//! \brief
//- **************************************************************************
void adc_convert_channels(void)
{
  enum status_code status;
  uint16_t result;

  for(uint16_t i = 0; i < (uint16_t)BMS_ADC_CH_NUM; i++)
  {
    enum adc_positive_input ch_mux = adc_ch_map_cfg[i];

    /* Select ADC channel */
    adc_set_positive_input(&adc_instance, ch_mux);

    /* Start one-shot conversion */
    adc_start_conversion(&adc_instance);

    /* Poll until conversion is complete */
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

//- **************************************************************************
//! \brief 
//- **************************************************************************
uint16_t bms_adc_read_ch(bms_adc_ch_type ch)
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
//- **************************************************************************
//! \brief 
//- **************************************************************************

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
