/*
 * leds.c
 *
 * Author :  David Pye
 *  Contact: davidmpye@gmail.com
 *  License: GNU GPL v3 or later
 */

/*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "leds.h"
#include "sw_timer.h"

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
//speed of LED sequence
#define LED_SEQ_TIME          25
#define TIMRER_FREQ_HZ        (8000000ul)
#define PWM_FREQ_HZ           (200ul)
#define CAPTURE_VALUE         (TIMRER_FREQ_HZ / PWM_FREQ_HZ)

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/
typedef struct
{
  Tc *const                       hw;
  uint32_t                        pin_out;
  uint32_t                        pin_mux;
  enum tc_compare_capture_channel chnl;
}leds_cfg_t;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
struct tc_module tc_instances[LEDS_NUM];

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/
static const leds_cfg_t leds_cfg[] =
{
  [LEDS_LED_ERR_LEFT]  = {.hw = TC3, .pin_out = PIN_PA19F_TC3_WO1, .pin_mux = PINMUX_PA19F_TC3_WO1, .chnl = TC_COMPARE_CAPTURE_CHANNEL_1 }, // PIN_PA19
  [LEDS_LED_ERR_RIGHT] = {.hw = TC2, .pin_out = PIN_PA00F_TC2_WO0, .pin_mux = PINMUX_PA00F_TC2_WO0, .chnl = TC_COMPARE_CAPTURE_CHANNEL_0 }, // PIN_PA00
};

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**
 * @brief Initialise LED PWM timers.
 *
 * Configures one TC instance per LED in 16-bit normal PWM mode
 * with duty initially set to 0 (off).
 */
 void leds_init(void)
 {
  struct tc_config config_tc;
  tc_get_config_defaults(&config_tc);

  for(uint8_t i = 0; i < (uint8_t)LEDS_NUM; i++)
  {
    config_tc.counter_size           = TC_COUNTER_SIZE_16BIT;
    config_tc.wave_generation        = TC_WAVE_GENERATION_NORMAL_PWM;
    config_tc.counter_16_bit.value   = 0;
    config_tc.counter_16_bit.compare_capture_channel[leds_cfg[i].chnl] = 0;
    config_tc.pwm_channel[0].enabled = true;
    config_tc.pwm_channel[0].pin_out = leds_cfg[i].pin_out;
    config_tc.pwm_channel[0].pin_mux = leds_cfg[i].pin_mux;

    tc_init(&tc_instances[i], leds_cfg[i].hw, &config_tc);
    tc_enable(&tc_instances[i]);
    tc_set_compare_value(&tc_instances[i], leds_cfg[i].chnl, 0);
  }
}

/**
 * @brief Play a smooth fade-in / fade-out LED sequence.
 *
 * Both LEDs ramp up then down over 400 steps with 1 ms spacing.
 */
void leds_sequence(void)
{
  uint8_t duty;
  uint8_t duty_lim;

  for(int j = 0; j < 400; j++)
  {
    duty = (j + 0) % 200;
    duty_lim = (duty > 100) ? (200 - duty) : duty;
    duty_lim = (duty_lim < 20) ? 0 : (duty_lim);

    leds_set_led_duty(LEDS_LED_ERR_RIGHT, duty_lim);
    leds_set_led_duty(LEDS_LED_ERR_LEFT,  duty_lim);

    sw_timer_delay_ms(1);
  }
}

/**
 * @brief Turn off all LEDs (duty = 0%).
 */
void leds_off(void)
{
  const uint32_t cc_0_ppt = 0;

  for(uint8_t i = 0; i < (uint8_t)LEDS_NUM; i++)
  {
    tc_set_compare_value(&tc_instances[i], leds_cfg[i].chnl, cc_0_ppt);
  }
}

/**
 * @brief Turn on all LEDs at full brightness (duty = 100%).
 */
void leds_on(void)
{
  const uint32_t cc_100_ppt = (CAPTURE_VALUE);

  for(uint8_t i = 0; i < (uint8_t)LEDS_NUM; i++)
  {
    tc_set_compare_value(&tc_instances[i], leds_cfg[i].chnl, cc_100_ppt);
  }
}

/**
 * @brief Blink all LEDs once for a given duration.
 *
 * LEDs are on for the first half and off for the second half.
 *
 * @param ms  Total blink duration in milliseconds.
 */
void leds_blink_leds(uint32_t ms)
{
  const uint32_t cc_100_ppt = (CAPTURE_VALUE);
  const uint32_t cc_0_ppt = 0;

  for(uint8_t i = 0; i < (uint8_t)LEDS_NUM; i++)
  {
    tc_set_compare_value(&tc_instances[i], leds_cfg[i].chnl, cc_100_ppt);
  }

  sw_timer_delay_ms(ms / 2);

  for(uint8_t i = 0; i < (uint8_t)LEDS_NUM; i++)
  {
    tc_set_compare_value(&tc_instances[i], leds_cfg[i].chnl, cc_0_ppt);
  }

  sw_timer_delay_ms(ms / 2);
}

/**
 * @brief Blink a single LED once for a given duration.
 *
 * @param led  LED index.
 * @param ms   Total blink duration in milliseconds.
 */
void leds_blink_led(leds_t led, uint32_t ms)
{
  const uint32_t cc_100_ppt = (CAPTURE_VALUE);
  const uint32_t cc_0_ppt = 0;

  if(led < LEDS_NUM)
  {
    tc_set_compare_value(&tc_instances[led], leds_cfg[led].chnl, cc_100_ppt);
    sw_timer_delay_ms(ms / 2);
    tc_set_compare_value(&tc_instances[led], leds_cfg[led].chnl, cc_0_ppt);
    sw_timer_delay_ms(ms / 2);
  }
}

/**
 * @brief Blink a LED (or all LEDs) a specified number of times.
 *
 * @param led  LED index, or LEDS_NUM to blink all LEDs.
 * @param num  Number of blinks.
 * @param ms   Duration per blink in milliseconds.
 */
void leds_blink_leds_num(leds_t led, uint32_t num, uint32_t ms)
{
  for(uint32_t num_l = 0; num_l < num; num_l++)
  {
    if(led <= LEDS_NUM)
    {
      switch (led)
      {
        case LEDS_LED_ERR_LEFT:
        case LEDS_LED_ERR_RIGHT:
          leds_blink_led(led, ms);
        break;
        default:
          leds_blink_leds(ms);
        break;
      }
    }
  }
}

/**
 * @brief Set the PWM duty cycle of a single LED.
 *
 * @param led      LED index.
 * @param duty_ppt Duty cycle in percent (0–100).
 */
void leds_set_led_duty(leds_t led, uint8_t duty_ppt)
{
  if(led < LEDS_NUM)
  {
    uint32_t cc = (CAPTURE_VALUE / 100uL) * duty_ppt;
    tc_set_compare_value(&tc_instances[(uint8_t)led], leds_cfg[(uint8_t)led].chnl, cc);
  }
}

/*-----------------------------------------------------------------------------
  DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
  END OF MODULE
-----------------------------------------------------------------------------*/
