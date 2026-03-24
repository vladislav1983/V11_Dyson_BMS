/*
 * dio.c
 *
 * Created: 13/02/2026 22:13:02
 * Author : Vladislav Gyurov
 * License: GNU GPL v3 or later
 */
 /*-----------------------------------------------------------------------------
    INCLUDE FILES
-----------------------------------------------------------------------------*/
#include "dio.h"
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
#define DIO_TASK_TICKS      (10 / SW_TIMER_TICK_MS)

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL TYPES
-----------------------------------------------------------------------------*/
typedef struct
{
  const uint8_t gpio_pin;
  uint8_t deb_ticks;
}dio_cfg_t;

typedef struct
{
  uint8_t  value_old;
  uint8_t  debounced_value;
  uint16_t debounce_counter;
}dio_data_t;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL VARIABLES
-----------------------------------------------------------------------------*/
static dio_data_t dio_data[DIO_NUM] =
{
  [DIO_CHARGER_CONNECTED] = {.value_old = 0, .debounced_value = 0, .debounce_counter = 0 },
  [DIO_MODE_BUTTON      ] = {.value_old = 0, .debounced_value = 0, .debounce_counter = 0 },
  [DIO_TRIGGER_PRESSED  ] = {.value_old = 0, .debounced_value = 0, .debounce_counter = 0 },
};

static sw_timer task_timer = 0;

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL CONSTANTS
-----------------------------------------------------------------------------*/
static const dio_cfg_t dio_cfg[DIO_NUM] =
{
  [DIO_CHARGER_CONNECTED] = {.gpio_pin = CHARGER_CONNECTED_PIN, .deb_ticks = (50 / DIO_TASK_TICKS)},
  [DIO_MODE_BUTTON      ] = {.gpio_pin = MODE_BUTTON_PIN,       .deb_ticks = (50 / DIO_TASK_TICKS)},
  [DIO_TRIGGER_PRESSED  ] = {.gpio_pin = TRIGGER_PRESSED_PIN,   .deb_ticks = (50 / DIO_TASK_TICKS)},
};

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS PROTOTYPES
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    DEFINITION OF GLOBAL FUNCTIONS
-----------------------------------------------------------------------------*/

/**
 * @brief Initialise the digital I/O module.
 *
 * Starts the periodic task timer used by dio_mainloop().
 */
void dio_init(void)
{
  sw_timer_start(&task_timer);
}

/**
 * @brief Periodic DIO task — poll and debounce every digital input.
 *
 * Called from the main loop.  Runs every DIO_TASK_TICKS milliseconds.
 */
void dio_mainloop(void)
{
  bool in;
  dio_data_t* d;
  const dio_cfg_t* c;

  if(true == sw_timer_is_elapsed(&task_timer, DIO_TASK_TICKS))
  {
    sw_timer_start(&task_timer);

    for (uint32_t i = 0; i < (uint32_t)DIO_NUM; i++)
    {
      d = &dio_data[i];
      c = &dio_cfg[i];
      in = port_pin_get_input_level(c->gpio_pin);
      dio_debounce(in, d->value_old, &(d->debounced_value), &(d->debounce_counter), c->deb_ticks);
    }
  }
}

/**
 * @brief Read the debounced value of a digital input.
 *
 * @param dio  Channel index (DIO_CHARGER_CONNECTED, etc.).
 * @return     true if the input is active, false otherwise.
 */
bool dio_read(dio_type_t dio)
{
  bool dio_level = false;

  if(dio < DIO_NUM)
  {
    dio_level = dio_data[(uint32_t)dio].debounced_value;
  }

  return dio_level;
}

/**
 * @brief Generic debounce algorithm.
 *
 * Counts consecutive samples where the current value matches value_old.
 * When the counter reaches zero the debounced output is updated.
 *
 * @param value                    Current raw input sample.
 * @param value_old                Previous raw input sample.
 * @param debounced_value          Pointer to the debounced output.
 * @param debounce_counter         Pointer to the remaining tick counter.
 * @param debounce_counter_preset  Number of ticks required for a stable reading.
 * @return true when the debounce has settled, false while counting.
 */
bool dio_debounce(uint8_t value, uint8_t value_old, uint8_t *debounced_value, uint16_t *debounce_counter, uint16_t debounce_counter_preset)
{
  bool debounce_finished = false;

  if ((*debounce_counter == 0) || (debounce_counter_preset == 0) || (debounced_value == NULL) || (debounce_counter == NULL))
  {
    debounce_finished = true;
    *debounce_counter = 0;
  }
  else
  {
    if (value != value_old)
    {
      *debounce_counter = debounce_counter_preset;
    }
    else
    {
      *debounce_counter = *debounce_counter - 1;
    }
  }

  if (*debounce_counter == 0)
  {
    debounce_finished = true;
    *debounced_value = value;                       /* latest value is considered to be the new debounced value */
  }

  return debounce_finished;
}

/*-----------------------------------------------------------------------------
    DEFINITION OF LOCAL FUNCTIONS
-----------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------
    END OF MODULE
-----------------------------------------------------------------------------*/
