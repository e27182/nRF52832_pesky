#include <stdbool.h>
#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"

#define LEDS_NUMBER    3

#define LED_0          22
#define LED_1          23
#define LED_2          24

#define LEDS_LIST { LED_0, LED_1, LED_2 }

#define BSP_LED_0      LED_0
#define BSP_LED_1      LED_1
#define BSP_LED_2      LED_2

#define BSP_LED_0_MASK (1<<BSP_LED_0)
#define BSP_LED_1_MASK (1<<BSP_LED_1)
#define BSP_LED_2_MASK (1<<BSP_LED_2)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK)

#define LEDS_INVERT(leds_mask) do { uint32_t gpio_state = NRF_GPIO->OUT;      \
                              NRF_GPIO->OUTSET = ((leds_mask) & ~gpio_state); \
                              NRF_GPIO->OUTCLR = ((leds_mask) & gpio_state); } while (0)

#define LEDS_CONFIGURE(leds_mask) do { uint32_t pin;                  \
                                  for (pin = 0; pin < 32; pin++) \
                                      if ( (leds_mask) & (1 << pin) )   \
                                          nrf_gpio_cfg_output(pin); } while (0)


static const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;

int main(void)
{
    LEDS_CONFIGURE(LEDS_MASK);

    while (true)
    {
        for (int i = 0; i < LEDS_NUMBER; i++)
        {
            LEDS_INVERT(1 << leds_list[i]);
            nrf_delay_ms(1000);
        }
    }
}