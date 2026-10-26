#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>

#ifndef LED_ACTIVE_HIGH
#define LED_ACTIVE_HIGH 1
#endif

typedef enum
{
    off = 0,
    on  = 1
} led_state_t;

typedef enum
{
    ph10 = 0,
    ph11,
    ph12,
    LED_PIN_COUNT
} led_pin_t;

extern void Led_set(led_pin_t pin, led_state_t state);

#endif
