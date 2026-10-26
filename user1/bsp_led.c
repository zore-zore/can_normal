#include "bsp_led.h"
#include "main.h"

#define LED_FIRST_PIN   10U
#define LED_PIN_MASK    (0x7UL << LED_FIRST_PIN)
#define LED_MODER_MASK  ((3UL << 20) | (3UL << 22) | (3UL << 24))
#define LED_MODER_OUT   ((1UL << 20) | (1UL << 22) | (1UL << 24))
#define LED_PUPDR_DOWN  ((2UL << 20) | (2UL << 22) | (2UL << 24))

#if LED_ACTIVE_HIGH
#define LED_BSRR_ON(mask)  (mask)
#define LED_BSRR_OFF(mask) ((mask) << 16U)
#else
#define LED_BSRR_ON(mask)  ((mask) << 16U)
#define LED_BSRR_OFF(mask) (mask)
#endif

static uint8_t led_ready;

static void led_hw_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;

    GPIOH->BSRR = LED_BSRR_OFF(LED_PIN_MASK);

    GPIOH->OTYPER &= ~LED_PIN_MASK;
    GPIOH->OSPEEDR &= ~LED_MODER_MASK;
    GPIOH->PUPDR &= ~LED_MODER_MASK;
    GPIOH->PUPDR |= LED_PUPDR_DOWN;
    GPIOH->MODER &= ~LED_MODER_MASK;
    GPIOH->MODER |= LED_MODER_OUT;

    led_ready = 1U;
}

void Led_set(led_pin_t pin, led_state_t state)
{
    uint32_t pin_mask;

    if ((uint32_t)pin >= (uint32_t)LED_PIN_COUNT)
    {
        return;
    }

    if (led_ready == 0U)
    {
        led_hw_init();
    }

    pin_mask = 1UL << (LED_FIRST_PIN + (uint32_t)pin);

    GPIOH->BSRR = (state != off) ? LED_BSRR_ON(pin_mask) : LED_BSRR_OFF(pin_mask);
}
