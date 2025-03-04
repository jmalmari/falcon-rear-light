#pragma once

#include "driver/gpio.h"

extern char const *const TAG;

#define LED_STRIP_LED_NUMBERS 32

#define LEDGEN_DBG_GPIO
#ifdef LEDGEN_DBG_GPIO

#define LEDGEN_GPIO_DBG_ISR 10
#define LEDGEN_GPIO_DBG_IN 8
#define LEDGEN_GPIO_DBG_OUT 20

#define LEDGEN_GPIO_DBG_ISR_ON()  gpio_set_level(LEDGEN_GPIO_DBG_ISR, 1)
#define LEDGEN_GPIO_DBG_ISR_OFF() gpio_set_level(LEDGEN_GPIO_DBG_ISR, 0)
#define LEDGEN_GPIO_DBG_IN_ON()   gpio_set_level(LEDGEN_GPIO_DBG_IN, 1)
#define LEDGEN_GPIO_DBG_IN_OFF()  gpio_set_level(LEDGEN_GPIO_DBG_IN, 0)
#define LEDGEN_GPIO_DBG_OUT_ON()  gpio_set_level(LEDGEN_GPIO_DBG_OUT, 1)
#define LEDGEN_GPIO_DBG_OUT_OFF() gpio_set_level(LEDGEN_GPIO_DBG_OUT, 0)

#else

#define LEDGEN_GPIO_DBG_ISR_ON()
#define LEDGEN_GPIO_DBG_ISR_OFF()
#define LEDGEN_GPIO_DBG_IN_ON()
#define LEDGEN_GPIO_DBG_IN_OFF()
#define LEDGEN_GPIO_DBG_OUT_ON()
#define LEDGEN_GPIO_DBG_OUT_OFF()

#endif
