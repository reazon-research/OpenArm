#ifndef __LED_BSP_H__
#define __LED_BSP_H__

#include "main.h"

#define led_bsp_gpio_t  GPIO_TypeDef

typedef struct {
	led_bsp_gpio_t *port; //port address
	uint16_t  pin;
} led_bsp_t;

#define led_set    GPIO_PIN_SET
#define led_reset  GPIO_PIN_RESET

void led_bsp_init(void);
void led_bsp_on(led_bsp_gpio_t *port, uint16_t pin);
void led_bsp_off(led_bsp_gpio_t *port, uint16_t pin);
void led_bsp_toggle(led_bsp_gpio_t *port, uint16_t pin);

#endif /* __LED_BSP_H__ */

