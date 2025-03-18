#ifndef __KEY_BSP_H__
#define __KEY_BSP_H__

#include "main.h"

#define key_bsp_gpio_t  GPIO_TypeDef

typedef struct
{
	key_bsp_gpio_t *port; //port address
	uint16_t  pin;
} key_bsp_t;

#define key_set    GPIO_PIN_SET
#define key_reset  GPIO_PIN_RESET

#define key_bsp_state		GPIO_PinState

/* key io */
#define key_1 key_bsp_read_pin(GPIOA, GPIO_PIN_15)
//#define key_2 key_bsp_read_pin(GPIOC, GPIO_PIN_7)
//#define key_3 key_bsp_read_pin(GPIOC, GPIO_PIN_8)
//#define key_4 key_bsp_read_pin(GPIOC, GPIO_PIN_9)

void key_bsp_init(void);
key_bsp_state key_bsp_read_pin(key_bsp_gpio_t *port, uint16_t pin);
uint32_t get_key_bsptick(void);

#endif /* __KEY_BSP_H__ */

