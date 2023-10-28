#ifndef  GPIO_ALLWINNER_H6_H
#define  GPIO_ALLWINNER_H6_H

#include  <stdint.h>
#include  <linux/bitops.h>


typedef enum  {
	ALLWINNER_H6_GPIO_DRV_LEVEL0 = 0,
	ALLWINNER_H6_GPIO_DRV_LEVEL1,
    ALLWINNER_H6_GPIO_DRV_LEVEL2,
    ALLWINNER_H6_GPIO_DRV_LEVEL3,
    ALLWINNER_H6_GPIO_DRV_MAX = ALLWINNER_H6_GPIO_DRV_LEVEL3,
} allwinner_h6_gpio_drv_mode_e;


typedef enum {
	ALLWINNER_H6_GPIO_PUPD_NO = 0,
	ALLWINNER_H6_GPIO_PUPD_UP,
	ALLWINNER_H6_GPIO_PUPD_DOWN,
    ALLWINNER_H6_GPIO_MAX = ALLWINNER_H6_GPIO_PUPD_DOWN
} allwinner_h6_gpio_pupd_e;














#endif
