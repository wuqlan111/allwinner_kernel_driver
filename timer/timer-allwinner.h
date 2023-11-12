#ifndef  GPIO_ALLWINNER_H6_H
#define  GPIO_ALLWINNER_H6_H

#include  <linux/bitops.h>

#define  ALLWINNER_TIMER_NUMBER      (2u)

#define  ALLWINNER_TIMER_MAP_SIZE         (0x400u)

#define  ALLWINNER_TIMER_CTRL_MODE                BIT(7)
#define  ALLWINNER_TIMER_CTRL_CLK_PRES            GENMASK(6,  4)
#define  ALLWINNER_TIMER_CTRL_CLK_SRC             GENMASK(3,  2)
#define  ALLWINNER_TIMER_CTRL_RELOAD              BIT(1)
#define  ALLWINNER_TIMER_CTRL_EN                  BIT(0)

#endif
