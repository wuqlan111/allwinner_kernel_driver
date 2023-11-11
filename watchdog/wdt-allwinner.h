#ifndef  GPIO_ALLWINNER_H6_H
#define  GPIO_ALLWINNER_H6_H

#include  <linux/bitops.h>
#include  <asm-generic/bitops-instrumented.h>

#define  ALLWINNER_WDT_IRQ_EN                   BIT(0)
#define  ALLWINNER_WDT_IRQ_PENDING              BIT(0)
#define  ALLWINNER_WDT_CTRL_KEY                 GENMASK(12,  1)
#define  ALLWINNER_WDT_CTRL_RESTART             BIT(0)
#define  ALLWINNER_WDT_CFG_CONFIG               GENMASK(1,   0)
#define  ALLWINNER_WDT_MODE_VALUE               GENMASK(7,   4)
#define  ALLWINNER_WDT_MODE_EN                  BIT(0)

#define  ALLWINNER_WDT_CTRL_KEY_FLAG            (0xa57ul<<1)

#define  ALLWINNER_WDT_MAP_SIZE                 (0x400u)

#define  ALLWINNER_WDT_MIN_TIMEOUT              (1)    //s
#define  ALLWINNER_WDT_MAX_TIMEOUT              (16)   //s
#define  ALLWINNER_WDT_REG_OFFSET               (0xa0u)

#endif
