#ifndef  GPIO_ALLWINNER_H6_H
#define  GPIO_ALLWINNER_H6_H

#include  <linux/bitops.h>

#define  ALLWINNER_BANK_PC_PINS    17
#define  ALLWINNER_BANK_PD_PINS    27
#define  ALLWINNER_BANK_PF_PINS    7
#define  ALLWINNER_BANK_PG_PINS    15
#define  ALLWINNER_BANK_PH_PINS    11
#define  ALLWINNER_BANK_PL_PINS    11
#define  ALLWINNER_BANK_PM_PINS    5


typedef enum {
    ALLWINNER_BANK_PA =  0,
    ALLWINNER_BANK_PB,
    ALLWINNER_BANK_PC,
    ALLWINNER_BANK_PD,
    ALLWINNER_BANK_PE,
    ALLWINNER_BANK_PF,
    ALLWINNER_BANK_PG,
    ALLWINNER_BANK_PH,
    ALLWINNER_BANK_PL,
    ALLWINNER_BANK_PM,
    ALLWINNER_BANK_MAX = ALLWINNER_BANK_PM
} allwinner_pin_bank_e;


typedef enum {
	ALLWINNER_H6_PINMUX_AF0 = 0,
	ALLWINNER_H6_PINMUX_AF1,
	ALLWINNER_H6_PINMUX_AF2,
	ALLWINNER_H6_PINMUX_AF3,
	ALLWINNER_H6_PINMUX_AF4,
	ALLWINNER_H6_PINMUX_AF5,
	ALLWINNER_H6_PINMUX_AF6,
	ALLWINNER_H6_PINMUX_AF7,
	ALLWINNER_H6_PINMUX_MAX_AF  =  ALLWINNER_H6_PINMUX_AF7
} allwinner_h6_pinmux_af_e;


typedef enum {
	ALLWINNER_H6_GPIO_PUPD_NO = 0,
	ALLWINNER_H6_GPIO_PUPD_UP,
	ALLWINNER_H6_GPIO_PUPD_DOWN,
    ALLWINNER_H6_GPIO_MAX = ALLWINNER_H6_GPIO_PUPD_DOWN
} allwinner_h6_gpio_pupd_e;


#define ALLWINNER_H6_GPIO_IN    ALLWINNER_H6_PINMUX_AF0
#define ALLWINNER_H6_GPIO_OUT   ALLWINNER_H6_PINMUX_AF1

#define  ALLWINNNER_H6_PIN_AF_IDX(offset)      ((offset)>>3)
#define  ALLWINNNER_H6_PIN_AF_SHIFT(offset)    (((offset) & 0x7) << 2)
#define  ALLWINNNER_H6_PIN_AF_MASK              GENMASK(2, 0)


#define  ALLWINNNER_H6_PIN_PULL_IDX(offset)      ((offset)>>4)
#define  ALLWINNNER_H6_PIN_PULL_SHIFT(offset)    (((offset) & 0xf) << 1)
#define  ALLWINNNER_H6_PIN_PULL_MASK              GENMASK(1, 0)

#define  ALLWINNER_PINCTRL_MAP_SIZE           (0x400u)


#define  PINCTRL_BANK_MAX_PIN       32
#define  PINCTRL_PIN_NUMBER(bank, offset)         (  ((bank) << 8) | (offset) )
#define  ALLWINNER_PIN_NUMBER(bank, offset)        PINCTRL_PIN_NUMBER(ALLWINNER_BANK_##bank, offset) 
#define  PIN_NUMBER_BANK(pin)               (((pin) >> 8) & 0xff)
#define  PIN_NUMBER_OFFSET(pin)             ((pin) & 0xff)



#endif
