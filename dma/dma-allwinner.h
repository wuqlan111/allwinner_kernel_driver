#ifndef  GPIO_ALLWINNER_H6_H
#define  GPIO_ALLWINNER_H6_H

#include  <linux/bitops.h>

#define  ALLWINNER_LOSC_CTRL_KEY               GENMASK(31,  16)
#define  ALLWINNER_LOSC_CTRL_AUTO_SWT          BIT(14)
#define  ALLWINNER_LOSC_CTRL_ALM_DD            BIT(9)
#define  ALLWINNER_LOSC_CTRL_RTC_HH            BIT(8)
#define  ALLWINNER_LOSC_CTRL_RTC_YY            BIT(7)

#define  ALLWINNER_LOSC_AUTO_STA_SRC           BIT(0)

#define  ALLWINNER_INTOSC_CLK_PRESCAL         GENMASK(15,  0)

#define  ALLWINNER_YYMMDD_LEAP                 BIT(22)
#define  ALLWINNER_YYMMDD_YEAR                 GENMASK(21,  16)
#define  ALLWINNER_YYMMDD_MONTH                GENMASK(11,  8)
#define  ALLWINNER_YYMMDD_DAY                  GENMASK(4,   0)

#define  ALLWINNER_HHMMSS_WKNO                 GENMASK(31,  29)
#define  ALLWINNER_HHMMSS_HOUR                 GENMASK(20,  16)
#define  ALLWINNER_HHMMSS_MINUTE               GENMASK(13,  8)
#define  ALLWINNER_HHMMSS_SECOND               GENMASK(5,   0)

#define  ALLWINNER_RTC_MAP_SIZE     (0x400u)
#define  ALLWINNER_RTC_INTERNAL_CLK           (16000000u)
#define  ALLWINNER_RTC_EXTERNAL_CLK           (32768u)



#endif
