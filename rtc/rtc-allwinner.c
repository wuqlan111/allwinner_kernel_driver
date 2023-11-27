#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/syscore_ops.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <asm-generic/io.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "rtc-allwinner.h"
#include "dbg_log.h"



typedef struct {
    struct  rtc_device * rtcdev;
    uint32_t  * map_base;    
    uint32_t  phy_addr;
    uint32_t  year_base;
} allwinner_rtc_plat_t;

typedef  struct  {
    uint32_t  losc_ctrl;
    uint32_t  losc_auto_swt;
    uint32_t  intosc_clk_prescal;
    uint32_t  intosc_clk_cali;
    uint32_t  yy_mm_dd;
    uint32_t  hh_mm_ss;
    uint32_t  rsv1[2];
    uint32_t  alarm0_cnt;
    uint32_t  alarm0_cur;
    uint32_t  alarm0_enable;
    uint32_t  alarm0_irq_enable;
    uint32_t  alarm0_irq_sta;
    uint32_t  rsv2[3];
    uint32_t  alarm1_wk;
    uint32_t  alarm1_enable;
    uint32_t  alarm1_irq_enable;
    uint32_t  alarm1_irq_sta;
} __packed allwinner_h6_rtc_t;


static  void  allwinner_rtc_get_src_rate(allwinner_h6_rtc_t * regs, uint64_t * rate)
{
    uint32_t  losc_sr  =  readl_relaxed(&regs->losc_auto_swt);
    uint32_t  presalar  =  readl_relaxed(&regs->intosc_clk_prescal);
    const uint32_t is_extern  =  losc_sr & ALLWINNER_LOSC_AUTO_STA_SRC? 1: 0;

    if (is_extern) {
        *rate  =  ALLWINNER_RTC_EXTERNAL_CLK;
    } else {
        uint32_t  div  =  presalar << 5;
        *rate  =  ALLWINNER_RTC_INTERNAL_CLK / div;
    }

}




static	int32_t  allwinner_rtc_read_time(struct device * dev, struct rtc_time * tm)
{
    struct platform_device * pdev  =  container_of(dev, struct platform_device, dev);
    allwinner_rtc_plat_t * rtc = platform_get_drvdata(pdev);

    allwinner_h6_rtc_t * regs = (allwinner_h6_rtc_t * )rtc->map_base;

    if (!regs) {
        return -EINVAL;
    }

    uint32_t  rtc_yy  =  readl_relaxed(&regs->yy_mm_dd);
    uint32_t  rtc_hh  =  readl_relaxed(&regs->hh_mm_ss);

    tm->tm_year  =  ((rtc_yy & ALLWINNER_YYMMDD_YEAR) >> 16) + rtc->year_base - 1900;
    tm->tm_mon   =  (rtc_yy & ALLWINNER_YYMMDD_MONTH) >> 8;
    tm->tm_mday  =  rtc_yy &  ALLWINNER_YYMMDD_DAY;
    tm->tm_hour  =  (rtc_hh &  ALLWINNER_HHMMSS_HOUR) >> 16;
    tm->tm_min   =  (rtc_hh &  ALLWINNER_HHMMSS_MINUTE) >> 8;
    tm->tm_sec   =  rtc_hh &  ALLWINNER_HHMMSS_SECOND;
    tm->tm_wday  =  ( rtc_hh & ALLWINNER_HHMMSS_WKNO ) >> 29;

    return  0;

}

static  int32_t allwinner_rtc_set_time(struct device * dev, struct rtc_time * tm)
{
    struct platform_device * pdev  =  container_of(dev, struct platform_device, dev);
    allwinner_rtc_plat_t * rtc = platform_get_drvdata(pdev);

    allwinner_h6_rtc_t * regs = (allwinner_h6_rtc_t * )rtc->map_base;

    if (!regs) {
        return -EINVAL;
    }

    uint32_t  year  =  tm->tm_year + 1900;
    rtc->year_base  =  year & ~0x3f;
    uint32_t  year_res  =   year & 0x3f;

    uint32_t  is_leap  =  is_leap_year(year)? 1: 0;


    uint32_t  rtc_yy  =  (is_leap << 22) | (year_res << 16) | (tm->tm_mon << 8) | tm->tm_mday;
    uint32_t  rtc_hh  =  (tm->tm_wday << 29) | (tm->tm_hour << 16) | (tm->tm_min << 8) 
                            | tm->tm_sec;

    writel_relaxed(rtc_yy, &regs->yy_mm_dd);
    writel_relaxed(rtc_hh, &regs->hh_mm_ss);

    return  0;

}


static 	int32_t  allwinner_rtc_read_alarm(struct device * dev, struct rtc_wkalrm * alarm)
{
    struct platform_device * pdev  =  container_of(dev, struct platform_device, dev);
    allwinner_rtc_plat_t * rtc = platform_get_drvdata(pdev);

    allwinner_h6_rtc_t * regs = (allwinner_h6_rtc_t * )rtc->map_base;

    uint32_t  rtc_enable = readl_relaxed(&regs->alarm1_enable) ? 1: 0;
    if (rtc_enable ) {
        uint32_t  hh_mm_ss   =  readl_relaxed(&regs->alarm1_wk);

        alarm->enabled =  1;
        alarm->time.tm_sec = hh_mm_ss & ALLWINNER_HHMMSS_SECOND;
		alarm->time.tm_min =  (hh_mm_ss & ALLWINNER_HHMMSS_MINUTE) >> 8;
		alarm->time.tm_hour =  (hh_mm_ss & ALLWINNER_HHMMSS_HOUR) >> 16;
    }

    return  0;
}



static  int32_t  allwinner_rtc_set_alarm(struct device * dev, struct rtc_wkalrm * alarm)
{

    struct platform_device * pdev  =  container_of(dev, struct platform_device, dev);
    allwinner_rtc_plat_t * rtc = platform_get_drvdata(pdev);

    allwinner_h6_rtc_t * regs = (allwinner_h6_rtc_t * )rtc->map_base;



    return  0;
}


static int32_t allwinner_rtc_alarm_irq_enable(struct device * dev, uint32_t enabled)
{
    struct platform_device * pdev  =  container_of(dev, struct platform_device, dev);
    allwinner_rtc_plat_t * rtc = platform_get_drvdata(pdev);

    allwinner_h6_rtc_t * regs = (allwinner_h6_rtc_t * )rtc->map_base;

    writel_relaxed(ALLWINNER_RTC_ALARM_IRQ_EN,  &regs->alarm1_irq_enable);

    return  0;

}


static struct rtc_class_ops allwinner_rtc_ops  =  {
    .read_time  =  allwinner_rtc_read_time,
    .set_time   =  allwinner_rtc_set_time,
    .read_alarm  =  allwinner_rtc_read_alarm,
    .set_alarm   =  allwinner_rtc_set_alarm,
};



static int32_t  allwinner_rtc_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct device * dev  =  &pdev->dev;
    struct device_node * dev_node = dev->of_node;
 
    allwinner_rtc_plat_t * rtc_plat  =  devm_kzalloc(dev,  sizeof(allwinner_rtc_plat_t),
                            GFP_KERNEL);

    if (!rtc_plat) {
        return  -ENOMEM;
    }

    uint32_t  phy_addr =  0;
    if (of_property_read_u32(dev_node, "addr",  &phy_addr)) {
        return -EINVAL;
    }

    rtc_plat->phy_addr  =  phy_addr;
    rtc_plat->map_base  =  devm_ioremap(dev, phy_addr, ALLWINNER_RTC_MAP_SIZE);
    if (!rtc_plat->map_base) {
        return -EINVAL;
    }

    rtc_plat->rtcdev  =  devm_rtc_device_register( dev,  "allwinner-rtc",   &allwinner_rtc_ops, 
                            THIS_MODULE);
    if ( IS_ERR(rtc_plat->rtcdev) ) {
        ret  =  PTR_ERR(rtc_plat->rtcdev);
        _PRINTF_ERROR("register rtc dev failed! ret = %ld\n", PTR_ERR(rtc_plat->rtcdev));
    } else {
        _PRINTF_DBG("register rtc dev successful!\n");
    }

    platform_set_drvdata(pdev, rtc_plat);

	return   ret;

}


static int32_t allwinner_rtc_remove(struct platform_device * pdev)
{
    // struct  device * dev =  &pdev->dev;
    // allwinner_rtc_plat_t * rtc = platform_get_drvdata(pdev);

    // rtc_device_unregister(rtc->rtcdev);

	return  0;
}


static struct of_device_id allwinner_rtc_ids[] =  {
	{.compatible = "allwinner,H6-v200-rtc"},
    {}
};


struct  platform_driver  allwinner_rtc_driver = {
    .probe   =  allwinner_rtc_probe,
    .remove	 =  allwinner_rtc_remove,
    .driver  =  {
        .name  =  "allwinner_rtc_driver",
        .of_match_table  =  allwinner_rtc_ids,
    }

};


static int32_t  __init  allwinner_rtc_init(void)
{
    int32_t ret =  0;

    ret = platform_driver_register(&allwinner_rtc_driver);
    if (ret) {
        _PRINTF_ERROR("register rtc driver failed! ret = %d\n", ret);
    } else {
        _PRINTF_INFO("register allwinner rtc driver success!\n");
    }

    return  ret;

}


static void  __exit  allwinner_rtc_exit(void)
{
    platform_driver_unregister(&allwinner_rtc_driver);
    _PRINTF_INFO("allwinner rtc driver remove!\n");

}


module_init(allwinner_rtc_init);
module_exit(allwinner_rtc_exit);

MODULE_DESCRIPTION("allwinner's soc rtc driver");
MODULE_LICENSE("GPL v2");


