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
#include <linux/watchdog.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "wdt-allwinner.h"

#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)



typedef  struct {
    uint32_t  irq_en;
    uint32_t  sr;
    uint32_t  ctrl;
    uint32_t  cfg;
    uint32_t  mode;
} __packed  allwinner_wdt_t;


typedef  struct {
    struct  watchdog_device wdt_dev;
    phys_addr_t  phy_addr;
    uint32_t  * map_base;
    uint32_t  offset;
} allwinner_wdt_dev_t;





static  int32_t  allwinner_wdt_start(struct watchdog_device * wdt)
{
    return  0;
}


static  int32_t  allwinner_wdt_stop(struct watchdog_device * dev)
{
    return  0;
}


static  uint32_t  allwinner_wdt_status(struct watchdog_device * dev)
{
    return  0;
}

static  int32_t  allwinner_wdt_set_timeout(struct watchdog_device * dev, uint32_t timeout)
{
    return  0;
}


static  struct watchdog_ops  wdt_ops = {
	.owner = THIS_MODULE,
    .start  =  allwinner_wdt_start,
    .stop  =  allwinner_wdt_stop,
    .status  =  allwinner_wdt_status,
    .set_timeout  =  allwinner_wdt_set_timeout,
};



static int32_t  allwinner_wdt_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct device * dev  =  &pdev->dev;
    struct device_node * dev_node = dev->of_node;

    uint32_t offset =  (uintptr_t)device_get_match_data(dev);
    uint32_t  phy_addr =  0;
    if (of_property_read_u32(dev_node,  "addr",  &phy_addr)) {
        return -EINVAL;
    }

    allwinner_wdt_dev_t * wdt = devm_kzalloc(dev, sizeof(allwinner_wdt_dev_t), GFP_KERNEL);
    if (!wdt) {
        return -ENOMEM;
    }

    memset(wdt,  0,  sizeof(allwinner_wdt_dev_t));

    wdt->map_base  =  devm_ioremap(dev,  phy_addr,  ALLWINNER_WDT_MAP_SIZE);
    wdt->phy_addr  = phy_addr;
    wdt->offset   =  offset;

    wdt->wdt_dev.ops  =  &wdt_ops;
    wdt->wdt_dev.min_timeout  =  ALLWINNER_WDT_MIN_TIMEOUT;
    wdt->wdt_dev.max_timeout  =  ALLWINNER_WDT_MAX_TIMEOUT;


    ret  =  watchdog_register_device(&wdt->wdt_dev);
    if (ret) {
        _PRINTF_ERROR("register wdt device failed! ret = %d\n", ret);
    }

    platform_set_drvdata(pdev,  wdt);

	return   ret;

}


static int32_t allwinner_wdt_remove(struct platform_device * pdev)
{
    allwinner_wdt_dev_t * wdt_dev  =  platform_get_drvdata(pdev);
    watchdog_unregister_device(&wdt_dev->wdt_dev);
	return  0;
}


static struct of_device_id allwinner_wdt_ids[] =  {
	{.compatible = "allwinner,H6-v200-wdt", .data = (void *)ALLWINNER_WDT_REG_OFFSET},
    {}
};


struct  platform_driver  allwinner_wdt_driver = {
    .probe   =  allwinner_wdt_probe,
    .remove	 =  allwinner_wdt_remove,
    .driver  =  {
        .name  =  "allwinner_wdt_driver",
        .of_match_table  =  allwinner_wdt_ids,
    }

};


static int32_t  __init  allwinner_wdt_init(void)
{
    int32_t ret =  0;

    ret = platform_driver_register(&allwinner_wdt_driver);
    if (ret) {
        _PRINTF_ERROR("register wdt driver failed! ret = %d\n", ret);
    } else {
        _PRINTF_INFO("register allwinner wdt driver success!\n");
    }

    return  ret;

}


static void  __exit  allwinner_wdt_exit(void)
{
    platform_driver_unregister(&allwinner_wdt_driver);
    _PRINTF_INFO("allwinner wdt driver remove!\n");

}


module_init(allwinner_wdt_init);
module_exit(allwinner_wdt_exit);

MODULE_DESCRIPTION("allwinner's soc wdt driver");
MODULE_LICENSE("GPL v2");


