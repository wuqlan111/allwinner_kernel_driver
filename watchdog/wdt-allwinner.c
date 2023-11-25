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
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "wdt-allwinner.h"
#include "dbg_log.h"



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

static allwinner_wdt_dev_t * allwinner_wdt_dev_get(struct watchdog_device * wdt)
{
    return  container_of(wdt, allwinner_wdt_dev_t, wdt_dev );
}

static  int32_t  allwinner_wdt_start(struct watchdog_device * wdt)
{
    allwinner_wdt_dev_t * wdt_dev = allwinner_wdt_dev_get(wdt);
    allwinner_wdt_t * regs = (allwinner_wdt_t *)(&wdt_dev->map_base[wdt_dev->offset >> 2]);

    writel_relaxed(0x1, &regs->cfg);
    uint32_t  flag  =  readl_relaxed(&regs->mode);
    flag |= ALLWINNER_WDT_MODE_EN;
    writel_relaxed(flag,  &regs->mode);

    set_bit(WDOG_HW_RUNNING, &wdt_dev->wdt_dev.status);

    return  0;
}

static  int32_t  allwinner_wdt_ping(struct watchdog_device * wdt)
{
    allwinner_wdt_dev_t * wdt_dev = allwinner_wdt_dev_get(wdt);
    allwinner_wdt_t * regs = (allwinner_wdt_t *)(&wdt_dev->map_base[wdt_dev->offset >> 2]);

    uint32_t flag  =  ALLWINNER_WDT_CTRL_KEY_FLAG | ALLWINNER_WDT_CTRL_RESTART;
    writel_relaxed(flag,  &regs->ctrl);

    return  0;

}


// static  uint32_t  allwinner_wdt_status(struct watchdog_device * wdt)
// {
//     return  0;
// }

static  int32_t  allwinner_wdt_set_timeout(struct watchdog_device * wdt, uint32_t timeout)
{
    allwinner_wdt_dev_t * wdt_dev = allwinner_wdt_dev_get(wdt);
    allwinner_wdt_t * regs = (allwinner_wdt_t *)(&wdt_dev->map_base[wdt_dev->offset >> 2]);

    uint32_t  inv_value = 0;

    if (timeout <= 6) {
        inv_value  =  timeout;
    } else {
        uint32_t delta  =  (timeout - 6) >> 1;
        inv_value  =  6 + delta + (delta & 0x1);
    }
	
    uint32_t  flag  =  inv_value << 4;
    writel_relaxed(flag,  &regs->mode);

    return  0;
}


static  struct watchdog_ops  wdt_ops = {
	.owner = THIS_MODULE,
    .start  =  allwinner_wdt_start,
    .ping   =  allwinner_wdt_ping,
    // .status  =  allwinner_wdt_status,
    .set_timeout  =  allwinner_wdt_set_timeout,
};


static const struct watchdog_info allwinner_wdt_info = {
	.identity = "allwinner watchdog",
	.options = WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT,
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
    wdt->wdt_dev.info  = &allwinner_wdt_info;
    wdt->wdt_dev.min_timeout  =  ALLWINNER_WDT_MIN_TIMEOUT;
    wdt->wdt_dev.max_timeout  =  ALLWINNER_WDT_MAX_TIMEOUT;
    wdt->wdt_dev.parent  =  &pdev->dev;

    watchdog_stop_on_unregister(&wdt->wdt_dev);

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


