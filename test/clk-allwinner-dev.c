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
#include <linux/clk-provider.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>
#include <linux/resource.h>

#include "dbg_log.h"

#define  ALLWINNER_CCU_MAP_SIZE        (0x1000u)
#define  ALLWINNER_CCU_BASE_ADDR       (0xc0000u)

static struct resource  dev_res[]  =  {
    {
        .name  = DEVICE_PHY_ADDR_RESOURCE,
        .flags =  IORESOURCE_MEM,
        .start = ALLWINNER_CCU_BASE_ADDR, 
        .end = ALLWINNER_CCU_BASE_ADDR + ALLWINNER_CCU_MAP_SIZE,
    },

};



static void allwinner_clk_dev_release(struct device * dev)
{
    _PRINTF_INFO("release dev -- %s\n", dev_name(dev));
}


static struct platform_device  allwinner_clk_dev =  {
    .name  =  "allwinner_ccu_driver",
    .id  =  PLATFORM_DEVID_NONE,
    .resource  =  dev_res,
    .num_resources  =  ARRAY_SIZE(dev_res),
    .dev  =  { 
        .release  = allwinner_clk_dev_release,
    },
};


static int32_t  __init  allwinner_clk_dev_init(void)
{
    int32_t ret =  0;

    struct device * dev  = &allwinner_clk_dev.dev;

    ret = platform_device_register(&allwinner_clk_dev);
    if (ret) {
        _PRINTF_ERROR("register clk dev failed! ret = %d\n", ret);
    } else {
        _PRINTF_INFO("register allwinner clk dev success!\n");
    }

    return  ret;

}


static void  __exit  allwinner_clk_dev_exit(void)
{
    platform_device_unregister(&allwinner_clk_dev);
    _PRINTF_INFO("allwinner clk dev remove!\n");

}


module_init(allwinner_clk_dev_init);
module_exit(allwinner_clk_dev_exit);

MODULE_DESCRIPTION("allwinner's soc clk device");
MODULE_LICENSE("GPL v2");



