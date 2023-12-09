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
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "dbg_log.h"



static void allwinner_pinctrl_dev_release(struct device * dev)
{
    _PRINTF_INFO("release dev -- %s\n", dev_name(dev));
}


static struct platform_device  allwinner_pinctrl_dev =  {
    .name  =  "allwinner_pinctrl_driver",
    .id  =  PLATFORM_DEVID_NONE,
    .num_resources  =  0,
    .dev  =  { 
        .release  = allwinner_pinctrl_dev_release,
    },
};


static int32_t  __init  allwinner_pinctrl_dev_init(void)
{
    int32_t ret =  0;

    struct device * dev  = &allwinner_pinctrl_dev.dev;

    ret = platform_device_register(&allwinner_pinctrl_dev);
    if (ret) {
        _PRINTF_ERROR("register pinctrl dev failed! ret = %d\n", ret);
        return  ret;
    } else {
        _PRINTF_INFO("register allwinner pinctrl dev success!\n");
    }

    // ret = device_add_groups(dev, clk_dev_test_groups);
    if (ret) {
        _PRINTF_ERROR("add device attribute failed! ret = %d\n", ret);
        platform_device_unregister(&allwinner_pinctrl_dev);
    }

    return  ret;

}


static void  __exit  allwinner_pinctrl_dev_exit(void)
{
    platform_device_unregister(&allwinner_pinctrl_dev);
    _PRINTF_INFO("allwinner pinctrl dev remove!\n");

}


module_init(allwinner_pinctrl_dev_init);
module_exit(allwinner_pinctrl_dev_exit);

MODULE_DESCRIPTION("allwinner's soc pinctrl device");
MODULE_LICENSE("GPL v2");




