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

#include "pinctrl-allwinner.h"

#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)


static int32_t  allwinner_pinctrl_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct  device * dev =  &pdev->dev;
    struct  device_node * dev_node  =  dev->of_node;

    uint32_t  phy_addr =  0;
    if (of_property_read_u32(dev_node, "addr", &phy_addr)) {
        _PRINTF_ERROR("read pinctrl base addr failed\n");
        return -EINVAL;
    }

	return   ret;

}


static int32_t allwinner_pinctrl_remove(struct platform_device * pdev)
{
    int32_t  ret  =  0;


	return  ret;
}


static struct of_device_id allwinner_pinctrl_ids[] =  {
	{.compatible = "allwinner,H6-v200-pinctrl"},
    {}
};


struct  platform_driver  allwinner_pinctrl_driver = {
    .probe   =  allwinner_pinctrl_probe,
    .remove	 =  allwinner_pinctrl_remove,
    .driver  =  {
        .name  =  "allwinner_pinctrl_driver",
        .of_match_table  =  allwinner_pinctrl_ids,
    }

};


static int32_t  __init  allwinner_pinctrl_init(void)
{
    int32_t ret =  0;

    ret = platform_driver_register(&allwinner_pinctrl_driver);
    if (ret) {
        _PRINTF_ERROR("register pinctrl driver failed! ret = %d\n", ret);
    } else {
        _PRINTF_INFO("register allwinner pinctrl driver success!\n");
    }

    return  ret;

}


static void  __exit  allwinner_pinctrl_exit(void)
{
    platform_driver_unregister(&allwinner_pinctrl_driver);
    _PRINTF_INFO("allwinner pinctrl driver remove!\n");

}


module_init(allwinner_pinctrl_init);
module_exit(allwinner_pinctrl_exit);

MODULE_DESCRIPTION("allwinner's soc pinctrl driver");
MODULE_LICENSE("GPL v2");


