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
#include <linux/clk.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>
#include <linux/resource.h>

#include "dbg_log.h"

#define  ALLWINNER_CCU_MAP_SIZE        (0x1000u)
#define  ALLWINNER_CCU_BASE_ADDR       (0xc0000u)


static uint32_t  request_clk_id = 0;


static  void _dbg_printf_clk(struct clk * clk)
{
    if (!clk) {
        return;
    }

    _PRINTF_INFO("clk_name:\t%s\n", __clk_get_name(clk));
    _PRINTF_INFO("parent:\t%s\n", __clk_get_name(clk_get_parent(clk)));
    _PRINTF_INFO("rate:\t%ld\n", clk_get_rate(clk));
}





static ssize_t request_clk_id_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
    struct of_phandle_args  clk_get = {
        .np = NULL,
        .args_count = 1,
        .args = { request_clk_id },
    };

    struct clk * target_clk =  of_clk_get_from_provider(&clk_get);

    if (!target_clk) {
        _PRINTF_ERROR("get target clk by id [%u] failed\n", request_clk_id);
        return  0;
    }

    _dbg_printf_clk(target_clk);

    return  sprintf(buf, "%12s%12s%16s\n%12s%12s%16ld", "name", "parent", "rate",
       __clk_get_name(target_clk), __clk_get_name(clk_get_parent(target_clk)),  
       clk_get_rate(target_clk));

}



static ssize_t request_clk_id_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
    ssize_t rc = 0;
	rc = kstrtou32(buf, 0, &request_clk_id);
	if (rc) {
        _PRINTF_ERROR("request clk id invalid! rc=%ld\n", rc);
    } else {
        rc = count;
    }
	
    return rc;

}


static DEVICE_ATTR_RW(request_clk_id);




static umode_t clk_dev_test_is_visible(struct kobject *kobj,
				   struct attribute *attr, int32_t index)
{
	return attr->mode;
}

static struct attribute * clk_dev_test_attributes[] = {
	&dev_attr_request_clk_id.attr,
	NULL
};

static const struct attribute_group clk_dev_test_group = {
	.attrs = clk_dev_test_attributes,
	.is_visible = clk_dev_test_is_visible,
};

const static struct attribute_group * clk_dev_test_groups[] = {
    &clk_dev_test_group,
};


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
        return  ret;
    } else {
        _PRINTF_INFO("register allwinner clk dev success!\n");
    }

    ret = device_add_groups(dev, clk_dev_test_groups);
    if (ret) {
        _PRINTF_ERROR("add device attribute failed!\n");
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



