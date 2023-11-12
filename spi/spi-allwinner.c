#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
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
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "spi-allwinner.h"

#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)







static int32_t  allwinner_spi_probe(struct platform_device * pdev)
{


    return  0;
}


static int32_t allwinner_spi_remove(struct platform_device * pdev)
{

	return  0;
}




static struct of_device_id allwinner_spi_ids[] =  {
	{.compatible = "allwinner,H6-v200-spi"},
    {}
};


struct  platform_driver  allwinner_spi_driver = {
    .probe   =  allwinner_spi_probe,
    .remove	 =  allwinner_spi_remove,
    .driver  =  {
        .name  =  "allwinner_spi_driver",
        .of_match_table  =  allwinner_spi_ids,
    }

};


static int32_t  __init  allwinner_spi_init(void)
{
    int32_t ret =  0;

    ret = platform_driver_register(&allwinner_spi_driver);
    if (ret) {
        _PRINTF_ERROR("register spi driver failed! ret = %d\n", ret);
    } else {
        _PRINTF_INFO("register allwinner spi driver success!\n");
    }

    return  ret;

}


static void  __exit  allwinner_spi_exit(void)
{
    platform_driver_unregister(&allwinner_spi_driver);
    _PRINTF_INFO("allwinner spi driver remove!\n");

}


module_init(allwinner_spi_init);
module_exit(allwinner_spi_exit);

MODULE_DESCRIPTION("allwinner's soc spi driver");
MODULE_LICENSE("GPL v2");


