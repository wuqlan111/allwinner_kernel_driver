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
#include <linux/dmaengine.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "dma-allwinner.h"

#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)


typedef struct {
    struct  dma_device dmadev;
    uint32_t  * map_base;    
    uint32_t  phy_addr;
} allwinner_dma_plat_t;

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
} __packed allwinner_h6_dma_t;




static int32_t  allwinner_dma_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct device * dev  =  &pdev->dev;
    struct device_node * dev_node = dev->of_node;

    allwinner_dma_plat_t * dma_plat  =  devm_kzalloc(dev,  sizeof(allwinner_dma_plat_t),
                            GFP_KERNEL);

    if (!dma_plat) {
        return  -ENOMEM;
    }

    uint32_t  phy_addr =  0;
    if (of_property_read_u32(dev_node, "addr",  &phy_addr)) {
        return -EINVAL;
    }

    dma_plat->phy_addr  =  phy_addr;
    dma_plat->map_base  =  devm_ioremap(dev, phy_addr, ALLWINNER_RTC_MAP_SIZE);
    if (!dma_plat->map_base) {
        return -EINVAL;
    }

    dma_plat->dmadev.dev  = dev;
    ret  =  dmaenginem_async_device_register(&dma_plat->dmadev);

    if (ret) {
        _PRINTF_ERROR("dma register device failed! ret = %d\n",  ret);
    } else {
        _PRINTF_DBG("dma register device successful\n");
    }

    platform_set_drvdata(pdev, dma_plat);

	return   ret;

}


static int32_t allwinner_dma_remove(struct platform_device * pdev)
{
    allwinner_dma_plat_t * dma = platform_get_drvdata(pdev);

    dmaenginem_async_device_register(&dma->dmadev);

	return  0;
}


static struct of_device_id allwinner_dma_ids[] =  {
	{.compatible = "allwinner,H6-v200-dma"},
    {}
};


struct  platform_driver  allwinner_dma_driver = {
    .probe   =  allwinner_dma_probe,
    .remove	 =  allwinner_dma_remove,
    .driver  =  {
        .name  =  "allwinner_dma_driver",
        .of_match_table  =  allwinner_dma_ids,
    }

};


static int32_t  __init  allwinner_dma_init(void)
{
    int32_t ret =  0;

    ret = platform_driver_register(&allwinner_dma_driver);
    if (ret) {
        _PRINTF_ERROR("register dma driver failed! ret = %d\n", ret);
    } else {
        _PRINTF_INFO("register allwinner dma driver success!\n");
    }

    return  ret;

}


static void  __exit  allwinner_dma_exit(void)
{
    platform_driver_unregister(&allwinner_dma_driver);
    _PRINTF_INFO("allwinner dma driver remove!\n");

}


module_init(allwinner_dma_init);
module_exit(allwinner_dma_exit);

MODULE_DESCRIPTION("allwinner's soc dma driver");
MODULE_LICENSE("GPL v2");


