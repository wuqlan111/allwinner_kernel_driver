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
    uint32_t  enable;
    uint32_t  pause;
    uint32_t  addr;
    uint32_t  cfg;
    uint32_t  cur_src;
    uint32_t  cur_dst;
    uint32_t  byte_left;
    uint32_t  param;
    uint32_t  rsv[2];
    uint32_t  mode;
    uint32_t  former_desc;
    uint32_t  pkg_num;
    uint32_t  rsv[3];
} __packed allwinner_h6_dma_channel_t;


typedef  struct  {
    uint32_t  irq_en0;
    uint32_t  irq_en1;
    uint32_t  rsv1[2];
    uint32_t  irq_pend0;
    uint32_t  irq_pend1;
    uint32_t  rsv2[2];
    uint32_t  sec;
    uint32_t  rsv3;
    uint32_t  auto_gate;
    uint32_t  rsv4;
    uint32_t  sta;
    uint32_t  rsv5[51];
    allwinner_h6_dma_channel_t  channel_cfg[ALLWINNER_DMA_CHANNEL_NUM];
} __packed allwinner_h6_dma_t;


typedef  struct {
    uint32_t  cfg;
    uint32_t  src;
    uint32_t  dst;
    uint32_t  bytes;
    uint32_t  param;
    uint32_t  link;
} __packed allwinner_h6_dma_desc_t;


typedef struct {
    uint16_t  port;
    uint16_t  drq;
} allwinner_dma_drq_t;

#define  DMA_DRQ_ENTRY(port_id, drq_type)  {  \
    .port  =  port_id,                  \
    .drq   =  drq_type,               \
}

#define  DMA_DRQ_TABLE              \
    DMA_DRQ_ENTRY(0,  ALLWINNER_DMA_DRQ_SRAM),      \
    DMA_DRQ_ENTRY(1,  ALLWINNER_DMA_DRQ_DRAM),      \
    DMA_DRQ_ENTRY(2,  ALLWINNER_DMA_DRQ_OWA),      \
    DMA_DRQ_ENTRY(3,  ALLWINNER_DMA_DRQ_I2S_PCM0),      \
    DMA_DRQ_ENTRY(4,  ALLWINNER_DMA_DRQ_I2S_PCM1),      \
    DMA_DRQ_ENTRY(5,  ALLWINNER_DMA_DRQ_I2S_PCM2),      \
    DMA_DRQ_ENTRY(6,  ALLWINNER_DMA_DRQ_I2S_PCM3),      \
    DMA_DRQ_ENTRY(7,  ALLWINNER_DMA_DRQ_DMIC),      \
    DMA_DRQ_ENTRY(9,  ALLWINNER_DMA_DRQ_CE),      \
    DMA_DRQ_ENTRY(10,  ALLWINNER_DMA_DRQ_NAND0),      \
    DMA_DRQ_ENTRY(14,  ALLWINNER_DMA_DRQ_UART0),      \
    DMA_DRQ_ENTRY(15,  ALLWINNER_DMA_DRQ_UART1),      \
    DMA_DRQ_ENTRY(16,  ALLWINNER_DMA_DRQ_UART2),      \
    DMA_DRQ_ENTRY(17,  ALLWINNER_DMA_DRQ_UART3),      \
    DMA_DRQ_ENTRY(22,  ALLWINNER_DMA_DRQ_SPI0),      \
    DMA_DRQ_ENTRY(23,  ALLWINNER_DMA_DRQ_SPI1),      \
    DMA_DRQ_ENTRY(30,  ALLWINNER_DMA_DRQ_OTG_EP1),      \
    DMA_DRQ_ENTRY(31,  ALLWINNER_DMA_DRQ_OTG_EP2),      \
    DMA_DRQ_ENTRY(32,  ALLWINNER_DMA_DRQ_OTG_EP3),      \
    DMA_DRQ_ENTRY(33,  ALLWINNER_DMA_DRQ_OTG_EP4),      \
    DMA_DRQ_ENTRY(43,  ALLWINNER_DMA_DRQ_AUDIO_HUB1),      \
    DMA_DRQ_ENTRY(44,  ALLWINNER_DMA_DRQ_AUDIO_HUB2),      \
    DMA_DRQ_ENTRY(45,  ALLWINNER_DMA_DRQ_AUDIO_HUB3),


static  allwinner_dma_drq_t port_drq[]  =  { DMA_DRQ_TABLE };

static  int32_t  allwinner_drq_2_port(const uint32_t drq, uint32_t * port)
{
    if (!port  || (drq > ALLWINNER_DMA_DRQ_MAX)) {
        return -EINVAL;
    }

    *port  =  0;
    for (int32_t i  =  0; i < ARRAY_SIZE(port_drq); i++) {
        if (port_drq[i].drq == drq) {
            *port  =  port_drq[i].port;
            return  0;
        }
    }

    _PRINTF_ERROR("can't get port_id by drq! drq =  %u\n", drq);
    return  -EINVAL;

}



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


