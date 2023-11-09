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


typedef  struct  {
    uint32_t  enable;
    uint32_t  pause;
    uint32_t  addr;
    uint32_t  cfg;
    uint32_t  cur_src;
    uint32_t  cur_dst;
    uint32_t  byte_left;
    uint32_t  param;
    uint32_t  rsv1[2];
    uint32_t  mode;
    uint32_t  former_desc;
    uint32_t  pkg_num;
    uint32_t  rsv2[3];
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

typedef  struct {
    struct dma_chan chan;
    uint32_t  * map_base;
    uint32_t  phy_addr;
} allwinner_dma_chan_t;


typedef struct {
    struct  dma_device dmadev;
    uint32_t  * map_base;
    uint32_t  phy_addr;
    allwinner_dma_chan_t chans[ALLWINNER_DMA_CHANNEL_NUM];
} allwinner_dma_device_t;



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

static inline allwinner_dma_chan_t * to_allwinner_dma_chan(struct dma_chan * chan)
{
	return  container_of(chan, allwinner_dma_chan_t, chan);
}

static int32_t allwinner_dma_alloc_chan_resources(struct dma_chan *chan)
{
	allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

    return  0;
}


static void allwinner_dma_free_chan_resources(struct dma_chan *chan)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);
}

static enum dma_status allwinner_dma_tx_status(struct dma_chan *chan,
					    dma_cookie_t cookie,
					    struct dma_tx_state * txstate)
{
	// return dma_cookie_status(chan, cookie, txstate);
    return  0;
}

static struct dma_async_tx_descriptor * allwinner_dma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		uint32_t sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void * context)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

    return NULL;
}


static struct dma_async_tx_descriptor * allwinner_dma_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t dma_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction,
		unsigned long flags)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

    return NULL;
}

static struct dma_async_tx_descriptor * allwinner_dma_prep_dma_memcpy(
	struct dma_chan *chan, dma_addr_t dest,
	dma_addr_t src, size_t len, unsigned long flags)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

    return NULL;
}

static int32_t allwinner_dma_config(struct dma_chan *chan,
			 struct dma_slave_config *dmaengine_cfg)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

    return  0;
}


static int32_t allwinner_dma_terminate_all(struct dma_chan *chan)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

    return  0;
}


static void allwinner_dma_issue_pending(struct dma_chan *chan)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

}


static int32_t  allwinner_dma_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct device * dev  =  &pdev->dev;
    struct device_node * dev_node = dev->of_node;

    allwinner_dma_device_t * dma_plat  =  devm_kzalloc(dev,  sizeof(allwinner_dma_device_t),
                            GFP_KERNEL);

    if (!dma_plat) {
        return  -ENOMEM;
    }

    uint32_t  phy_addr =  0;
    if (of_property_read_u32(dev_node, "addr",  &phy_addr)) {
        return -EINVAL;
    }

    dma_plat->phy_addr  =  phy_addr;
    dma_plat->map_base  =  devm_ioremap(dev, phy_addr, ALLWINNER_DMA_MAP_SIZE);
    if (!dma_plat->map_base) {
        return -EINVAL;
    }

	dma_cap_set(DMA_SLAVE,  dma_plat->dmadev.cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_plat->dmadev.cap_mask);
	dma_cap_set(DMA_MEMCPY, dma_plat->dmadev.cap_mask);

    INIT_LIST_HEAD(&dma_plat->dmadev.channels);
    for (int32_t  i =  0; i < ALLWINNER_DMA_CHANNEL_NUM; i++) {
        dma_plat->chans[i].map_base  = dma_plat->map_base;
        dma_plat->chans[i].phy_addr  = dma_plat->phy_addr;
        dma_plat->chans[i].chan.device  =  &dma_plat->dmadev;
        list_add_tail(&dma_plat->chans[i].chan.device_node,  &dma_plat->dmadev.channels);
    }


	dma_plat->dmadev.device_alloc_chan_resources = allwinner_dma_alloc_chan_resources;
	dma_plat->dmadev.device_free_chan_resources = allwinner_dma_free_chan_resources;
	dma_plat->dmadev.device_tx_status = allwinner_dma_tx_status;
	dma_plat->dmadev.device_prep_slave_sg = allwinner_dma_prep_slave_sg;
	dma_plat->dmadev.device_prep_dma_cyclic = allwinner_dma_prep_dma_cyclic;
	dma_plat->dmadev.device_prep_dma_memcpy = allwinner_dma_prep_dma_memcpy;
	dma_plat->dmadev.device_config = allwinner_dma_config;
	dma_plat->dmadev.device_terminate_all = allwinner_dma_terminate_all;
	dma_plat->dmadev.device_issue_pending = allwinner_dma_issue_pending;

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
    allwinner_dma_device_t * dma = platform_get_drvdata(pdev);

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


