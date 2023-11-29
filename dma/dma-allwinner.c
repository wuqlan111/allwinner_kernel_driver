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
#include <linux/dmapool.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/of_dma.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "dma-allwinner.h"
#include "dbg_log.h"


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
} __packed allwinner_dma_phy_desc_t;

typedef  struct {
    allwinner_dma_phy_desc_t phy_desc;
    struct  list_head  node;
    enum  dma_status  status;
    struct dma_async_tx_descriptor desc;
    uint32_t  is_cyclic;
} __aligned(4) allwinner_dma_desc_t;

typedef  struct {
    struct dma_chan chan;
    // uint32_t  * map_base;
    // uint32_t  phy_addr;
    struct tasklet_struct tasklet;	
    spinlock_t  lock;
    struct dma_slave_config	config;
    struct list_head  free_desc;
    struct list_head  submit_desc;
    struct list_head  xfer_desc;
    struct list_head  one_desc;
    struct list_head  cyclic_desc;
    uint32_t  allocated;
} allwinner_dma_chan_t;


typedef struct {
    struct  dma_device dmadev;
    uint32_t  * map_base;
    uint32_t  phy_addr;
    struct dma_pool * desc_pool;
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

static inline allwinner_dma_device_t * to_allwinner_dma_dev(struct dma_device * dev)
{
	return  container_of(dev, allwinner_dma_device_t, dmadev);
}


static dma_cookie_t allwinner_dma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	allwinner_dma_chan_t * dma_chan = to_allwinner_dma_chan(tx->chan);
	allwinner_dma_device_t * allwinner_dma  = container_of(dma_chan->chan.device, 
                            allwinner_dma_device_t,   dmadev);
    allwinner_dma_desc_t * dma_desc = container_of(tx, allwinner_dma_desc_t, desc);
	dma_cookie_t  cookie  =  0;
	unsigned  long flags  =  0;

	spin_lock_irqsave(&dma_chan->lock, flags);
    if (dma_desc->is_cyclic) {
        list_move_tail(&dma_desc->node, &dma_chan->cyclic_desc);
    } else {
        list_move_tail(&dma_desc->node, &dma_chan->one_desc);
    }
	cookie = ++dma_chan->chan.cookie;
	spin_unlock_irqrestore(&dma_chan->lock, flags);

	return  cookie;
}


static int32_t allwinner_dma_alloc_chan_resources(struct dma_chan *chan)
{
	allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);
    allwinner_dma_device_t * dma_dev = to_allwinner_dma_dev(chan->device);

    while (allwinner_chan->allocated < ALLWINNER_DMA_MAX_CHAN_DESCRIPTORS) {
        dma_addr_t  phy_addr = 0;
		allwinner_dma_desc_t * desc = dma_pool_zalloc(dma_dev->desc_pool, 
                                        GFP_ATOMIC, &phy_addr);
		if (!desc) {
			break;            
        }

		dma_async_tx_descriptor_init(&desc->desc, chan);
		desc->desc.tx_submit = allwinner_dma_tx_submit;

		desc->desc.flags = DMA_CTRL_ACK;
		desc->status = DMA_COMPLETE;

		list_add_tail(&desc->node, &allwinner_chan->free_desc);
		allwinner_chan->allocated++;
	}

    return   0;
}


static void allwinner_dma_free_chan_resources(struct dma_chan *chan)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);
    allwinner_dma_device_t * dma_dev = to_allwinner_dma_dev(chan->device);

    struct list_head  to_free = {0};
    INIT_LIST_HEAD(&to_free);

    unsigned long flags =  0;
    spin_lock_irqsave(&allwinner_chan->lock, flags);

	list_splice_tail_init(&allwinner_chan->free_desc,  &to_free);

	spin_unlock_irqrestore(&allwinner_chan->lock, flags);

    allwinner_dma_desc_t *desc, *_desc;
    list_for_each_entry_safe(desc, _desc, &to_free, node) {
		dma_pool_free(dma_dev->desc_pool, desc, desc->desc.phys);
		allwinner_chan->allocated--;
	}

}

static enum dma_status allwinner_dma_tx_status(struct dma_chan *chan,
					    dma_cookie_t cookie,  struct dma_tx_state * txstate)
{
    enum  dma_status ret  =  DMA_ERROR;
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);
    allwinner_dma_device_t * dma_dev = to_allwinner_dma_dev(chan->device);
    allwinner_h6_dma_t * regs = (allwinner_h6_dma_t *)dma_dev->map_base;
    const uint32_t chan_id  =  chan->chan_id;
    allwinner_dma_desc_t * desc = NULL;

	if ( !txstate) {
		return ret;        
    }

    unsigned long flags =  0;
	spin_lock_irqsave(&allwinner_chan->lock, flags);
    desc = list_first_entry(&allwinner_chan->xfer_desc, allwinner_dma_desc_t, node);

    // if (desc->desc.cookie > cookie)
    uint32_t  residue =  readl_relaxed(&regs->channel_cfg[chan_id].byte_left);
    spin_unlock_irqrestore(&allwinner_chan->lock, flags);

    txstate->residue = residue;

    return  ret;
}

static struct dma_async_tx_descriptor * allwinner_dma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		uint32_t sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void * context)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

    if (list_empty(&allwinner_chan->free_desc)) {
        _PRINTF_ERROR("no idle allwinner dma desc!\n");
        return  NULL;
    }

    allwinner_dma_desc_t  * dma_desc = list_first_entry(&allwinner_chan->free_desc, 
                                            allwinner_dma_desc_t, node);

    return  NULL;
}


static struct dma_async_tx_descriptor * allwinner_dma_prep_dma_cyclic(
		struct dma_chan *chan, dma_addr_t dma_addr, size_t buf_len,
		size_t period_len, enum dma_transfer_direction direction,
		unsigned long flags)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

    if (list_empty(&allwinner_chan->free_desc)) {
        _PRINTF_ERROR("no idle allwinner dma desc!\n");
        return  NULL;
    }

    allwinner_dma_desc_t  * dma_desc = list_first_entry(&allwinner_chan->free_desc,
                                        allwinner_dma_desc_t, node);

    return NULL;
}

static struct dma_async_tx_descriptor * allwinner_dma_prep_dma_memcpy(
	struct dma_chan *chan, dma_addr_t dest,
	dma_addr_t src, size_t len, unsigned long flags)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

    if (list_empty(&allwinner_chan->free_desc)) {
        _PRINTF_ERROR("no idle allwinner dma desc!\n");
        return  NULL;
    }

    allwinner_dma_desc_t  * dma_desc = list_first_entry(&allwinner_chan->free_desc,  
                                allwinner_dma_desc_t, node);

    return NULL;
}


static int32_t allwinner_check_slave_config(struct dma_slave_config *sconfig)
{
    int32_t  ret =  0;
	if (sconfig->direction == DMA_TRANS_NONE) {
        return  -EINVAL;
    }

    if ( !ALLWINNER_BUS_WIDTH_VALID(sconfig->src_addr_width) ||  
                    !ALLWINNER_BUS_WIDTH_VALID(sconfig->dst_addr_width)) {
        ret  =  -EINVAL;
    }

	return 0;
}

static int32_t allwinner_dma_config(struct dma_chan *chan,
			 struct dma_slave_config *dmaengine_cfg)
{
    int32_t  ret  =  0;
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

    ret = allwinner_check_slave_config(dmaengine_cfg);
    if (ret) {
        return  ret;
    }

    memcpy(&allwinner_chan->config, dmaengine_cfg, sizeof(struct dma_slave_config));

    return  0;
}


static int32_t allwinner_dma_device_pause(struct dma_chan *chan)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);
    allwinner_dma_device_t * allwinner_dma  = container_of(allwinner_chan->chan.device, 
                            allwinner_dma_device_t,   dmadev);
    allwinner_h6_dma_t * regs  =  (allwinner_h6_dma_t *)allwinner_dma->map_base;

    const uint32_t chan_id = chan->chan_id;

    unsigned  long flags = 0;
	spin_lock_irqsave(&allwinner_chan->lock, flags);
    writel_relaxed(ALLWINNER_DMA_PAUSE_TRANSFER,  &regs->channel_cfg[chan_id].pause);
	spin_unlock_irqrestore(&allwinner_chan->lock, flags);

	return  0;
}

static int32_t allwinner_dma_device_resume(struct dma_chan *chan)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);
    allwinner_dma_device_t * allwinner_dma  = container_of(allwinner_chan->chan.device, 
                            allwinner_dma_device_t,   dmadev);
    allwinner_h6_dma_t * regs  =  (allwinner_h6_dma_t *)allwinner_dma->map_base;

    const uint32_t chan_id = chan->chan_id;

    unsigned  long flags = 0;
	spin_lock_irqsave(&allwinner_chan->lock, flags);
    writel_relaxed(0,  &regs->channel_cfg[chan_id].pause);
	spin_unlock_irqrestore(&allwinner_chan->lock, flags);

	return  0;

}


static int32_t allwinner_dma_terminate_all(struct dma_chan *chan)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);
    allwinner_dma_device_t * allwinner_dma  = container_of(allwinner_chan->chan.device, 
                            allwinner_dma_device_t,   dmadev);
    allwinner_h6_dma_t * regs  =  (allwinner_h6_dma_t *)allwinner_dma->map_base;

    const uint32_t chan_id  =  chan->chan_id;

    unsigned  long flags  =  0;
 	spin_lock_irqsave(&allwinner_chan->lock, flags);
    writel_relaxed(0,  &regs->channel_cfg[chan_id].enable);    
	list_splice_tail_init(&allwinner_chan->one_desc, &allwinner_chan->free_desc);
	list_splice_tail_init(&allwinner_chan->cyclic_desc, &allwinner_chan->free_desc);
	list_splice_tail_init(&allwinner_chan->submit_desc, &allwinner_chan->free_desc);
	spin_unlock_irqrestore(&allwinner_chan->lock, flags);

    return  0;
}


static void allwinner_dma_issue_pending(struct dma_chan *chan)
{
    allwinner_dma_chan_t * allwinner_chan = to_allwinner_dma_chan(chan);

}

static void allwinner_dma_tasklet(unsigned long data)
{
	allwinner_dma_chan_t * dma_chan = (allwinner_dma_chan_t *)data;
}

typedef struct {
	allwinner_dma_device_t * dma_dev;
	int32_t  chan_id;
} allwinner_dma_filter_data_t;


static bool allwinner_dma_filter_fn(struct dma_chan *chan, void *param)
{
	allwinner_dma_filter_data_t * fdata = param;
	allwinner_dma_chan_t * dma_chan = to_allwinner_dma_chan(chan);
	
    chan->private = NULL;

    uint32_t  find  =  (chan->device->dev != fdata->dma_dev->dmadev.dev) || 
                    (fdata->chan_id != chan->chan_id) ? 0: 1;

	return  find;
}

static struct dma_chan * allwinner_dma_xlate(struct of_phandle_args *dma_spec,
						struct of_dma *ofdma)
{
	int32_t count = dma_spec->args_count;
	allwinner_dma_device_t * dma_dev = ofdma->of_dma_data;
	allwinner_dma_filter_data_t fdata = {0};

	if (count != 1) {
        _PRINTF_ERROR("dma of node invalid!\n");
		return  NULL;        
    }

	fdata.dma_dev =  dma_dev;
	fdata.chan_id =  dma_spec->args[0];

	return  dma_request_channel(dma_dev->dmadev.cap_mask,
					 allwinner_dma_filter_fn, &fdata);
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

    dma_plat->desc_pool  = dma_pool_create("allwinner_dma_desc", dev, 
                                sizeof(allwinner_dma_desc_t), 4, 0 );
    if (!dma_plat->desc_pool ) {
        _PRINTF_ERROR("allocate dma pool failed!\n");
        return  -1;
    }

	dma_cap_set(DMA_SLAVE,  dma_plat->dmadev.cap_mask);
	dma_cap_set(DMA_CYCLIC, dma_plat->dmadev.cap_mask);
	dma_cap_set(DMA_MEMCPY, dma_plat->dmadev.cap_mask);
    // dma_cap_set(DMA_PRIVATE, atxdmac->dma.cap_mask);

    INIT_LIST_HEAD(&dma_plat->dmadev.channels);
    for (int32_t  i =  0; i < ALLWINNER_DMA_CHANNEL_NUM; i++) {
        dma_plat->chans[i].chan.device  =  &dma_plat->dmadev;
        dma_plat->chans[i].allocated  =  0;
        list_add_tail(&dma_plat->chans[i].chan.device_node,  &dma_plat->dmadev.channels);
        INIT_LIST_HEAD(&dma_plat->chans[i].free_desc);
        INIT_LIST_HEAD(&dma_plat->chans[i].cyclic_desc);
        INIT_LIST_HEAD(&dma_plat->chans[i].one_desc);

        tasklet_init(&dma_plat->chans[i].tasklet, allwinner_dma_tasklet,
                            (unsigned long)&dma_plat->chans[i]);
    }

	dma_plat->dmadev.device_alloc_chan_resources = allwinner_dma_alloc_chan_resources;
	dma_plat->dmadev.device_free_chan_resources = allwinner_dma_free_chan_resources;
	dma_plat->dmadev.device_tx_status = allwinner_dma_tx_status;
	dma_plat->dmadev.device_prep_slave_sg = allwinner_dma_prep_slave_sg;
	dma_plat->dmadev.device_prep_dma_cyclic = allwinner_dma_prep_dma_cyclic;
	dma_plat->dmadev.device_prep_dma_memcpy = allwinner_dma_prep_dma_memcpy;
	dma_plat->dmadev.device_config = allwinner_dma_config;
    dma_plat->dmadev.device_pause			= allwinner_dma_device_pause;
	dma_plat->dmadev.device_resume			= allwinner_dma_device_resume;
	dma_plat->dmadev.device_terminate_all = allwinner_dma_terminate_all;
	dma_plat->dmadev.device_issue_pending = allwinner_dma_issue_pending;

    dma_plat->dmadev.dev  = dev;

    ret  =  dmaenginem_async_device_register(&dma_plat->dmadev);
    if (ret) {
        _PRINTF_ERROR("dma register device failed! ret = %d\n",  ret);
        return  ret;
    } else {
        _PRINTF_DBG("dma register device successful\n");
    }

    ret = of_dma_controller_register(dev_node, allwinner_dma_xlate, dma_plat);
    if (ret) {
        _PRINTF_ERROR("register of_dma_controller failed! ret = %d\n",  ret);
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


