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
#include "dbg_log.h"




typedef struct {
    struct  spi_controller controller;
    void * map_base;
    phys_addr_t  phy_addr;
} allwinner_spi_controller_t;


static int32_t allwinner_spi_setup(struct spi_device *spi)
{




}


static int32_t allwinner_spi_transfer_one_message(struct spi_controller *master,
						struct spi_message *msg)
{




}








static int32_t  allwinner_spi_probe(struct platform_device * pdev)
{
    int32_t  ret =  0;
    struct device * dev  = &pdev->dev;
    struct device_node * dev_node =  dev->of_node;

    uint32_t phy_addr =  0;
    uint32_t * map_base = NULL;
    if (of_property_read_u32(dev_node,  "addr",  &phy_addr)) {
        return -EINVAL;
    }

    map_base  =  devm_ioremap(dev, phy_addr,  ALLWINNER_SPI_MAP_SIZE);
    if (!map_base) {
        return  -EINVAL;
    }

    allwinner_spi_controller_t * spi_host  =  devm_kzalloc( dev,  sizeof(allwinner_spi_controller_t), 
                                            GFP_KERNEL );
    if (!spi_host) {
        return -ENOMEM;
    }
    memset(spi_host,  0,   sizeof(allwinner_spi_controller_t));

    spi_host->map_base  =  map_base;
    spi_host->phy_addr  =  phy_addr;

    spi_host->controller.dev.parent  = dev;
    spi_host->controller.mode_bits  =  SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
    spi_host->controller.num_chipselect  =  4;
    spi_host->controller.bits_per_word_mask  =  SPI_BPW_RANGE_MASK(0, 32);
    spi_host->controller.setup   =  allwinner_spi_setup;
    spi_host->controller.flags   =  SPI_MASTER_MUST_RX | SPI_MASTER_MUST_TX;
    spi_host->controller.transfer_one_message  =  allwinner_spi_transfer_one_message;
	// spi_host->controller.cleanup = allwinner_spi_cleanup;
	spi_host->controller.auto_runtime_pm = true;
	spi_host->controller.max_dma_len = ALLWINNER_SPI_MAX_DMA_XFER;
	// spi_host->controller.can_dma = allwinner_spi_can_dma;

    ret  =  devm_spi_register_controller(dev,  &spi_host->controller);
    if (ret) {
        _PRINTF_ERROR("register spi host failed! ret = %d\n", ret);
    } else {
        platform_set_drvdata(pdev,  spi_host);
    }


    return  0;
}


static int32_t allwinner_spi_remove(struct platform_device * pdev)
{
    allwinner_spi_controller_t * spi_host  =  platform_get_drvdata(pdev);

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


