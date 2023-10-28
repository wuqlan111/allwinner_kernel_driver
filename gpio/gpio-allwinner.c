#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/syscore_ops.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>


#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)


typedef struct {
    uint32_t  cfgx[4];
    uint32_t  dat;
    uint32_t  drvx[2];
    uint32_t  pullx[2];
} __packed allwinner_h6_gpio_config_t;


typedef struct {
    uint32_t  cfgx[4];
    uint32_t  ctl;
    uint32_t  sta;
    uint32_t  deb;
} __packed allwinner_h6_gpio_interrupt_t;


typedef struct {
    allwinner_h6_gpio_config_t  *  config_base;
    allwinner_h6_gpio_interrupt_t  *  interrupt_base;
    uint32_t  has_interrupt;
    uint32_t  hw_irq;
    struct  gpio_chip  gpio_chip;
} allwinner_h6_gpio_plat_t;










static int32_t allwinner_gpio_chip_init(allwinner_h6_gpio_plat_t * gpio_data, struct irq_chip * irqc)
{
    int32_t  ret  =  0;
	ret = gpiochip_add_data(&gpio_data->gpio_chip,  gpio_data);
	if (ret) {
		_PRINTF_ERROR("allwinner register gpio driver failed!\n");
		return  ret;
	}

	ret = gpiochip_irqchip_add(&gpio_data->gpio_chip, irqc, gpio_data->hw_irq, handle_bad_irq,
				   IRQ_TYPE_NONE);
	if (ret) {
		_PRINTF_ERROR("Couldn't add irqchip to gpiochip!\n");
		gpiochip_remove(&gpio_data->gpio_chip);
		return -ENODEV;
	}

	ret = devm_request_irq(gpio_data->gpio_chip.parent, gpio_data->hw_irq,
			       NULL,
			       0, dev_name(gpio_data->gpio_chip.parent), gpio_data);
	if (ret) {
        _PRINTF_ERROR("request irq [%u] failed!\n", gpio_data->hw_irq);
		gpiochip_remove(&gpio_data->gpio_chip);        
    }

	return  ret;
}



static int32_t  allwinner_gpio_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct device * dev  =  &pdev->dev;
    struct device_node *  dev_node  = dev_of_node(dev);
    if (!dev_node) {
        _PRINTF_ERROR("dev_node is null\n");
        return  -EINVAL;
    }

    allwinner_h6_gpio_plat_t * gpio_data  =  devm_kzalloc(dev, sizeof(allwinner_h6_gpio_plat_t), 
                                            GFP_KERNEL);
    if (!gpio_data) {
        _PRINTF_ERROR("kmalloc gpio data failed!\n");
        return  -ENOMEM;
    }
    memset(gpio_data,  0,   sizeof(allwinner_h6_gpio_plat_t));

    struct irq_chip * irqc  = devm_kzalloc(dev, sizeof(struct irq_chip), GFP_KERNEL);
    if (!irqc) {
        _PRINTF_ERROR("alloc irq chip failed!\n");
        return -ENOMEM;
    }
    memset(irqc,  0,  sizeof(struct irq_chip));

    uint32_t  config_base,  int_base, irq_hw, ngpios;
    if ( of_property_read_u32_index(dev_node, "addr", 0, &config_base) ) {
        _PRINTF_ERROR("get allwinner gpio config base failed!\n");
        return  -EINVAL;
    }

    if ( of_property_read_u32_index(dev_node, "irq", 0, &irq_hw) ) {
        _PRINTF_ERROR("get allwinner gpio hwirq base failed!\n");
        return  -EINVAL;
    }

    if ( of_property_read_u32_index(dev_node, "gpios", 0, &ngpios) ) {
        _PRINTF_ERROR("get allwinner gpio cnt failed!\n");
        return  -EINVAL;
    }

    if ( of_property_read_u32_index(dev_node, "addr", 1, &int_base) ) {
        _PRINTF_WARN("allwinner gpio no interrupt!\n");
        config_base  =  0;
        gpio_data->has_interrupt  =  0;
    } else {
        gpio_data->has_interrupt  =  1;
    }

    gpio_data->config_base  =  devm_ioremap(dev, config_base,  sizeof(allwinner_h6_gpio_config_t));
    
    if (gpio_data->has_interrupt) {
        gpio_data->config_base  =  devm_ioremap(dev,  int_base,  sizeof(allwinner_h6_gpio_interrupt_t));
    }

    gpio_data->hw_irq  =  irq_hw;
    gpio_data->gpio_chip.base  =  irq_hw;
    gpio_data->gpio_chip.parent =  dev;
    gpio_data->gpio_chip.label  =  "allwinner_gpio";
    gpio_data->gpio_chip.ngpio  =  ngpios;
    gpio_data->gpio_chip.owner  =  THIS_MODULE;


	platform_set_drvdata(pdev, gpio_data);

	pm_runtime_enable(dev);
	pm_runtime_irq_safe(dev);
	pm_runtime_get_sync(dev);

    ret = allwinner_gpio_chip_init(gpio_data, irqc);
	if (ret) {
		pm_runtime_put_sync(dev);
		pm_runtime_disable(dev);
		return ret;
	}


    return   ret;

}








struct of_device_id allwinner_gpio_ids[] =  {
	{.compatible = "allwinner,H6-v200-gpio"},
    {}
};




struct  platform_driver  allwinner_gpio_driver = {
    .probe   =  allwinner_gpio_probe,
    .remove  =  NULL,
    .driver  =  {
        .name  =  "allwinner_gpio_driver",
        .of_match_table  =  allwinner_gpio_ids,
    }

};



static int32_t  __init  allwinner_gpio_init(void)
{
    int32_t ret =  0;
    ret = platform_driver_register(&allwinner_gpio_driver);

    if (ret) {
        _PRINTF_ERROR("register gpio driver failed! ret = %d\n", ret);
    } else {
        _PRINTF_INFO("allwinner gpio driver!\n");
    }

    return  ret;

}


static void  __exit  allwinner_gpio_exit(void)
{
    platform_driver_unregister(&allwinner_gpio_driver);
    _PRINTF_INFO("allwinner gpio driver remove!\n");
}


module_init(allwinner_gpio_init);
module_exit(allwinner_gpio_exit);

MODULE_DESCRIPTION("allwinner's soc gpio driver");
MODULE_LICENSE("GPL v2");


