#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/syscore_ops.h>
#include <linux/err.h>
#include <linux/io.h>
#include <asm-generic/io.h>
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

#include "gpio-allwinner.h"

#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)

#define  ALLWINNER_GPIO_CAN_SET_FUNC(pin_func)  ( ( (pin_func) == ALLWINNER_H6_PINMUX_INPUT) ||     \
            ((pin_func) == ALLWINNER_H6_PINMUX_OUTPUT) || ( (pin_func) == ALLWINNER_H6_PINMUX_DISABLE) )

typedef struct {
    uint32_t  cfgx[4];
    uint32_t  dat;
    uint32_t  drvx[2];
    uint32_t  pullx[2];
} __packed allwinner_h6_gpio_config_t;


typedef struct {
    uint32_t  cfgx[4];
    uint32_t  ctrl;
    uint32_t  sta;
    uint32_t  debounce;
} __packed allwinner_h6_gpio_interrupt_t;


typedef struct {
    allwinner_h6_gpio_config_t  *  config_base;
    allwinner_h6_gpio_interrupt_t  *  interrupt_base;
    raw_spinlock_t lock;
    uint32_t  has_interrupt;
    uint32_t  hw_irq;
    struct  gpio_chip  gpio_chip;
} allwinner_h6_gpio_plat_t;


static inline allwinner_h6_gpio_plat_t * allwinner_irq_data_get_plat(struct irq_data *d)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(d);
	return gpiochip_get_data(chip);
}


static int32_t  allwinner_gpio_set_pin_value(allwinner_h6_gpio_config_t * regs, const uint32_t pin, const uint32_t value)
{
    uint32_t  mask  =  1 << pin;
    uint32_t  ori_data  =  readl_relaxed(&regs->dat);

    if (value) {
        ori_data  |=  mask;
    } else {
        ori_data  &= ~mask;
    }

    writel_relaxed(ori_data,  &regs->dat);
    return  0;

}

static int32_t  allwinner_gpio_get_pin_value(allwinner_h6_gpio_config_t * regs, const uint32_t pin)
{
    uint32_t  mask  =  1 << pin;
    uint32_t  ori_data  =  readl_relaxed(&regs->dat);
    int32_t  ret  =  ori_data & mask ? 1:  0;
    return  ret;

}

static int32_t  allwinner_gpio_set_pin_config(allwinner_h6_gpio_config_t * regs, uint32_t pin, uint64_t  config)
{
    int32_t  ret  =  0;
    uint32_t  conf_param  =  pinconf_to_config_param(config);
    uint32_t  args  =   pinconf_to_config_argument(config);
    uint32_t  reg_idx,  shift, pull_mode, flag;

    _PRINTF_DBG("gpio_config:\tparam -- %u,\targs -- %#08x\n", conf_param, args);
    switch (conf_param) {
        case  PIN_CONFIG_BIAS_PULL_UP:
            pull_mode  =  ALLWINNER_H6_GPIO_PUPD_UP;
            break;
        case  PIN_CONFIG_BIAS_PULL_DOWN:
            pull_mode  =  ALLWINNER_H6_GPIO_PUPD_DOWN;
            break;
        case  PIN_CONFIG_DRIVE_OPEN_DRAIN:
            pull_mode  =  ALLWINNER_H6_GPIO_PUPD_NO;
            break;
    
        default:
            ret  =  -ENOTSUPP;
            _PRINTF_ERROR("not support config param [%u]\n", conf_param);
    }

    switch (conf_param) {
        case  PIN_CONFIG_BIAS_PULL_UP:
        case  PIN_CONFIG_BIAS_PULL_DOWN:
        case  PIN_CONFIG_DRIVE_OPEN_DRAIN:
            reg_idx  =  ALLWINNNER_H6_PIN_PULL_IDX(pin);
            shift  =  ALLWINNNER_H6_PIN_PULL_SHIFT(pin);
            flag  =  readl_relaxed(&regs->pullx[reg_idx]);
            flag  &= ~(ALLWINNNER_H6_PIN_DRIVE_MASK << shift);
            flag  |=  pull_mode << shift;
            writel_relaxed(flag,  &regs->pullx[reg_idx]);
            break;

    }

    return  ret;

}


static int32_t  allwinner_gpio_set_pin_func(allwinner_h6_gpio_config_t * regs, uint32_t offset, uint32_t func)
{
    uint32_t  cfg_idx = ALLWINNNER_H6_PIN_CFG_IDX(offset);
    uint32_t  shift   =  ALLWINNNER_H6_PIN_CFG_SHIFT(offset);
    uint32_t  ori_cfg  =  readl_relaxed(&regs[cfg_idx]);
    uint32_t  ori_pinctrl = ( ori_cfg >> shift ) & ALLWINNNER_H6_PIN_CFG_MASK;

    if ( !ALLWINNER_GPIO_CAN_SET_FUNC(ori_pinctrl) ) {
        _PRINTF_ERROR("gpio offset [%u] has other func [%u]\n", offset, ori_pinctrl);
        return  -EINVAL;
    }

    if (!ALLWINNER_GPIO_CAN_SET_FUNC(func)) {
        _PRINTF_ERROR("gpio can't set target func [%u]\n", func);
        return  -EINVAL;
    }

    if (ori_pinctrl == func) {
        _PRINTF_ERROR("gpio [%u] has been target func [%u]\n", offset, func);
        return  0;
    }

    ori_cfg &= ~(ALLWINNNER_H6_PIN_CFG_MASK << shift);
    ori_cfg |= func << shift;
    writel_relaxed(ori_cfg,  &regs[cfg_idx]);

    return  0;

}


static int32_t  allwinner_gpio_set_irq_state(allwinner_h6_gpio_interrupt_t * regs, uint32_t pin, uint32_t enable)
{
    uint32_t  int_enable  =  readl_relaxed(&regs->ctrl);
    uint32_t  mask  =  1 << pin;

    if (enable) {
        int_enable  |=  mask;
    } else {
        int_enable  &= ~mask;
    }

    writel_relaxed(int_enable,  &regs->ctrl);

    return  0;

}


static int32_t  allwinner_gpio_set_trigger_type(allwinner_h6_gpio_interrupt_t * regs, uint32_t pin, uint32_t type)
{
    uint32_t  cfg_idx  =  ALLWINNNER_H6_PIN_TRIGGER_IDX(pin);
    uint32_t  shift    =  ALLWINNNER_H6_PIN_TRIGGER_SHIFT(pin);
    uint32_t  ori_cfg  =  readl_relaxed(&regs->cfgx[cfg_idx]);

    if (type > ALLWINNER_H6_PINMUX_MAX_TRIGGER) {
        _PRINTF_ERROR("don't support trigger type [%u]\n",  type);
        return -EOPNOTSUPP;
    }

    ori_cfg  &= ~(ALLWINNNER_H6_PIN_TRIGGER_MASK << shift);
    ori_cfg  |=  type << shift;

    writel_relaxed(ori_cfg,  &regs->cfgx[cfg_idx]);

    return  0;

}


// gpio driver function
static int32_t allwinner_gpio_request(struct gpio_chip *chip, uint32_t offset)
{
    int32_t  ret  =  0;
	allwinner_h6_gpio_plat_t * gpio_data = gpiochip_get_data(chip);
    allwinner_h6_gpio_config_t * regs = gpio_data->config_base;

    unsigned long save_flag  =  0;
    raw_spin_lock_irqsave(&gpio_data->lock,  save_flag);
    ret  =  allwinner_gpio_set_pin_func(regs, offset,  ALLWINNER_H6_PINMUX_INPUT);
    raw_spin_unlock_irqrestore(&gpio_data->lock,  save_flag);

	return  ret;
}

static void allwinner_gpio_free(struct gpio_chip *chip, uint32_t offset)
{
    allwinner_h6_gpio_plat_t * gpio_data = gpiochip_get_data(chip);
    allwinner_h6_gpio_config_t * regs = gpio_data->config_base;

    unsigned long save_flag  =  0;
    raw_spin_lock_irqsave(&gpio_data->lock,  save_flag);
    allwinner_gpio_set_pin_func(regs, offset,  ALLWINNER_H6_PINMUX_DISABLE);
    raw_spin_unlock_irqrestore(&gpio_data->lock,  save_flag);

}

static int32_t allwinner_gpio_get_direction(struct gpio_chip *chip, uint32_t offset)
{
    int32_t  ret  =  0;
    allwinner_h6_gpio_plat_t * gpio_data = gpiochip_get_data(chip);
    allwinner_h6_gpio_config_t * regs = gpio_data->config_base;

    uint32_t  cfg_idx = ALLWINNNER_H6_PIN_CFG_IDX(offset);
    uint32_t  shift   =  ALLWINNNER_H6_PIN_CFG_SHIFT(offset);

    unsigned long save_flag  =  0;
    raw_spin_lock_irqsave(&gpio_data->lock,  save_flag);
    uint32_t  ori_cfg  =  readl_relaxed(&regs[cfg_idx]);
    raw_spin_unlock_irqrestore(&gpio_data->lock,  save_flag);

    uint32_t  ori_pinctrl = ( ori_cfg >> shift ) & ALLWINNNER_H6_PIN_CFG_MASK;
    if (ori_pinctrl == ALLWINNER_H6_PINMUX_INPUT) {
        ret  =  1;
    } else if (ori_pinctrl == ALLWINNER_H6_PINMUX_OUTPUT) {
        ret  =  0;
    } else {
        ret  =  -EINVAL;
    }

	return  ret;
}

static int32_t allwinner_gpio_input(struct gpio_chip *chip, uint32_t offset)
{
    int32_t  ret  =  0;
	allwinner_h6_gpio_plat_t * gpio_data = gpiochip_get_data(chip);
    allwinner_h6_gpio_config_t * regs = gpio_data->config_base;

    unsigned long save_flag  =  0;
    raw_spin_lock_irqsave(&gpio_data->lock,  save_flag);
    ret  =  allwinner_gpio_set_pin_func(regs, offset,  ALLWINNER_H6_PINMUX_INPUT);
    raw_spin_unlock_irqrestore(&gpio_data->lock,  save_flag);

	return  ret;
}

static int32_t allwinner_gpio_get(struct gpio_chip *chip, uint32_t offset)
{
    int32_t  ret  =  0;
	allwinner_h6_gpio_plat_t * gpio_data = gpiochip_get_data(chip);
    allwinner_h6_gpio_config_t * regs = gpio_data->config_base;

    unsigned long save_flag  =  0;
    raw_spin_lock_irqsave(&gpio_data->lock,  save_flag);
    ret  =  allwinner_gpio_get_pin_value(regs,  offset);
    raw_spin_unlock_irqrestore(&gpio_data->lock,  save_flag);

	return  ret;
}

static int32_t allwinner_gpio_output(struct gpio_chip *chip, uint32_t offset, int32_t value)
{
    int32_t  ret  =  0;
	allwinner_h6_gpio_plat_t * gpio_data = gpiochip_get_data(chip);
    allwinner_h6_gpio_config_t * regs = gpio_data->config_base;

    unsigned long save_flag  =  0;
    raw_spin_lock_irqsave(&gpio_data->lock,  save_flag);
    ret   =  allwinner_gpio_set_pin_value(regs, offset,  value);
    if (!ret ) {
        ret = allwinner_gpio_set_pin_func(regs, offset,  ALLWINNER_H6_PINMUX_OUTPUT);        
    }
    raw_spin_unlock_irqrestore(&gpio_data->lock,  save_flag);

	return  ret;
}

static int32_t allwinner_gpio_set_config(struct gpio_chip *chip, uint32_t offset,
				unsigned long config)
{
    int32_t  ret  =  0;
	allwinner_h6_gpio_plat_t * gpio_data = gpiochip_get_data(chip);
    allwinner_h6_gpio_config_t * regs = gpio_data->config_base;

    unsigned long save_flag  =  0;
    raw_spin_lock_irqsave(&gpio_data->lock,  save_flag);
    ret  =  allwinner_gpio_set_pin_config(regs,  offset,  config);
    raw_spin_unlock_irqrestore(&gpio_data->lock,  save_flag);

	return  ret;

}


static void allwinner_gpio_set(struct gpio_chip *chip, uint32_t offset, int32_t value)
{
	allwinner_h6_gpio_plat_t * gpio_data = gpiochip_get_data(chip);
    allwinner_h6_gpio_config_t * regs = gpio_data->config_base;

    unsigned long save_flag  =  0;
    raw_spin_lock_irqsave(&gpio_data->lock,  save_flag);
    allwinner_gpio_set_pin_value(regs, offset,  value);
    raw_spin_unlock_irqrestore(&gpio_data->lock,  save_flag);

}


static uint32_t allwinner_gpio_irq_startup(struct irq_data *d)
{
    int32_t  ret =  0;
	allwinner_h6_gpio_plat_t * gpio_data = allwinner_irq_data_get_plat(d);
    allwinner_h6_gpio_config_t * cfg_regs = gpio_data->config_base;
    allwinner_h6_gpio_interrupt_t * int_regs = gpio_data->interrupt_base;

	unsigned long save_flag =  0;
	uint32_t offset = d->hwirq;

    if (!gpio_data->has_interrupt) {
        _PRINTF_ERROR("gpio port don't support irq\n");
        return -ENOTSUPP;
    }

	raw_spin_lock_irqsave(&gpio_data->lock, save_flag);
    ret = allwinner_gpio_set_pin_func(cfg_regs, offset,  ALLWINNER_H6_PINMUX_INPUT);
    writel_relaxed(1 << offset,  &int_regs->sta);
    if (!ret) {
        ret =  allwinner_gpio_set_irq_state(int_regs, offset, 1);
    }
	raw_spin_unlock_irqrestore(&gpio_data->lock, save_flag);
	return  ret;

}


static void allwinner_gpio_irq_shutdown(struct irq_data *d)
{
	allwinner_h6_gpio_plat_t * gpio_data = allwinner_irq_data_get_plat(d);
    allwinner_h6_gpio_interrupt_t * regs = gpio_data->interrupt_base;
	unsigned  long  flags  = 0;
	uint32_t  offset = d->hwirq;

    if (!gpio_data->has_interrupt) {
        _PRINTF_ERROR("gpio port don't support irq\n");
        return;
    }

	raw_spin_lock_irqsave(&gpio_data->lock, flags);
	allwinner_gpio_set_irq_state(regs, offset, 0);
    writel_relaxed(BIT(offset),  &regs->sta);
	raw_spin_unlock_irqrestore(&gpio_data->lock, flags);

}


static void allwinner_gpio_ack_irq(struct irq_data *d)
{
	allwinner_h6_gpio_plat_t * gpio_data = allwinner_irq_data_get_plat(d);
    allwinner_h6_gpio_interrupt_t * regs = gpio_data->interrupt_base;
	unsigned  long  flags  = 0;
	uint32_t  offset = d->hwirq;

    if (!gpio_data->has_interrupt) {
        _PRINTF_ERROR("gpio port don't support irq\n");
        return;
    }

	raw_spin_lock_irqsave(&gpio_data->lock, flags);
    writel_relaxed(BIT(offset),  &regs->sta);
	raw_spin_unlock_irqrestore(&gpio_data->lock, flags);
}


static void allwinner_gpio_mask_irq(struct irq_data *d)
{
	allwinner_h6_gpio_plat_t * gpio_data = allwinner_irq_data_get_plat(d);
    allwinner_h6_gpio_interrupt_t * regs = gpio_data->interrupt_base;
	unsigned  long  flags  = 0;
	uint32_t  offset = d->hwirq;

    if (!gpio_data->has_interrupt) {
        _PRINTF_ERROR("gpio port don't support irq\n");
        return;
    }

	raw_spin_lock_irqsave(&gpio_data->lock, flags);
    allwinner_gpio_set_irq_state(regs, offset, 0);
	raw_spin_unlock_irqrestore(&gpio_data->lock, flags);
}

static void allwinner_gpio_unmask_irq(struct irq_data *d)
{
	allwinner_h6_gpio_plat_t * gpio_data = allwinner_irq_data_get_plat(d);
    allwinner_h6_gpio_interrupt_t * regs = gpio_data->interrupt_base;
	unsigned  long  flags  = 0;
	uint32_t  offset = d->hwirq;

    if (!gpio_data->has_interrupt) {
        _PRINTF_ERROR("gpio port don't support irq\n");
        return;
    }

	raw_spin_lock_irqsave(&gpio_data->lock, flags);
    allwinner_gpio_set_irq_state(regs, offset, 1);
	raw_spin_unlock_irqrestore(&gpio_data->lock, flags);
}


static int32_t allwinner_gpio_irq_type(struct irq_data *d, uint32_t type)
{
	allwinner_h6_gpio_plat_t * gpio_data = allwinner_irq_data_get_plat(d);
    allwinner_h6_gpio_interrupt_t * regs = gpio_data->interrupt_base;
	unsigned  long  flags  = 0;
	uint32_t offset = d->hwirq;

    if (!gpio_data->has_interrupt) {
        _PRINTF_ERROR("gpio port don't support irq\n");
        return -ENOTSUPP;
    }

	if (type & ~IRQ_TYPE_SENSE_MASK) {
		return -EINVAL;        
    }

	raw_spin_lock_irqsave(&gpio_data->lock, flags);
	// retval = omap_set_gpio_triggering(bank, offset, type);
	// if (retval) {
	// 	raw_spin_unlock_irqrestore(&bank->lock, flags);
	// 	goto error;
	// }
	// omap_gpio_init_irq(bank, offset);
	// if (!omap_gpio_is_input(bank, offset)) {
	// 	raw_spin_unlock_irqrestore(&bank->lock, flags);
	// 	retval = -EINVAL;
	// 	goto error;
	// }
	raw_spin_unlock_irqrestore(&gpio_data->lock, flags);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH)) {
		irq_set_handler_locked(d, handle_level_irq);        
    } else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING)) {
		irq_set_handler_locked(d, handle_simple_irq);        
    }

	return 0;

}


static irqreturn_t allwinner_gpio_irq_handler(int32_t irq, void * dev_id)
{
	allwinner_h6_gpio_plat_t * gpio_data = dev_id;
    allwinner_h6_gpio_interrupt_t * int_regs  =  gpio_data->interrupt_base;
    unsigned long  lock_flags =  0;

    if (!gpio_data->has_interrupt) {
        _PRINTF_WARN("gpio don't support irq\n");
        return  IRQ_HANDLED;
    }

    uint32_t  enable, isr,  bit;
    enable =  isr = bit =  0;
	while (1) {
		raw_spin_lock_irqsave(&gpio_data->lock, lock_flags);

        enable  =  readl_relaxed(&int_regs->ctrl);
        isr   =  readl_relaxed(&int_regs->sta) & enable;

		raw_spin_unlock_irqrestore(&gpio_data->lock, lock_flags);

		if (!isr) {
			break; 
        }

		while (isr) {
			bit = __ffs(isr);
			isr &= ~(BIT(bit));

            raw_spin_lock_irqsave(&gpio_data->lock, lock_flags);
            writel_relaxed(BIT(bit), &int_regs->sta);
            raw_spin_unlock_irqrestore(&gpio_data->lock, lock_flags);

			generic_handle_irq(irq_find_mapping(gpio_data->gpio_chip.irq.domain,
							    bit));
		}
	}

	return  IRQ_HANDLED;
}



static int32_t allwinner_gpio_chip_init(allwinner_h6_gpio_plat_t * gpio_data, struct irq_chip * irqc)
{
    int32_t  ret  =  0;

	irqc->irq_startup = allwinner_gpio_irq_startup,
	irqc->irq_shutdown = allwinner_gpio_irq_shutdown,
	irqc->irq_ack = allwinner_gpio_ack_irq,
	irqc->irq_mask = allwinner_gpio_mask_irq,
	irqc->irq_unmask = allwinner_gpio_unmask_irq,
	irqc->irq_set_type = allwinner_gpio_irq_type,
	irqc->flags = IRQCHIP_MASK_ON_SUSPEND;

    gpio_data->gpio_chip.request  =  allwinner_gpio_request;
    gpio_data->gpio_chip.free  =  allwinner_gpio_free;
    gpio_data->gpio_chip.get_direction  = allwinner_gpio_get_direction;
    gpio_data->gpio_chip.direction_input  = allwinner_gpio_input;
    gpio_data->gpio_chip.get  =  allwinner_gpio_get;
    gpio_data->gpio_chip.direction_output  =  allwinner_gpio_output;
    gpio_data->gpio_chip.set_config  =  allwinner_gpio_set_config;
    gpio_data->gpio_chip.set  =  allwinner_gpio_set;


    //alloc virtual irq desc
    int32_t  irq_base  =  0;
    irq_base = devm_irq_alloc_descs(gpio_data->gpio_chip.parent,
					-1, 0, gpio_data->gpio_chip.ngpio, 0);
	if (irq_base < 0) {
		_PRINTF_ERROR("Couldn't allocate IRQ numbers\n");
		return -ENODEV;
	}

	ret = gpiochip_irqchip_add(&gpio_data->gpio_chip, irqc, irq_base, handle_bad_irq,
				   IRQ_TYPE_NONE);
	if (ret) {
		_PRINTF_ERROR("Couldn't add irqchip to gpiochip!\n");
		gpiochip_remove(&gpio_data->gpio_chip);
		return -ENODEV;
	}

	ret = gpiochip_add_data(&gpio_data->gpio_chip,  gpio_data);
	if (ret) {
		_PRINTF_ERROR("allwinner register gpio driver failed!\n");
		return  ret;
	}

    // gpiochip_set_chained_irqchip(&gpio_data->gpio_chip, irqc, gpio_data->hw_irq, NULL);

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

    irqc->name = dev_name(&pdev->dev);

    gpio_data->hw_irq  =  irq_hw;
    gpio_data->gpio_chip.base  =  irq_hw;
    gpio_data->gpio_chip.parent =  dev;
    gpio_data->gpio_chip.label  =  "allwinner_gpio";
    gpio_data->gpio_chip.ngpio  =  ngpios;
    gpio_data->gpio_chip.owner  =  THIS_MODULE;

    raw_spin_lock_init(&gpio_data->lock);

	platform_set_drvdata(pdev, gpio_data);


    ret = allwinner_gpio_chip_init(gpio_data, irqc);

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


