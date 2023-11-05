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
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "pinctrl-allwinner.h"

#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)

#define  PINCTRL_BANK_MAX_PIN       32
#define  PINCTRL_PIN_NUMBER(bank, offset)         (  (bank) << 8 | (offset) )

#define PINCTRL_PHY_ADDR_BASE              (0x0300b000u)
#define PINCTRL_R_PHY_ADDR_BASE            (0x07022000u)

typedef  struct {
    char * bank_name;
    uint32_t  bank;
    uint32_t  pins;
} allwinner_pin_bank_t;

#define  ALLWINNER_BANK_INIT(bank_id)   [ALLWINNER_BANK_##bank_id] = {        \
    .bank_name  =  #bank_id,                \
    .bank  =  ALLWINNER_BANK_##bank_id,                  \
    .pins  =  ALLWINNER_BANK_##bank_id##_PINS,         \
}

static  allwinner_pin_bank_t allwinner_h6_bank_info[]  = {
    ALLWINNER_BANK_INIT(PC),
    ALLWINNER_BANK_INIT(PD),
    ALLWINNER_BANK_INIT(PF),
    ALLWINNER_BANK_INIT(PG),
    ALLWINNER_BANK_INIT(PH),
    ALLWINNER_BANK_INIT(PL),
    ALLWINNER_BANK_INIT(PM),
};

typedef  struct {
	const char	*name;
	const char	**groups;
	uint32_t  ngroups;
} allwinner_pmx_func_t;

typedef struct {
	uint8_t  bank;
	uint8_t  pin;
	uint16_t mux;
	uint64_t  conf;
} allwinner_pmx_pin_t;

typedef struct { 
	const char	*name;
	allwinner_pmx_pin_t	*pins_conf;
    uint32_t  * pins;
	uint32_t  npins;
} allwinner_pin_group_t;

typedef struct {
	struct device		* dev;
	struct pinctrl_dev	* pctl;
    struct  pinctrl_desc  pindev;
	allwinner_pmx_func_t * functions;
	uint32_t nfunctions;
	allwinner_pin_group_t * groups;
	uint32_t ngroups;
    void * pinctrl_membase;
    void * pinctrl_r_membase;
} allwinner_pinctrl_desc_t;

typedef struct {
    uint32_t number;
    uint32_t pins;
    allwinner_pin_bank_t * bank_info;
} allwinner_pinctrl_platform_mach_t;

static  allwinner_pinctrl_platform_mach_t h6_v200_platdata = {
    .number = ARRAY_SIZE(allwinner_h6_bank_info), 
    .bank_info  =  allwinner_h6_bank_info,
    .pins  =  93,
};


typedef struct {
    uint32_t  cfgx[4];
    uint32_t  dat;
    uint32_t  drvx[2];
    uint32_t  pullx[2];
} __packed allwinner_h6_pin_config_t;


static int32_t allwinner_get_groups_count(struct pinctrl_dev *pctldev)
{
    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);
    return  pinctrl->ngroups;
}

static  const char *  allwinner_get_group_name(struct pinctrl_dev *pctldev,
				       uint32_t selector)
{
    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);
    if (selector >= pinctrl->ngroups) {
        _PRINTF_ERROR("selector %u excced groups number\n", selector);
        return  NULL;
    }

    return  pinctrl->groups[selector].name;
}

static  int32_t  allwinner_get_group_pins(struct pinctrl_dev *pctldev, uint32_t selector,
			       const uint32_t **pins, uint32_t *num_pins)
{
    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);
    
    if (selector >= pinctrl->ngroups) {
        _PRINTF_ERROR("selector %u excced groups number\n", selector);
        return  -EINVAL;
    }

    *pins  =  pinctrl->groups[selector].pins;
    *num_pins  =  pinctrl->groups[selector].npins;
    return  0;
}

static void allwinner_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
			  uint32_t offset)
{

}

static int32_t  allwinner_dt_node_to_map(struct pinctrl_dev *pctldev,
			       struct device_node *np_config,
			       struct pinctrl_map **map, uint32_t *num_maps)
{
    int32_t  ret =  0;
    return  ret;
}


static  void  allwinner_dt_free_map(struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map, uint32_t num_maps)
{

}


static const struct pinctrl_ops allwinner_pctrl_ops = {
	.get_groups_count = allwinner_get_groups_count,
	.get_group_name = allwinner_get_group_name,
	.get_group_pins = allwinner_get_group_pins,
	.pin_dbg_show = allwinner_pin_dbg_show,
	.dt_node_to_map = allwinner_dt_node_to_map,
	.dt_free_map = allwinner_dt_free_map,
};


static int32_t allwinner_pmx_get_funcs_count(struct pinctrl_dev *pctldev)
{
    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);
    return  pinctrl->nfunctions;
}

static const char *allwinner_pmx_get_func_name(struct pinctrl_dev *pctldev,
					  uint32_t selector)
{
    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);

    if ( selector >= pinctrl->nfunctions ) {
        _PRINTF_ERROR("selector %u excced functions\n", selector);
        return  NULL;
    }

    return  pinctrl->functions[selector].name;

}

static int32_t  allwinner_pmx_get_groups(struct pinctrl_dev *pctldev, uint32_t selector,
				  const char * const **groups, uint32_t * num_groups)
{
    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);

    if ( selector >= pinctrl->nfunctions ) {
        _PRINTF_ERROR("selector %u excced functions\n", selector);
        return  -EINVAL;
    }

    *num_groups = pinctrl->functions[selector].ngroups;
    *groups  =  pinctrl->functions[selector].groups;

    return  0;

}

static int32_t allwinner_pmx_set(struct pinctrl_dev *pctldev, uint32_t func_selector,
			uint32_t group_selector)
{
    return  0;
}

static const struct pinmux_ops allwinner_pmx_ops = {
	.get_functions_count = allwinner_pmx_get_funcs_count,
	.get_function_name = allwinner_pmx_get_func_name,
	.get_function_groups = allwinner_pmx_get_groups,
	.set_mux = allwinner_pmx_set,
};


static int32_t  allwinner_pinconf_get(struct pinctrl_dev *pctldev,
			    uint32_t pin_id, unsigned long * config)
{
	return 0;
}

static int32_t allwinner_pinconf_set(struct pinctrl_dev *pctldev, uint32_t pin_id, 
                unsigned long *configs,  uint32_t num_configs)
{

}


static const struct pinconf_ops allwinner_pinconf_ops = {
	.pin_config_get = allwinner_pinconf_get,
	.pin_config_set = allwinner_pinconf_set,
	// .pin_config_dbg_show = allwinner_pinconf_dbg_show,
	// .pin_config_group_dbg_show = allwinner_pinconf_group_dbg_show,
};

static void  allwinner_pinctrl_function_groups(struct device_node * np_node,  allwinner_pinctrl_desc_t * pinctrl_desc)
{
    struct device_node *child = NULL;

    pinctrl_desc->ngroups  =  0;
    pinctrl_desc->nfunctions =  0;
	for_each_child_of_node(np_node, child) {
		if (of_device_is_compatible(child, ALLWINNER_GPIO_COMPAILE)) {
			continue;
		} else {
			pinctrl_desc->nfunctions++;
			pinctrl_desc->ngroups += of_get_child_count(child);
		}
	}
}


static int32_t allwinner_pinctrl_parse_groups(struct device * dev,  struct device_node *np,  allwinner_pin_group_t * grp)
{
    int32_t ret  =  0;
    int32_t size = 0;
    uint32_t * list = NULL;
	grp->name = np->name;

	list =  (uint32_t * )of_get_property(np, "atmel,pins", &size);
	uint32_t  times  =  size / sizeof(uint32_t);
	if (!times || (size & 0x3) ) {
		_PRINTF_ERROR("pin config must be 4 cells\n");
		return -EINVAL;
	}

	grp->npins = times >> 2;
	grp->pins_conf = devm_kzalloc(dev, grp->npins * sizeof(allwinner_pmx_pin_t),
				GFP_KERNEL);
	grp->pins = devm_kzalloc(dev, grp->npins * sizeof(uint32_t), GFP_KERNEL);
	if (!grp->pins_conf || !grp->pins) {
		return -ENOMEM;        
    }

	for (uint32_t i = 0; i < grp->npins; i++) {
        uint32_t tmp_idx = i << 2;

        grp->pins_conf[i].bank   =  be32_to_cpu(list[tmp_idx]);
        grp->pins_conf[i].pin    =  be32_to_cpu(list[tmp_idx+1]);
        grp->pins_conf[i].mux    =  be32_to_cpu(list[tmp_idx+2]);
        grp->pins_conf[i].conf   =  be32_to_cpu(list[tmp_idx+3]);

		grp->pins[i] = PINCTRL_PIN_NUMBER(grp->pins_conf[i].bank, grp->pins_conf[i].pin);

	}

	return  0;
}



static int32_t allwinner_pinctrl_probe_np(struct platform_device * pdev, allwinner_pinctrl_desc_t * pinctrl_desc )
{
    int32_t  ret  =  0;
    struct  device * dev =  &pdev->dev;
    struct  device_node * dev_node  =  dev->of_node;

    allwinner_pinctrl_function_groups(dev_node,  pinctrl_desc);

    pinctrl_desc->groups  =  devm_kzalloc(dev, pinctrl_desc->ngroups * sizeof(allwinner_pin_group_t), 
                                GFP_KERNEL);
    
    if (!pinctrl_desc->groups) {
        return  -ENOMEM;
    }

    struct  device_node * func_child = NULL;
    struct  device_node * group_child = NULL;
    uint32_t  func_idx, group_idx; 
    func_idx  =  group_idx = 0;
	for_each_child_of_node(dev_node, func_child) {
		if (of_device_is_compatible(func_child, ALLWINNER_GPIO_COMPAILE)) {
			continue;            
        }

        pinctrl_desc->functions[func_idx].name  = func_child->name;
        uint32_t  func_groups =  of_get_child_count(func_child);
        pinctrl_desc->functions[func_idx].ngroups  =  func_groups;
        pinctrl_desc->functions[func_idx].groups  = devm_kzalloc(dev, func_groups * sizeof(char *), GFP_KERNEL);
        if (!pinctrl_desc->functions[func_idx].groups) {
            return  -ENOMEM;
        }

        uint32_t  tmp_goup_idx = 0;
        for_each_child_of_node(func_child, group_child) {
            pinctrl_desc->functions[func_idx].groups[tmp_goup_idx++]  =  group_child->name;
            
            ret  =  allwinner_pinctrl_parse_groups(dev, group_child, &pinctrl_desc->groups[group_idx]);
            if (ret) {
                _PRINTF_ERROR("allwinner pinctrl parse groups failed! ret = %d\n", ret);
                return  ret;  
            }
            group_idx++;
        }
        func_idx++;
	}

	return  ret;

}


static int32_t  allwinner_pinctrl_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct  device * dev =  &pdev->dev;
    struct  device_node * dev_node  =  dev->of_node;

    const allwinner_pinctrl_platform_mach_t * const match_data = device_get_match_data(dev);

    struct pinctrl_pin_desc * pin_descs =  devm_kzalloc(dev,  
                sizeof(struct pinctrl_pin_desc) * match_data->pins,  GFP_KERNEL);
    allwinner_pinctrl_desc_t * pinctrl_desc = devm_kzalloc(dev, 
                                sizeof(allwinner_pinctrl_desc_t), GFP_KERNEL);
    if (!pin_descs || !pinctrl_desc) {
        _PRINTF_ERROR("alloc memory failed!");
        return -ENOMEM;
    }

    ret  =  allwinner_pinctrl_probe_np(pdev,  pinctrl_desc);
    if (ret) {
        _PRINTF_ERROR("allwinner pinctrl parse dev_node failed! ret = %d\n", ret);
        return  ret;
    }

    uint32_t  pin_number = 0;
    for (uint32_t i  =  0; i < match_data->number; i++) {
        allwinner_pin_bank_t * bank_info =  &match_data->bank_info[i];

        for (uint32_t j = 0; j < bank_info->pins; j++) {
            char tmp_name[64] = {0};
            sprintf(tmp_name, "%s-%u", bank_info->bank_name, j);
            pin_descs[pin_number].name = devm_kstrdup_const(dev,  tmp_name,  GFP_KERNEL);
            pin_descs[pin_number].number  =  PINCTRL_PIN_NUMBER(bank_info->bank, j);
            pin_number++;
        }
    }

    pinctrl_desc->pinctrl_membase  =  devm_ioremap(dev, PINCTRL_PHY_ADDR_BASE,  0x400);
    pinctrl_desc->pinctrl_r_membase  =  devm_ioremap(dev, PINCTRL_R_PHY_ADDR_BASE,  0x400);

    pinctrl_desc->pindev.owner  = THIS_MODULE;
    pinctrl_desc->pindev.name  = "allwinner-pinctrl";
    pinctrl_desc->pindev.npins = match_data->number;
    pinctrl_desc->pindev.pins = pin_descs;

    struct pinctrl_dev * pinctrl = NULL;

    ret =  devm_pinctrl_register_and_init(dev,  &pinctrl_desc->pindev,  pinctrl_desc, &pinctrl_desc->pctl);
    if (ret) {
        _PRINTF_ERROR("register pinctrl dev failed! ret = %d\n", ret);
    }

    platform_set_drvdata(pdev, pinctrl_desc);

	return   ret;

}


static int32_t allwinner_pinctrl_remove(struct platform_device * pdev)
{
    struct  device * dev =  &pdev->dev;
    struct pinctrl_dev * pinctrl = platform_get_drvdata(pdev);

    devm_pinctrl_unregister(dev, pinctrl);

	return  0;
}


static struct of_device_id allwinner_pinctrl_ids[] =  {
	{.compatible = "allwinner,H6-v200-pinctrl", .data = &h6_v200_platdata},
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


