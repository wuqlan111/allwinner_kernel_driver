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
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "pinctrl-allwinner.h"

#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)

#define  ALLWINNER_BANK_PC_PINS    17
#define  ALLWINNER_BANK_PD_PINS    27
#define  ALLWINNER_BANK_PF_PINS    7
#define  ALLWINNER_BANK_PG_PINS    15
#define  ALLWINNER_BANK_PH_PINS    11
#define  ALLWINNER_BANK_PL_PINS    11
#define  ALLWINNER_BANK_PM_PINS    5

#define  ALLWINNER_BANK_PC_BASE    0
#define  ALLWINNER_BANK_PD_BASE    (ALLWINNER_BANK_PC_BASE + ALLWINNER_BANK_PC_PINS)
#define  ALLWINNER_BANK_PF_BASE    (ALLWINNER_BANK_PD_BASE + ALLWINNER_BANK_PD_PINS)
#define  ALLWINNER_BANK_PG_PINS    (ALLWINNER_BANK_PF_BASE + ALLWINNER_BANK_PF_PINS)
#define  ALLWINNER_BANK_PH_PINS    (ALLWINNER_BANK_PG_BASE + ALLWINNER_BANK_PG_PINS)
#define  ALLWINNER_BANK_PL_PINS    (ALLWINNER_BANK_PH_BASE + ALLWINNER_BANK_PH_PINS)
#define  ALLWINNER_BANK_PM_PINS    (ALLWINNER_BANK_PL_BASE + ALLWINNER_BANK_PL_PINS)W

#define  ALLWINNER_BANK_PC_PHY_BASE    (0x0300b000u)
#define  ALLWINNER_BANK_PD_PHY_BASE    (0x0300b000u)
#define  ALLWINNER_BANK_PF_PHY_BASE    (0x0300b000u)
#define  ALLWINNER_BANK_PG_PHY_BASE    (0x0300b000u)
#define  ALLWINNER_BANK_PH_PHY_BASE    (0x0300b000u)
#define  ALLWINNER_BANK_PL_PHY_BASE    (0x07022000u)
#define  ALLWINNER_BANK_PM_PHY_BASE    (0x07022000u)



typedef enum {
    ALLWINNER_BANK_PC =  0,
    ALLWINNER_BANK_PD,
    ALLWINNER_BANK_PF,
    ALLWINNER_BANK_PG,
    ALLWINNER_BANK_PH,
    ALLWINNER_BANK_PL,
    ALLWINNER_BANK_PM,
} allwinner_pin_bank_e;


typedef  struct {
    char * bank_name;
    uint32_t  bank;
    uint32_t  offset;
    uint32_t  pins;
    uint32_t  phy_base;
    uint32_t  map_size;
} allwinner_pin_bank_t;


#define  ALLWINNER_BANK_INIT(bank_id, reg_offset)  {        \
    .bank_name  =  #bank_id,                \
    .bank  =  bank_id,                  \
    .offset  =  reg_offset,            \
    .base  =  bank_id##_BASE,          \
    .pins  =  bank_id##_PINS,         \
    .phy_base  =  bank_id##_PHY_BASE,       \
    .map_size  =  0x400,               \
}


static  allwinner_pin_bank_t allwinner_h6_bank_info[]  = {
    ALLWINNER_BANK_INIT(ALLWINNER_BANK_PC,  0x48),
    ALLWINNER_BANK_INIT(ALLWINNER_BANK_PD,  0x6c),
    ALLWINNER_BANK_INIT(ALLWINNER_BANK_PF,  0xb4),
    ALLWINNER_BANK_INIT(ALLWINNER_BANK_PG,  0xd8),
    ALLWINNER_BANK_INIT(ALLWINNER_BANK_PH,  0xfc),
    ALLWINNER_BANK_INIT(ALLWINNER_BANK_PL,  0),
    ALLWINNER_BANK_INIT(ALLWINNER_BANK_PM,  0x24),
};


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






// static const struct pinctrl_ops allwinner_pctrl_ops = {
// 	.get_groups_count = allwinner_get_groups_count,
// 	.get_group_name = allwinner_get_group_name,
// 	.get_group_pins = allwinner_get_group_pins,
// 	.pin_dbg_show = allwinner_pin_dbg_show,
// 	.dt_node_to_map = allwinner_dt_node_to_map,
// 	.dt_free_map = allwinner_dt_free_map,
// };


// static const struct pinmux_ops allwinner_pmx_ops = {
// 	.get_functions_count = allwinner_pmx_get_funcs_count,
// 	.get_function_name = allwinner_pmx_get_func_name,
// 	.get_function_groups = allwinner_pmx_get_groups,
// 	.set_mux = allwinner_pmx_set,
// };


// static const struct pinconf_ops allwinner_pinconf_ops = {
// 	.pin_config_get = allwinner_pinconf_get,
// 	.pin_config_set = allwinner_pinconf_set,
// 	.pin_config_dbg_show = allwinner_pinconf_dbg_show,
// 	.pin_config_group_dbg_show = allwinner_pinconf_group_dbg_show,
// };


typedef  struct {
    struct  pinctrl_desc  pindev;
} allwinner_pinctrl_desc_t;

typedef  struct {
    uint32_t * base;
    uint32_t  pin_offset;
} allwinner_pin_desc_data_t;


static int32_t  allwinner_pinctrl_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct  device * dev =  &pdev->dev;
    struct  device_node * dev_node  =  dev->of_node;

    allwinner_pinctrl_platform_mach_t * match_data = device_get_match_data(dev);

    struct pinctrl_pin_desc * pin_descs =  devm_kzalloc(dev,  
                sizeof(struct pinctrl_pin_desc) * match_data->pins,  GFP_KERNEL);
    allwinner_pin_desc_data_t * pin_desc_data  =  devm_kzalloc(dev,  
                sizeof(allwinner_pin_desc_data_t) * match_data->pins,  GFP_KERNEL)
    allwinner_pinctrl_desc_t * pinctrl_desc = devm_kzalloc(dev, 
                                sizeof(allwinner_pinctrl_desc_t), GFP_KERNEL);
    if (!pin_descs || !pin_desc_data || !pinctrl_desc) {
        _PRINTF_ERROR("alloc memory failed!");
        return -ENOMEM;
    }
    
    uint32_t  pin_number = 0;
    for (uint32_t i  =  0; i < match_data->number; i++) {
        allwinner_pin_bank_t * bank_info =  &match_data->bank_info[i];
        uint32_t * base_addr = devm_ioremap(dev, bank_info->phy_base, bank_info->map_size);

        for (uint32_t j = 0; j < bank_info->pins; j++) {
            char tmp_name[64] = {0};
            sprintf(tmp_name, "%s-%u", bank_info->bank_name, j);
            pin_desc_data[pin_number].base  =  base_addr;
            pin_desc_data[pin_number].pin_offset  =  j;
            pin_descs[pin_number].drv_data  =  &pin_desc_data[pin_number];
            pin_descs[pin_number].name = devm_kstrdup_const(dev,  tmp_name,  GFP_KERNEL);
            pin_number++;
        }
    }

    pinctrl_desc->pindev.owner  = THIS_MODULE;
    pinctrl_desc->pindev.name  = "allwinner-pinctrl";
    pinctrl_desc->pindev.npins = match_data->number;
    pinctrl_desc->pindev.pins = pin_descs;

    struct pinctrl_dev * pinctrl = NULL;

    ret =  devm_pinctrl_register_and_init(dev,  pinctrl_desc,  NULL, &pinctrl);
    if (ret) {
        _PRINTF_ERROR("register pinctrl dev failed! ret = %d\n", ret);
    }

    platform_set_drvdata(pdev, pinctrl);

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


