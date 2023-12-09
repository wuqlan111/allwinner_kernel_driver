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
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "pinctrl-allwinner.h"
#include "dbg_log.h"


#define PINCTRL_PHY_ADDR_BASE              (0x0300b000u)
#define PINCTRL_R_PHY_ADDR_BASE            (0x07022000u)
#define PINCTRL_CFG_STEP        (0x24u)
#define PIN_GROUP_NO_CONFIG_PMX        (0xffffu)



#define ALLWINNER_PINCONF_UNPACK(conf, param)\
				(( (conf) >> ALLWINNER_PINCONF_ ##param ##_SHIFT) \
				& ALLWINNER_PINCONF_ ##param ##_MASK)

#define ALLWINNER_PINCONF_PACK(conf, val, param)	( (conf) |=   \
				(((val) & ALLWINNER_PINCONF_ ##param ##_MASK) << \
					ALLWINNER_PINCONF_ ##param ##_SHIFT))

#define  ALLWINNER_PINCONF_PULL_MASK		(0x3u)
#define  ALLWINNER_PINCONF_PULL_SHIFT		(0u)
#define  ALLWINNER_PINCONF_UNPACK_PULL(conf)	ALLWINNER_PINCONF_UNPACK(conf, PULL)
#define  ALLWINNER_PINCONF_PACK_PULL(conf, pull)	 ALLWINNER_PINCONF_PACK(conf, pull, PULL)

#define  ALLWINNER_PINCONF_DRV_MASK		(0x3u)
#define  ALLWINNER_PINCONF_DRV_SHIFT		(2u)
#define  ALLWINNER_PINCONF_UNPACK_DRV(conf)	ALLWINNER_PINCONF_UNPACK(conf, DRV)
#define  ALLWINNER_PINCONF_PACK_DRV(conf, drv)	 ALLWINNER_PINCONF_PACK(conf, drv, DRV)


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
	uint8_t  offset;
    uint16_t  pin;
    unsigned long  config;
} allwinner_pmx_pin_t;

typedef struct { 
	const char	* const name;
	allwinner_pmx_pin_t	*pins_conf;
    const uint32_t  * const pins;
    const uint32_t  function;
    const uint32_t  af_func;
	const uint32_t  npins;
} allwinner_pin_group_t;

typedef struct { 
    uint32_t  cur_function;
    uint32_t  pmx_count;
} allwinner_pin_group_state_t;

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
    allwinner_pin_group_state_t * groups_state;
} allwinner_pinctrl_desc_t;

typedef struct {
    uint32_t number;
    uint32_t pins;
    allwinner_pin_bank_t * bank_info;
    allwinner_pmx_func_t * functions;
	uint32_t nfunctions;
	allwinner_pin_group_t * groups;
	uint32_t ngroups;
} allwinner_pinctrl_platform_data_t;

typedef struct {
    uint32_t  cfgx[4];
    uint32_t  dat;
    uint32_t  drvx[2];
    uint32_t  pullx[2];
} __packed allwinner_h6_pin_config_t;



//pins of each group
const static uint32_t nand_pins[] = {
    ALLWINNER_PIN_NUMBER(PC, 0), ALLWINNER_PIN_NUMBER(PC, 1), ALLWINNER_PIN_NUMBER(PC, 2),
    ALLWINNER_PIN_NUMBER(PC, 3), ALLWINNER_PIN_NUMBER(PC, 4), ALLWINNER_PIN_NUMBER(PC, 5),
    ALLWINNER_PIN_NUMBER(PC, 6), ALLWINNER_PIN_NUMBER(PC, 7), ALLWINNER_PIN_NUMBER(PC, 8),
    ALLWINNER_PIN_NUMBER(PC, 9), ALLWINNER_PIN_NUMBER(PC, 10), ALLWINNER_PIN_NUMBER(PC, 11),
    ALLWINNER_PIN_NUMBER(PC, 12), ALLWINNER_PIN_NUMBER(PC, 13), ALLWINNER_PIN_NUMBER(PC, 14),
    ALLWINNER_PIN_NUMBER(PC, 15),  ALLWINNER_PIN_NUMBER(PC, 16), };

const static uint32_t spi0_pins[] = {
    ALLWINNER_PIN_NUMBER(PC, 0), ALLWINNER_PIN_NUMBER(PC, 2), ALLWINNER_PIN_NUMBER(PC, 3), 
    ALLWINNER_PIN_NUMBER(PC, 5), ALLWINNER_PIN_NUMBER(PC, 6), ALLWINNER_PIN_NUMBER(PC, 7), 
};

const static uint32_t spi1_pins[] = {
    ALLWINNER_PIN_NUMBER(PH, 3), ALLWINNER_PIN_NUMBER(PH, 4), ALLWINNER_PIN_NUMBER(PH, 5), 
    ALLWINNER_PIN_NUMBER(PH, 6), };


const static uint32_t uart0_pins[] = {
    ALLWINNER_PIN_NUMBER(PF, 2), ALLWINNER_PIN_NUMBER(PF, 4), };

const static uint32_t uart1_pins[] = {
    ALLWINNER_PIN_NUMBER(PG, 6), ALLWINNER_PIN_NUMBER(PG, 7), };

const static uint32_t uart2_pins[] = {
    ALLWINNER_PIN_NUMBER(PD, 19), ALLWINNER_PIN_NUMBER(PD, 20), ALLWINNER_PIN_NUMBER(PD, 21), 
    ALLWINNER_PIN_NUMBER(PD, 22), };


const static uint32_t uart3_pins[] = {
    ALLWINNER_PIN_NUMBER(PD, 23), ALLWINNER_PIN_NUMBER(PD, 24), ALLWINNER_PIN_NUMBER(PD, 25), 
    ALLWINNER_PIN_NUMBER(PD, 26), };


const static uint32_t sdc0_pins[] = {
    ALLWINNER_PIN_NUMBER(PF, 0), ALLWINNER_PIN_NUMBER(PF, 1), ALLWINNER_PIN_NUMBER(PF, 2), 
    ALLWINNER_PIN_NUMBER(PF, 3), ALLWINNER_PIN_NUMBER(PF, 4), ALLWINNER_PIN_NUMBER(PF, 5),
};


const static uint32_t sdc1_pins[] = {
    ALLWINNER_PIN_NUMBER(PG, 0), ALLWINNER_PIN_NUMBER(PG, 1), ALLWINNER_PIN_NUMBER(PG, 2), 
    ALLWINNER_PIN_NUMBER(PG, 3), ALLWINNER_PIN_NUMBER(PG, 4), ALLWINNER_PIN_NUMBER(PG, 5),
};

const static uint32_t sdc2_pins[] = {
    ALLWINNER_PIN_NUMBER(PC, 1), ALLWINNER_PIN_NUMBER(PC, 4), ALLWINNER_PIN_NUMBER(PC, 5), 
    ALLWINNER_PIN_NUMBER(PC, 6), ALLWINNER_PIN_NUMBER(PC, 7), ALLWINNER_PIN_NUMBER(PC, 8),
    ALLWINNER_PIN_NUMBER(PC, 9), ALLWINNER_PIN_NUMBER(PC, 10), ALLWINNER_PIN_NUMBER(PC, 11),
    ALLWINNER_PIN_NUMBER(PC, 12), ALLWINNER_PIN_NUMBER(PC, 13), ALLWINNER_PIN_NUMBER(PC, 14),
};


enum {
    ALLWINNER_FUNC_NAND =  0,
    ALLWINNER_FUNC_SPI,
    ALLWINNER_FUNC_UART,
    ALLWINNER_FUNC_SDC,
    ALLWINNER_FUNC_MAX = ALLWINNER_FUNC_SDC, 
} allwinner_pinctrl_function_e;

static const char * nand_groups[] = { "nand" };
static const char * spi_groups[]  = { "spi0",  "spi1" };
static const char * uart_groups[] = { "uart0", "uart1", "uart2", "uart3" };
static const char * sdc_groups[] = { "sdc0", "sdc1", "sdc2" };

#define ALLWINNER_FUNCTION(id, fname)  [ ALLWINNER_FUNC_##id ]  =  {  \
            .name  =  #fname,       \
            .groups = fname##_groups,        \
            .ngroups =  ARRAY_SIZE(fname##_groups),   \
}

const static allwinner_pmx_func_t allwinner_h6_functions[] = {
    ALLWINNER_FUNCTION(NAND, nand),
    ALLWINNER_FUNCTION(SPI, spi),
    ALLWINNER_FUNCTION(UART, uart),
    ALLWINNER_FUNCTION(SDC, sdc),
};

#define ALLWINNER_GROUP(fname, func, af) {   \
    .name =  #fname,          \
    .pins = fname##_pins,         \
    .function =  ALLWINNER_FUNC_##func,       \
    .af_func  =  ALLWINNER_H6_PINMUX_##af,       \
    .npins = ARRAY_SIZE(fname##_pins),   \
}

static allwinner_pin_group_t allwinner_h6_groups[] = {
    ALLWINNER_GROUP(nand, NAND,  AF2),
    ALLWINNER_GROUP(spi0, SPI,   AF4),
    ALLWINNER_GROUP(spi1, SPI,   AF2),
    ALLWINNER_GROUP(uart0,  UART,   AF3),
    ALLWINNER_GROUP(uart1,  UART,   AF2),
    ALLWINNER_GROUP(uart2,  UART,   AF4),
    ALLWINNER_GROUP(uart3,  UART,   AF4),
    ALLWINNER_GROUP(sdc0,   SDC,    AF2),
    ALLWINNER_GROUP(sdc1,   SDC,    AF2),
    ALLWINNER_GROUP(sdc2,   SDC,    AF3),
};


static int32_t allwinner_get_groups_count(struct pinctrl_dev *pctldev)
{
    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);
    return  pinctrl->ngroups;
}

static allwinner_h6_pin_config_t * allwinner_get_pin_config_register(void * base, void * r_base, uint32_t bank)
{
    uint32_t * base_addr = bank < ALLWINNER_BANK_PL? base: r_base;
    uint32_t  cfg_offset = bank < ALLWINNER_BANK_PL? bank: bank - ALLWINNER_BANK_PL;

    allwinner_h6_pin_config_t * regs =  (allwinner_h6_pin_config_t *)&base_addr[(cfg_offset * PINCTRL_CFG_STEP) >> 2 ];

    return regs;
}

static  int32_t allwinner_set_pin_af(allwinner_h6_pin_config_t * regs, uint32_t offset, uint32_t af_func)
{
    if (offset >= PINCTRL_BANK_MAX_PIN) {
        _PRINTF_ERROR("bank pin offset %u invalid!\n", offset);
        return -EINVAL;
    }

    if (af_func > ALLWINNER_H6_PINMUX_MAX_AF) {
        _PRINTF_ERROR("bank pin af_func %u invalid!\n", af_func);
        return -EINVAL;
    }

    uint32_t reg_idx =  ALLWINNNER_H6_PIN_AF_IDX(offset);
    uint32_t shift =  ALLWINNNER_H6_PIN_AF_SHIFT(offset);

    uint32_t flag = readl_relaxed(&regs->cfgx[reg_idx]);
    flag &= ~(ALLWINNNER_H6_PIN_AF_MASK<<shift);
    flag |= (af_func << shift);
    writel_relaxed(flag, &regs->cfgx[reg_idx]);

    return  0;

}

static  int32_t allwinner_get_pin_af(allwinner_h6_pin_config_t * regs, uint32_t offset, uint32_t * af_func)
{
    if (offset >= PINCTRL_BANK_MAX_PIN) {
        _PRINTF_ERROR("bank pin offset %u invalid!\n", offset);
        return -EINVAL;
    }

    uint32_t reg_idx =  ALLWINNNER_H6_PIN_AF_IDX(offset);
    uint32_t shift =  ALLWINNNER_H6_PIN_AF_SHIFT(offset);

    uint32_t flag = readl_relaxed(&regs->cfgx[reg_idx]);
    *af_func  =  (flag >> shift) & ALLWINNNER_H6_PIN_AF_MASK;

    return  0;
}


static  int32_t allwinner_set_pin_pull_type(allwinner_h6_pin_config_t * regs, uint32_t offset, uint32_t pull_type)
{
    if (offset >= PINCTRL_BANK_MAX_PIN) {
        _PRINTF_ERROR("bank pin offset %u invalid!\n", offset);
        return -EINVAL;
    }

    if (pull_type > ALLWINNER_H6_GPIO_MAX) {
        _PRINTF_ERROR("bank pin pull_type %u invalid!\n", pull_type);
        return -EINVAL;
    }

    uint32_t reg_idx = ALLWINNNER_H6_PIN_PULL_IDX(offset);
    uint32_t shift =  ALLWINNNER_H6_PIN_PULL_SHIFT(offset);

    uint32_t flag = readl_relaxed(&regs->pullx[reg_idx]);
    flag &= ~(ALLWINNNER_H6_PIN_PULL_MASK<<shift);
    flag |= (pull_type << shift);
    writel_relaxed(flag, &regs->pullx[reg_idx]);

    return  0;

}


static  int32_t allwinner_get_pin_pull_type(allwinner_h6_pin_config_t * regs, uint32_t offset, uint32_t * pull_type)
{
    if (offset >= PINCTRL_BANK_MAX_PIN) {
        _PRINTF_ERROR("bank pin offset %u invalid!\n", offset);
        return -EINVAL;
    }

    uint32_t reg_idx = ALLWINNNER_H6_PIN_PULL_IDX(offset);
    uint32_t shift =  ALLWINNNER_H6_PIN_PULL_SHIFT(offset);

    uint32_t flag = readl_relaxed(&regs->pullx[reg_idx]);
    *pull_type  = (flag >> shift) & ALLWINNNER_H6_PIN_PULL_MASK;

    return  0;

}


static allwinner_pin_group_t * allwinner_get_group_by_name(allwinner_pinctrl_desc_t * pctrldev, char * name)
{
    allwinner_pin_group_t * ret = NULL;
    for (uint32_t i = 0; i < pctrldev->ngroups; i++) {
        if (!strcmp(pctrldev->groups[i].name, name)) {
            ret = &pctrldev->groups[i];
            break;
        }
    }
 
    if (!ret) {
        _PRINTF_ERROR("pctrldev %s no group %s\n", pctrldev->pindev.name, name);
    }

    return  ret;

}

static int32_t allwinner_get_group_pin_idx(allwinner_pin_group_t * group, uint32_t pin, uint32_t * idx)
{
    uint32_t  find = 0;
    int32_t  ret =  0;
    *idx  =  0;

    for (uint32_t i = 0; i < group->npins; i++) {
        if (group->pins[i] ==  pin) {
            find =  1;
            *idx = i;
            break;
        }
    }

    ret = find ? 0: -EINVAL;
    if (ret) {
        _PRINTF_ERROR("pin [%u] not in group %s\n", pin, group->name);
    }

    return  ret;

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
    allwinner_pinctrl_desc_t * allwinner_pinctrl = pinctrl_dev_get_drvdata(pctldev);
	seq_printf(s, "%s", dev_name(allwinner_pinctrl->dev));
}

static int32_t  allwinner_dt_node_to_map(struct pinctrl_dev *pctldev,
			       struct device_node *np_config,
			       struct pinctrl_map **map, uint32_t *num_maps)
{
    // int32_t  ret =  0;
    allwinner_pinctrl_desc_t * allwinner_pinctrl = pinctrl_dev_get_drvdata(pctldev);
    allwinner_pin_group_t * grp = NULL;
	struct pinctrl_map *new_map = NULL;
	struct device_node *parent = NULL;

	grp = allwinner_get_group_by_name(allwinner_pinctrl, np_config->name);
	if (!grp) {
		return -EINVAL;
	}

	uint32_t map_num = grp->npins + 1;
	new_map = devm_kzalloc(allwinner_pinctrl->dev, sizeof(struct pinctrl_map) * map_num, GFP_KERNEL);
	if (!new_map) {
		return -ENOMEM;        
    }

	parent = of_get_parent(np_config);
	if (!parent) {
		devm_kfree(allwinner_pinctrl->dev, new_map);
		return -EINVAL;
	}

	*map = new_map;
	*num_maps = map_num;
	new_map[0].type = PIN_MAP_TYPE_MUX_GROUP;
	new_map[0].data.mux.function = parent->name;
	new_map[0].data.mux.group = np_config->name;
	of_node_put(parent);

    size_t  size =  0;
    uint32_t  prop_val[256] = {0};
    if (of_property_read_u32_array(np_config, "allwinner,pins", prop_val, &size)) {
        _PRINTF_ERROR("get pins for node [%s]\n", np_config->name);
        return  -EINVAL;
    }

#define  PIN_NP_BANK_IDX  0
#define  PIN_NP_BANK_OFFSET  1
#define  PIN_NP_CONFIG  2
#define  PIN_NP_CELL_SIZE   3

    if (size % PIN_NP_CELL_SIZE) {
        _PRINTF_ERROR("pin config node invalid!\n");
        return -EINVAL;
    }

    uint32_t times = size / PIN_NP_CELL_SIZE;
    for (uint32_t i = 0; i < times; i++) {
        uint32_t tmp_idx = i * PIN_NP_CELL_SIZE;
        uint32_t tmp_bank = prop_val[tmp_idx+PIN_NP_BANK_IDX];
        uint32_t tmp_offset = prop_val[tmp_idx+PIN_NP_BANK_OFFSET];
        uint32_t tmp_config = prop_val[tmp_idx+PIN_NP_CONFIG];
        uint32_t tmp_pin =  PINCTRL_PIN_NUMBER(tmp_bank, tmp_offset);

        uint32_t  grp_idx = 0;
        if (allwinner_get_group_pin_idx(grp, tmp_pin, &grp_idx)) {
            return -EINVAL;
        }

        grp->pins_conf[grp_idx].bank = tmp_bank;
        grp->pins_conf[grp_idx].offset  = tmp_offset;
        grp->pins_conf[grp_idx].pin  =  tmp_pin;
        grp->pins_conf[grp_idx].config  =  tmp_config;

        new_map[i+1].type = PIN_MAP_TYPE_CONFIGS_PIN;
		// new_map[i+1].data.configs.group_or_pin =
		// 		pin_get_name(pctldev, tmp_pin);
		new_map[i+1].data.configs.configs = &grp->pins_conf[grp_idx].config;
		new_map[i+1].data.configs.num_configs = 1;
    }

	_PRINTF_ERROR("maps:\tfunction=%s,\tgroup=%s,\tnum=%d\n", (*map)->data.mux.function, 
                            grp->name, map_num);

	return 0;

}


static  void  allwinner_dt_free_map(struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map, uint32_t num_maps)
{

}


static const struct pinctrl_ops allwinner_pctrl_ops = {
	.get_groups_count = allwinner_get_groups_count,
	.get_group_name = allwinner_get_group_name,
	.get_group_pins = allwinner_get_group_pins,
#ifdef CONFIG_DEBUG_FS
	.pin_dbg_show = allwinner_pin_dbg_show,
#endif
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
    int32_t  ret =  0;
    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);
    const uint32_t  grp_support_function = pinctrl->groups[group_selector].function ;
    allwinner_pin_group_state_t * grp_state = &pinctrl->groups_state[group_selector];
    allwinner_pin_group_t * grp = &pinctrl->groups[group_selector];

    if (grp_support_function != func_selector) {
        _PRINTF_ERROR("group %s don't support function %s\n", pinctrl->groups[group_selector].name,
                    pinctrl->functions[func_selector].name);
        return  -ENOTSUPP;
    }

    if (grp_state->cur_function == func_selector) {
        _PRINTF_WARN("group %s has been config function %s\n", pinctrl->groups[group_selector].name,
                    pinctrl->functions[func_selector].name);
        return   0;
    }

    for (uint32_t i = 0; i < grp->npins; i++) {
        uint32_t tmp_bank = PIN_NUMBER_BANK(grp->pins[i]);
        uint32_t tmp_offset  =  PIN_NUMBER_OFFSET(grp->pins[i]);

        allwinner_h6_pin_config_t *  regs = allwinner_get_pin_config_register(pinctrl->pinctrl_membase, 
                                                pinctrl->pinctrl_r_membase, tmp_bank);
        
        ret = allwinner_set_pin_af(regs, tmp_offset, grp->af_func);
        if (ret) {
            return ret;
        }
    }

    grp_state->cur_function = func_selector;
    grp_state->pmx_count++;

    return  0;
}

static const struct pinmux_ops allwinner_pmx_ops = {
    .strict = true,
	.get_functions_count = allwinner_pmx_get_funcs_count,
	.get_function_name = allwinner_pmx_get_func_name,
	.get_function_groups = allwinner_pmx_get_groups,
	.set_mux = allwinner_pmx_set,
};


static int32_t  allwinner_pinconf_get(struct pinctrl_dev *pctldev,
			    uint32_t pin_id, unsigned long * config)
{
    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);
    const  uint32_t  bank = PIN_NUMBER_BANK(pin_id);
    const  uint32_t  offset  = PIN_NUMBER_BANK(pin_id);

    allwinner_h6_pin_config_t * regs = allwinner_get_pin_config_register(pinctrl->pinctrl_membase, 
                            pinctrl->pinctrl_r_membase, bank);

    *config = 0;

    uint32_t  pull_type = 0;
    if (allwinner_get_pin_pull_type(regs, offset, &pull_type)) {
        return  -1;
    }
	
    ALLWINNER_PINCONF_PACK_PULL(*config, pull_type);

    _PRINTF_DBG("pin=%#x,\tconfig=%#lx\n", pin_id, *config);

	return 0;
}

static int32_t allwinner_pinconf_set(struct pinctrl_dev *pctldev, uint32_t pin_id, 
                unsigned long *configs,  uint32_t num_configs)
{
    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);
    const  uint32_t  bank = PIN_NUMBER_BANK(pin_id);
    const  uint32_t  offset  = PIN_NUMBER_BANK(pin_id);

    allwinner_h6_pin_config_t * regs =  allwinner_get_pin_config_register(pinctrl->pinctrl_membase, 
                            pinctrl->pinctrl_r_membase, bank);

    _PRINTF_DBG("pin_id=%#x,\tconfigs_num=%u\n", pin_id, num_configs);

    for (uint32_t  i  =  0; i < num_configs; i++) {  
        uint32_t pull_type = ALLWINNER_PINCONF_UNPACK_PULL(configs[i]); 
        allwinner_set_pin_pull_type(regs, offset, pull_type);
    }

    return  0;
}

static void allwinner_pinconf_dbg_show(struct pinctrl_dev *pctldev,
				   struct seq_file *s, uint32_t pin_id)
{

    allwinner_pinctrl_desc_t * pinctrl = pinctrl_dev_get_drvdata(pctldev);
    const  uint32_t  bank = PIN_NUMBER_BANK(pin_id);
    const  uint32_t  offset  = PIN_NUMBER_BANK(pin_id);

    allwinner_h6_pin_config_t * regs =  allwinner_get_pin_config_register(pinctrl->pinctrl_membase, 
                            pinctrl->pinctrl_r_membase, bank);

	unsigned long config = 0;
	uint32_t function = 0;
    char func_str[40] = {0};

    allwinner_get_pin_af(regs, offset,  &function);
    allwinner_pinconf_get(pctldev, pin_id,  &config);

    if ((function == ALLWINNER_H6_GPIO_IN) || (function == ALLWINNER_H6_GPIO_OUT)) {
        sprintf(func_str, "gpio");
    } else {
        sprintf(func_str, "AF_FUNC%u",  function);
    }

    seq_printf(s, "[PU:%ld]\t%s\n", ALLWINNER_PINCONF_UNPACK_PULL(config),  func_str);

}



static const struct pinconf_ops allwinner_pinconf_ops = {
	.pin_config_get = allwinner_pinconf_get,
	.pin_config_set = allwinner_pinconf_set,
#ifdef CONFIG_DEBUG_FS
	.pin_config_dbg_show = allwinner_pinconf_dbg_show,
    // .pin_config_group_dbg_show = allwinner_pinconf_group_dbg_show,
#endif
};


static int32_t  allwinner_pinctrl_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct  device * dev =  &pdev->dev;
    // struct  device_node * dev_node  =  dev->of_node;

    const allwinner_pinctrl_platform_data_t * const match_data = device_get_match_data(dev);

    struct pinctrl_pin_desc * pin_descs =  devm_kzalloc(dev,  
                sizeof(struct pinctrl_pin_desc) * match_data->pins,  GFP_KERNEL);
    allwinner_pinctrl_desc_t * pinctrl_desc = devm_kzalloc(dev, 
                                sizeof(allwinner_pinctrl_desc_t), GFP_KERNEL);
    if (!pin_descs || !pinctrl_desc) {
        _PRINTF_ERROR("alloc memory failed!");
        return -ENOMEM;
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

    allwinner_pin_group_state_t * grp_state = devm_kzalloc(dev, 
                    sizeof(allwinner_pin_group_state_t) * match_data->ngroups, GFP_KERNEL );
    for (uint32_t i = 0; i < match_data->ngroups; i++) {
        grp_state[i].cur_function = PIN_GROUP_NO_CONFIG_PMX;
        grp_state[i].pmx_count  =  0;

        uint32_t  grp_pins =  match_data->groups[i].npins;
        match_data->groups[i].pins_conf  =  devm_kzalloc(dev, sizeof(allwinner_pmx_pin_t) * grp_pins,
                                                GFP_KERNEL );
        if (!match_data->groups[i].pins_conf) {
            return -ENOMEM;
        }
    }

    pinctrl_desc->dev = dev;
    // pinctrl_dev
    pinctrl_desc->groups = match_data->groups;
    pinctrl_desc->ngroups = match_data->ngroups;
    pinctrl_desc->groups_state = grp_state;
    pinctrl_desc->functions = match_data->functions;
    pinctrl_desc->nfunctions = match_data->nfunctions;
    pinctrl_desc->pinctrl_membase  =  devm_ioremap(dev, PINCTRL_PHY_ADDR_BASE,  ALLWINNER_PINCTRL_MAP_SIZE);
    pinctrl_desc->pinctrl_r_membase  =  devm_ioremap(dev, PINCTRL_R_PHY_ADDR_BASE,  ALLWINNER_PINCTRL_MAP_SIZE);

    pinctrl_desc->pindev.owner  = THIS_MODULE;
    pinctrl_desc->pindev.name  = "allwinner-pinctrl";
    pinctrl_desc->pindev.npins = match_data->number;
    pinctrl_desc->pindev.pins = pin_descs;

    pinctrl_desc->pindev.confops  = &allwinner_pinconf_ops;
    pinctrl_desc->pindev.pctlops  = &allwinner_pctrl_ops;
    pinctrl_desc->pindev.pmxops  = &allwinner_pmx_ops;

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

const static  allwinner_pinctrl_platform_data_t h6_v200_platdata = {
    .number = ARRAY_SIZE(allwinner_h6_bank_info), 
    .bank_info  =  allwinner_h6_bank_info,
    .pins  =  93,
    .functions  =  allwinner_h6_functions,
    .nfunctions  =  ARRAY_SIZE(allwinner_h6_functions),
    .groups  =  allwinner_h6_groups,
    .nfunctions  =  ARRAY_SIZE(allwinner_h6_groups),
};

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


