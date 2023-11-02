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
#include <linux/clk-provider.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>

#include "clk-allwinner.h"

#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)

static const struct clk_ops pll_clk_ops = {0};
static const struct clk_ops normal_clk_ops = {0};


const char * pll_peri0_12x_parent[1] = { "pll_peri0_4x" };
const char * pll_peri1_12x_parent[1] = { "pll_peri1_4x" };
const char * clk_psi_ahb1_parent[4] = {"osc24", "ccu32k", "rc16m", "pll_peri0_1x"};
const char * clk_ap1_parent[4] = {"osc24", "ccu32k", "clk_psi_ahb1", "pll_peri0_1x"};

#define CLK_HW_INIT_DATA(name_str, clk_ops,  parent_nr,  parant_name)    {  \
            .name =  name_str,                 \
            .ops  =  clk_ops,                    \
            .num_parents  =  parent_nr,           \
            .parent_names  =  parant_name ,           \
}

#define _CLK_HW_INIT(clk_id,  name_str, ops,  parent_nr,  parant_name)   [clk_id] =   \
                            CLK_HW_INIT_DATA(name_str, ops,  parent_nr,  parant_name)


#define  CLK_INIT_DATAS     \
    _CLK_HW_INIT(ALLWINNER_PLL_CPUX, "pll_cpux",  &pll_clk_ops,  0, NULL),   \
    _CLK_HW_INIT(ALLWINNER_PLL_DDR0, "pll_ddr0",  &pll_clk_ops,  0, NULL),   \
    _CLK_HW_INIT(ALLWINNER_PLL_PERI0_4X, "pll_peri0_4x",  &pll_clk_ops,  0, NULL),   \
    _CLK_HW_INIT(ALLWINNER_PLL_PERI0_2X, "pll_peri0_2x",  &pll_clk_ops,  1,  pll_peri0_12x_parent),   \
    _CLK_HW_INIT(ALLWINNER_PLL_PERI0_1X, "pll_peri0_1x",  &pll_clk_ops,  1,  pll_peri0_12x_parent),   \
    _CLK_HW_INIT(ALLWINNER_PLL_PERI1_4X, "pll_peri1_4x",  &pll_clk_ops,  0, NULL),   \
    _CLK_HW_INIT(ALLWINNER_PLL_PERI1_2X, "pll_peri1_2x",  &pll_clk_ops,  1,  pll_peri1_12x_parent ),   \
    _CLK_HW_INIT(ALLWINNER_PLL_PERI1_1X, "pll_peri1_1x",  &pll_clk_ops,  1,  pll_peri1_12x_parent ),   \
                                                                   \
    _CLK_HW_INIT(ALLWINNER_CLK_PSI_AHB1_AHB2, "clk_psi_ahb1",  &normal_clk_ops,  4, clk_psi_ahb1_parent ),   \
    _CLK_HW_INIT(ALLWINNER_CLK_AHB3, "clk_ahb3",  &normal_clk_ops,  4,  clk_ap1_parent),   \
    _CLK_HW_INIT(ALLWINNER_CLK_APB1, "clk_apb1",  &normal_clk_ops,  4,  clk_ap1_parent),   \
    _CLK_HW_INIT(ALLWINNER_CLK_APB2, "clk_apb2",  &normal_clk_ops,  4,  clk_ap1_parent),


static struct clk_init_data _pll_normal_clk_init[] = {CLK_INIT_DATAS};

typedef  struct {
    struct  clk_hw  hw;
    uint32_t  clk_id;
    uint32_t  offset;
} allwinner_clk_hw_t;


#define  ALLWINNER_CLK_HW(clk, reg)    [clk] = {   \
    .hw.init = &_pll_normal_clk_init[clk],    \
    .clk_id  =  clk,                    \
    .offset  =  reg,            \
}

#define  ALLWINNER_CLKS              \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_CPUX,  0),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_DDR0,  0x10),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI0_4X,  0x20),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI0_2X,  0x20),           \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI0_1X,  0x20),            \
                                                               \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI0_4X,  0x20),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI0_2X,  0x20),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI0_1X,  0x20),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI1_4X,  0x28),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI1_2X,  0x28),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI1_1X,  0x28),            \
                                                               \
    ALLWINNER_CLK_HW(ALLWINNER_CLK_PSI_AHB1_AHB2,  0x510),            \
    ALLWINNER_CLK_HW(ALLWINNER_CLK_AHB3,  0x51c),            \
    ALLWINNER_CLK_HW(ALLWINNER_CLK_APB1,  0x520),            \
    ALLWINNER_CLK_HW(ALLWINNER_CLK_APB2,  0x524),


static  allwinner_clk_hw_t allwinner_clks[] = { ALLWINNER_CLKS };






#define  TYPE_CASE(x)         case x: return #x;
static char * allwinner_clk_2_str(const uint32_t clk)
{

    switch (clk) {
        TYPE_CASE(ALLWINNER_PLL_CPUX)
        TYPE_CASE(ALLWINNER_PLL_DDR0)
        TYPE_CASE(ALLWINNER_PLL_PERI0_4X)
        TYPE_CASE(ALLWINNER_PLL_PERI0_2X)
        TYPE_CASE(ALLWINNER_PLL_PERI0_1X)
        TYPE_CASE(ALLWINNER_PLL_PERI1_4X)
        TYPE_CASE(ALLWINNER_PLL_PERI1_2X)
        TYPE_CASE(ALLWINNER_PLL_PERI1_1X)
        TYPE_CASE(ALLWINNER_PLL_GPU)
        TYPE_CASE(ALLWINNER_PLL_VIDEO0_4X)
        TYPE_CASE(ALLWINNER_PLL_VIDEO0_1X)
        TYPE_CASE(ALLWINNER_PLL_VIDEO1_4X)
        TYPE_CASE(ALLWINNER_PLL_VIDEO1_1X)
        TYPE_CASE(ALLWINNER_PLL_VE)
        TYPE_CASE(ALLWINNER_PLL_DE)
        TYPE_CASE(ALLWINNER_PLL_HSIC)
        TYPE_CASE(ALLWINNER_PLL_AUDIO_4X)
        TYPE_CASE(ALLWINNER_PLL_AUDIO)

        // clk
        TYPE_CASE(ALLWINNER_CLK_CPUX_AXI)
        TYPE_CASE(ALLWINNER_CLK_CPUX_APB)
        TYPE_CASE(ALLWINNER_CLK_PSI_AHB1_AHB2)
        TYPE_CASE(ALLWINNER_CLK_AHB3)
        TYPE_CASE(ALLWINNER_CLK_APB1)
        TYPE_CASE(ALLWINNER_CLK_APB2)
        TYPE_CASE(ALLWINNER_CLK_MBUS)
        TYPE_CASE(ALLWINNER_CLK_NAND0)
        TYPE_CASE(ALLWINNER_CLK_NAND1)
        TYPE_CASE(ALLWINNER_CLK_SMHC0)
        TYPE_CASE(ALLWINNER_CLK_SMHC1)
        TYPE_CASE(ALLWINNER_CLK_SMHC2)
        TYPE_CASE(ALLWINNER_CLK_SPI0)
        TYPE_CASE(ALLWINNER_CLK_SPI1)
        TYPE_CASE(ALLWINNER_CLK_TS)
        TYPE_CASE(ALLWINNER_CLK_CIR_TX)
        TYPE_CASE(ALLWINNER_CLK_I2S_PCM3)
        TYPE_CASE(ALLWINNER_CLK_I2S_PCM0)
        TYPE_CASE(ALLWINNER_CLK_I2S_PCM1)
        TYPE_CASE(ALLWINNER_CLK_I2S_PCM2)
        TYPE_CASE(ALLWINNER_CLK_OWA)
        TYPE_CASE(ALLWINNER_CLK_DMIC)
        TYPE_CASE(ALLWINNER_CLK_AUDIO_HUB)
        TYPE_CASE(ALLWINNER_CLK_USB0)
        TYPE_CASE(ALLWINNER_CLK_USB1)
        TYPE_CASE(ALLWINNER_CLK_USB3)
        TYPE_CASE(ALLWINNER_CLK_PCIE_REF)
        TYPE_CASE(ALLWINNER_CLK_PCIE_MAXI)
        TYPE_CASE(ALLWINNER_CLK_PCIE_AUX)
        TYPE_CASE(ALLWINNER_CLK_HDMI)
        TYPE_CASE(ALLWINNER_CLK_HDMI_SLOW)
        TYPE_CASE(ALLWINNER_CLK_HDMI_CEC)
        TYPE_CASE(ALLWINNER_CLK_HDMI_HDCP)
        TYPE_CASE(ALLWINNER_CLK_TCON_LED)
        TYPE_CASE(ALLWINNER_CLK_TCON_TV)
        TYPE_CASE(ALLWINNER_CLK_CSI_MISC)
        TYPE_CASE(ALLWINNER_CLK_CSI_TOP)
        TYPE_CASE(ALLWINNER_CLK_CSI_MASTER)
        TYPE_CASE(ALLWINNER_CLK_UART0)
        TYPE_CASE(ALLWINNER_CLK_UART1)
        TYPE_CASE(ALLWINNER_CLK_UART2)
        TYPE_CASE(ALLWINNER_CLK_UART3)

    }

    return   "invalid allwinner clk";

}





typedef  struct {
    uint32_t  phy_addr;
    uint32_t *  map_base;
} platform_drv_data_t;

static struct clk_hw * allwinner_of_clk_get(struct of_phandle_args *clkspec, void * unused)
{
	uint32_t idx = clkspec->args[0];

	if (idx > ALLWINNER_PLL_MAX ) {
		_PRINTF_ERROR("invalid index %u\n", idx);
		return ERR_PTR(-EINVAL);
	}

	return  &allwinner_clks[idx].hw;
}

static int32_t  allwinner_clk_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct  device * dev =  &pdev->dev;
    struct  device_node * dev_node  =  dev->of_node;

    uint32_t  phy_addr =  0;
    if (of_property_read_u32(dev_node, "addr", &phy_addr)) {
        _PRINTF_ERROR("read clk base addr failed\n");
        return -EINVAL;
    }

    platform_drv_data_t * data  = devm_kzalloc(dev, sizeof(platform_drv_data_t), GFP_KERNEL);
    if (!data) {
        _PRINTF_ERROR("alloc driver data failed!\n");
        return -ENOMEM;
    }

    uint32_t  * map = devm_ioremap(dev,  phy_addr,   ALLWINNER_CCU_MAP_SIZE);
    if (IS_ERR(map)) {
        _PRINTF_ERROR("map  ccu phy addr to kernel failed!\n");
		return  PTR_ERR(map);
    }

    data->map_base  =  map;
    data->phy_addr  =  phy_addr;
    platform_set_drvdata(pdev, data);

    uint32_t size  = ARRAY_SIZE(allwinner_clks);
    for ( uint32_t i =  0; i < size; i++) {
        if ( !allwinner_clks[i].hw.init->ops || (allwinner_clks[i].clk_id != i) ) {
            continue;
        }

		ret = devm_clk_hw_register(dev, &allwinner_clks[i].hw);
		if (ret) {
            _PRINTF_ERROR("register clk_hw %s failed! ret = %d\n", allwinner_clk_2_str(i),  ret);
			return ret;
        }
    }

    ret  =  of_clk_add_hw_provider(dev_node, allwinner_of_clk_get, NULL);
	return   ret;

}


static int32_t allwinner_clk_remove(struct platform_device * pdev)
{
    int32_t  ret  =  0;


	return  ret;
}


static struct of_device_id allwinner_uart_ids[] =  {
	{.compatible = "allwinner,H6-v200-ccu"},
    {}
};


struct  platform_driver  allwinner_clk_driver = {
    .probe   =  allwinner_clk_probe,
    .remove	 =  allwinner_clk_remove,
    .driver  =  {
        .name  =  "allwinner_uart_driver",
        .of_match_table  =  allwinner_uart_ids,
    }

};


static int32_t  __init  allwinner_clk_init(void)
{
    int32_t ret =  0;

    ret = platform_driver_register(&allwinner_clk_driver);
    if (ret) {
        _PRINTF_ERROR("register clk driver failed! ret = %d\n", ret);
    } else {
        _PRINTF_INFO("register allwinner clk driver success!\n");
    }

    return  ret;

}


static void  __exit  allwinner_clk_exit(void)
{
    platform_driver_unregister(&allwinner_clk_driver);
    _PRINTF_INFO("allwinner clk driver remove!\n");

}


module_init(allwinner_clk_init);
module_exit(allwinner_clk_exit);

MODULE_DESCRIPTION("allwinner's soc clk driver");
MODULE_LICENSE("GPL v2");


