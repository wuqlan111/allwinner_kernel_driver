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
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/clk-provider.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/resource.h>

#include "clk-allwinner.h"
#include "dbg_log.h"


typedef  struct {
    struct  clk_hw  hw;
    uint32_t  clk_id;
    uint32_t  offset;
    phys_addr_t  phy_addr;
    uint32_t  *  map_base;
} allwinner_clk_hw_t;

typedef struct {
    struct  clk * all_clks[ALLWINNER_MAX_CLK_ID + 1];
} allwinner_clk_dev_info_t;

static  allwinner_clk_dev_info_t g_clk_info = {0};


#define  CLK_NAME_MAP(id, clk_name) [id] = clk_name

const static char * clk_name_maps[ALLWINNER_MAX_CLK_ID+2] = {
    CLK_NAME_MAP(ALLWINNER_PLL_CPUX, "pll_cpux"),
    CLK_NAME_MAP(ALLWINNER_PLL_DDR0, "pll_ddr0"),
    CLK_NAME_MAP(ALLWINNER_PLL_PERI0_4X,  "pll_peri0_4x"),
    CLK_NAME_MAP(ALLWINNER_PLL_PERI0_2X, "pll_peri0_2x"),
    CLK_NAME_MAP(ALLWINNER_PLL_PERI0_1X, "pll_peri0_1x"),
    CLK_NAME_MAP(ALLWINNER_PLL_PERI1_4X, "pll_peri1_4x"), 
    CLK_NAME_MAP(ALLWINNER_PLL_PERI1_2X, "pll_peri1_2x"), 
    CLK_NAME_MAP(ALLWINNER_PLL_PERI1_1X, "pll_peri1_1x"), 

    CLK_NAME_MAP(ALLWINNER_CLK_PSI_AHB1_AHB2, "clk_psi_ahb1"), 
    CLK_NAME_MAP(ALLWINNER_CLK_AHB3, "clk_ahb3"), 
    CLK_NAME_MAP(ALLWINNER_CLK_APB1, "clk_apb1"), 
    CLK_NAME_MAP(ALLWINNER_CLK_APB2, "clk_apb2"),
};



static const char * allwinner_clk_2_str(const uint32_t clk_id)
{
    return  clk_id <= ALLWINNER_MAX_CLK_ID ? clk_name_maps[clk_id]: NULL;
}

static int32_t  allwinner_clk_name_idx(const char * clk_name, uint32_t * clk_id)
{
    uint32_t find =  0;
    for (int32_t i = 0; i < ARRAY_SIZE(clk_name_maps); i++) {
        if (!clk_name_maps[i]) {
            continue;
        }

        if (!strcmp(clk_name, clk_name_maps[i])) {
            *clk_id = i;
            find = 1;
            break;
        }

    }

    return  find? 0: -1;
}



static  int32_t  _allwinner_h6_pll_enable(struct clk_hw * hw)
{
    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;

    _PRINTF_DBG("enbale pll_clk [%s],\toffset = %#x\n", __clk_get_name(hw->clk), reg_offset);

    uint32_t  flag = readl_relaxed(&clk_hw->map_base[reg_offset]);
    if ( !(flag & ALLWINNER_H6_PLLX_ENABLE) ) {
        flag |= ( ALLWINNER_H6_PLLX_ENABLE | ALLWINNER_H6_PLLX_LOCK_ENABLE );
        writel_relaxed(flag,  &clk_hw->map_base[reg_offset]);
    }

    while (!(readl_relaxed(&clk_hw->map_base[reg_offset]) & ALLWINNER_H6_PLLX_LOCK))  ;
    udelay(20);
    return   0;
}


static  void  _allwinner_h6_pll_disable(struct clk_hw * hw)
{
    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;

    uint32_t  flag = readl_relaxed(&clk_hw->map_base[reg_offset]);
    if ( flag & ALLWINNER_H6_PLLX_ENABLE ) {
        flag &= ~( ALLWINNER_H6_PLLX_ENABLE | ALLWINNER_H6_PLLX_LOCK_ENABLE );
        writel_relaxed(flag,  &clk_hw->map_base[reg_offset]);
    }

}


static  int32_t  _allwinner_h6_pll_enabled(struct clk_hw * hw)
{
    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;

    uint32_t  flag = readl_relaxed(&clk_hw->map_base[reg_offset]);
    int32_t  ret  =  flag & ALLWINNER_H6_PLLX_ENABLE ? 1:  0;

    return   ret;
}

static  unsigned long _allwinner_h6_pll_recalc_rate(struct clk_hw *hw, 
                unsigned long parent_rate)
{
    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;
    uint32_t  clk_id  =  clk_hw->clk_id;

    uint32_t * reg_addr =  &clk_hw->map_base[reg_offset];
    uint32_t  pll_n, pll_m, tmp_m;
    pll_n  = pll_m  = tmp_m = 0;    

    uint32_t  clk_ctrl = readl_relaxed(reg_addr);
    pll_n  =  ((clk_ctrl & ALLWINNER_H6_PLLX_FACTOR_N ) >>  8)  + 1;

    switch (clk_id) {
        case  ALLWINNER_PLL_CPUX:
            tmp_m  =  (clk_ctrl & GENMASK(17,  16)) >> 16;
            pll_m  =  1 << tmp_m;
            break;

        case  ALLWINNER_PLL_DDR0 ... ALLWINNER_PLL_GPU:
        case  ALLWINNER_PLL_VE ...  ALLWINNER_PLL_HSIC:
            tmp_m  =  clk_ctrl & 0x3;
            pll_m =  (tmp_m ==  0)  || (tmp_m == 3) ? tmp_m + 1: 2;
            break;
        case ALLWINNER_PLL_VIDEO0_4X ... ALLWINNER_PLL_VIDEO1_1X:
            pll_m  =  clk_ctrl &  BIT(1) ? 2: 1;
            break;

        case  ALLWINNER_PLL_AUDIO_4X:
        case  ALLWINNER_PLL_AUDIO: {
                uint32_t in_div  =  clk_ctrl & BIT(1)? 2: 1;
                uint32_t out_div  =  clk_ctrl & BIT(0)? 2:  1;
                uint32_t p = (clk_ctrl & GENMASK(21,  16) >> 16) + 1;
                if (clk_id == ALLWINNER_PLL_AUDIO_4X) {
                    pll_m  =  2 * in_div;
                } else {
                    pll_m  =  p * in_div * out_div;
                }
        }
            break;
        
        default:
            _PRINTF_ERROR("clk_id -- %u, invalid pll clk id!", clk_id);
            return  -EINVAL;
    }

    uint32_t  shift =  0;
    switch (clk_id) {
        case ALLWINNER_PLL_PERI0_1X:
        case ALLWINNER_PLL_PERI1_1X:
        case ALLWINNER_PLL_VIDEO0_1X:
        case ALLWINNER_PLL_VIDEO1_1X:
            shift =  2;
            break;

        case ALLWINNER_PLL_PERI0_2X:
        case ALLWINNER_PLL_PERI1_2X:
            shift  =  1;
            break;
    
        default:
            shift  =  0;
    }

    unsigned long rate  =  ( ALLWINNER_PLLX_SRC_CLK * pll_n) / ( pll_m * (1 << shift) );

    return  rate;

}

static  uint32_t  allwinner_pll_m[] = { 1,  2,  4};
static  long  _allwinner_h6_pll_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
    long  ret =  0;

    uint32_t  pll_m, pll_n, best_pll_m, best_pll_n;
    ulong rate_delta  =  rate;
    pll_m  = pll_n  = best_pll_m  =  best_pll_n  =  0;

    const  uint32_t  pll_m_count  =  ARRAY_SIZE(allwinner_pll_m);
    const  ulong   tmp_rate_min   =  rate * allwinner_pll_m[0];
    const  ulong   tmp_rate_max   =  rate * allwinner_pll_m[pll_m_count - 1];
    const  ulong   pll_n_min  =  tmp_rate_min / ALLWINNER_PLLX_SRC_CLK;
    const  ulong   pll_n_max  =  tmp_rate_max / ALLWINNER_PLLX_SRC_CLK;

    if (!rate) {
        _PRINTF_ERROR("rate is zero!");
        return  -EINVAL;
    }

    for (int32_t  i  =  0;  i  <  pll_m_count; i++) {
        uint32_t  find_best  =  0;
        for (pll_n =  pll_n_min; pll_n < pll_n_max; pll_n++) {
            ulong  tmp_rate  =  (pll_n * ALLWINNER_PLLX_SRC_CLK) / allwinner_pll_m[i];
            ulong  tmp_delta  =  rate - tmp_rate;

            if ( (pll_n > 0xff) || (tmp_rate > rate) ) {
                break;
            }

            if (tmp_delta < rate_delta) {
                rate_delta  =  tmp_delta;
                best_pll_m  =  allwinner_pll_m[i];
                best_pll_n  =  pll_n;
                *parent_rate   =  tmp_rate;
            }

            if (tmp_rate == rate) {
                find_best =  1;
                break;
            }

        }
    }

    if ( !best_pll_m || !best_pll_n) {
        _PRINTF_ERROR("rate -- [%lu] not support!", rate);
        ret  =  -EINVAL;
    }

    return  ret;

}

static  int32_t _allwinner_h6_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;
    uint32_t  clk_id  =  clk_hw->clk_id;

    uint32_t  pll_m, pll_n, best_pll_m, best_pll_n;
    ulong   rate_delta  =  rate;
    pll_m  = pll_n  = best_pll_m  =  best_pll_n  =  0;

    const  uint32_t  pll_m_count  =  ARRAY_SIZE(allwinner_pll_m);
    const  ulong   tmp_rate_min   =  rate * allwinner_pll_m[0];
    const  ulong   tmp_rate_max   =  rate * allwinner_pll_m[pll_m_count - 1];
    const  ulong   pll_n_min  =  tmp_rate_min / ALLWINNER_PLLX_SRC_CLK;
    const  ulong   pll_n_max  =  tmp_rate_max / ALLWINNER_PLLX_SRC_CLK;

    if (!rate) {
        _PRINTF_ERROR("rate is zero!");
        return  -EINVAL;
    }

    for (int32_t  i  =  0;  i  <  pll_m_count; i++) {
        uint32_t  find_best  =  0;
        for (pll_n =  pll_n_min; pll_n < pll_n_max; pll_n++) {
            ulong  tmp_rate  =  (pll_n * ALLWINNER_PLLX_SRC_CLK) / allwinner_pll_m[i];
            ulong  tmp_delta  =  rate - tmp_rate;

            if ( (pll_n > 0xff) || (tmp_rate > rate) ) {
                break;
            }

            if (tmp_delta < rate_delta) {
                rate_delta  =  tmp_delta;
                best_pll_m  =  allwinner_pll_m[i];
                best_pll_n  =  pll_n;
                // *new_rate   =  tmp_rate;
            }

            if (tmp_rate == rate) {
                find_best =  1;
                break;
            }

        }
    }


    if ( !best_pll_m || !best_pll_n) {
        _PRINTF_ERROR("rate -- [%lu] not support!", rate);
        return  -EINVAL;
    }

    uint32_t  flag,  mask;
    flag  =  (best_pll_n - 1) << 8;
    mask  =  ALLWINNER_H6_PLLX_FACTOR_N;

    switch (clk_id) {
        case  ALLWINNER_PLL_CPUX:
            if (best_pll_m < 4) {
                flag  |=  ( pll_m - 1)  << 16;
            } else {
                flag  |=  0x2  <<  16;
            }

            mask  |=  0x3 <<  16;
            break;

        case  ALLWINNER_PLL_DDR0 ... ALLWINNER_PLL_GPU:
        case  ALLWINNER_PLL_VE ...  ALLWINNER_PLL_HSIC:
            flag  |=  (best_pll_m - 1);
            mask  |=  0x3;
            break;
        // case  ALLWINNER_PLL_VIDEO0:
        // case  ALLWINNER_PLL_VIDEO1: 
            if (best_pll_m > 2) {
                _PRINTF_ERROR("pll_m -- [%u], rate -- [%lu] invalid!", best_pll_m, rate);
                return  -EINVAL;
            }
            flag  |=  (best_pll_m - 1) << 1;
            mask  |=  0x2;
            break;
        
        default:
            _PRINTF_ERROR("clk_id -- %u, invalid pll clk id!", clk_id);
            return  -EINVAL;
    }

    uint32_t * reg_addr =  &clk_hw->map_base[reg_offset];

    uint32_t  clk_ctrl  =  readl(reg_addr);
    uint32_t  clk_enable  =  clk_ctrl & ALLWINNER_H6_PLLX_ENABLE ? 1: 0;
    uint32_t  support_change_rate  =  (clk_id == ALLWINNER_PLL_CPUX) || 
                ( clk_id ==  ALLWINNER_PLL_GPU ) ? 1:  0;

    if ( (clk_ctrl & mask)  ==  flag) {
        _PRINTF_WARN("clk [%s] rate has been set!", allwinner_clk_2_str(clk_id));
        return  0;
    }

    if (clk_enable && !support_change_rate) {
        _PRINTF_ERROR("clk [%s] rate don't support DFS!", allwinner_clk_2_str(clk_id));
        return  -EINVAL;
    }

    return  0;

}


static const struct clk_ops pll_clk_ops = {
    .enable  = _allwinner_h6_pll_enable,
    .disable  = _allwinner_h6_pll_disable,
    .is_enabled  =  _allwinner_h6_pll_enabled,
    .recalc_rate  =  _allwinner_h6_pll_recalc_rate,
    .round_rate  = _allwinner_h6_pll_round_rate,
    .set_rate  =  _allwinner_h6_pll_set_rate,
};


enum {
    ALLWINNER_CLK_N_LOG2 = 0,
    ALLWINNER_CLK_N_FIXED,
    ALLWINNER_CLK_N_NORMAL,
    ALLWINNER_CLK_N_TYPE_MAX = ALLWINNER_CLK_N_NORMAL,
};


static  int32_t  allwinner_clk_get_factor_n(uint32_t clk_id, uint32_t * type, uint32_t * param)
{
    switch (clk_id)
    {
        case  ALLWINNER_CLK_PSI_AHB1_AHB2:
        case  ALLWINNER_CLK_AHB3:
        case  ALLWINNER_CLK_APB1:
        case  ALLWINNER_CLK_APB2:
        case  ALLWINNER_CLK_SMHC0:
        case  ALLWINNER_CLK_SMHC1:
        case  ALLWINNER_CLK_SMHC2:
        case  ALLWINNER_CLK_SPI0:
        case  ALLWINNER_CLK_SPI1:
            *type = ALLWINNER_CLK_N_LOG2;
            *param = 2;
            break;
    
        case  ALLWINNER_CLK_MBUS:
            *type = ALLWINNER_CLK_N_FIXED;
            *param = 1;
            break;

        default:
            return -EINVAL;

    }

    return  0;

}


static  int32_t  allwinner_clk_get_factor_m(uint32_t clk_id, uint32_t * width)
{
    switch (clk_id)
    {
        case  ALLWINNER_CLK_PSI_AHB1_AHB2:
        case  ALLWINNER_CLK_AHB3:
        case  ALLWINNER_CLK_APB1:
        case  ALLWINNER_CLK_APB2:
            *width = 2;
            break;

        case  ALLWINNER_CLK_NAND0:
        case  ALLWINNER_CLK_NAND1:
        case  ALLWINNER_CLK_SMHC0:
        case  ALLWINNER_CLK_SMHC1:
        case  ALLWINNER_CLK_SMHC2:
        case  ALLWINNER_CLK_SPI0:
        case  ALLWINNER_CLK_SPI1:
            *width = 4;
            break;
    
        case  ALLWINNER_CLK_MBUS:
            *width = 3;
            break;

        default:
            return -EINVAL;

    }

    return  0;

}

static  int32_t  allwinner_clk_get_src_select_mask(uint32_t clk_id, uint32_t * width)
{
    switch (clk_id)
    {
        case  ALLWINNER_CLK_PSI_AHB1_AHB2:
        case  ALLWINNER_CLK_AHB3:
        case  ALLWINNER_CLK_APB1:
        case  ALLWINNER_CLK_APB2:
        case  ALLWINNER_CLK_SMHC0:
        case  ALLWINNER_CLK_SMHC1:
        case  ALLWINNER_CLK_SMHC2:

            *width = 2;
            break;

        case  ALLWINNER_CLK_NAND0:
        case  ALLWINNER_CLK_NAND1:
        case  ALLWINNER_CLK_SPI0:
        case  ALLWINNER_CLK_SPI1:
            *width = 4;
            break;
    
        case  ALLWINNER_CLK_MBUS:
            *width = 3;
            break;

        default:
            return -EINVAL;

    }

    return  0;

}




static  int32_t  __allwinner_h6_clk_enable_or_disable(struct clk_hw * hw, uint32_t enable)
{
    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;

    _PRINTF_DBG("enbale pll_clk [%s],\toffset = %#x\n", __clk_get_name(hw->clk), reg_offset);

    uint32_t  flag = readl_relaxed(&clk_hw->map_base[reg_offset]);

    switch (clk_hw->clk_id)
    {
        case  ALLWINNER_CLK_PSI_AHB1_AHB2:
        case  ALLWINNER_CLK_AHB3:
        case  ALLWINNER_CLK_APB1:
        case  ALLWINNER_CLK_APB2:
            return -ENOTSUPP;

    }

    if (enable) {
        flag |=  ALLWINNER_H6_CLK_ENABLE;        
    } else {
        flag &=  ~ALLWINNER_H6_CLK_ENABLE;   
    }
    writel_relaxed(flag,  &clk_hw->map_base[reg_offset]);

    return   0;
}

static  int32_t  _allwinner_h6_clk_enable(struct clk_hw * hw)
{
    return  __allwinner_h6_clk_enable_or_disable(hw, 1);
}

static  int32_t  _allwinner_h6_clk_disable(struct clk_hw * hw)
{
    return  __allwinner_h6_clk_enable_or_disable(hw, 0);
}

static  int32_t  _allwinner_h6_clk_is_enabled(struct clk_hw * hw)
{
    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;

    _PRINTF_DBG("enbale pll_clk [%s],\toffset = %#x\n", __clk_get_name(hw->clk), reg_offset);

    uint32_t  flag = readl_relaxed(&clk_hw->map_base[reg_offset]);

    switch (clk_hw->clk_id)
    {
        case  ALLWINNER_CLK_PSI_AHB1_AHB2:
        case  ALLWINNER_CLK_AHB3:
        case  ALLWINNER_CLK_APB1:
        case  ALLWINNER_CLK_APB2:
            return 1;

    }

    return   flag & ALLWINNER_H6_CLK_ENABLE ? 1: 0;
}


static  unsigned long _allwinner_h6_clk_recalc_rate(struct clk_hw *hw, 
                unsigned long parent_rate)
{
    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;

    _PRINTF_DBG("enbale pll_clk [%s],\toffset = %#x\n", __clk_get_name(hw->clk), reg_offset);

    uint32_t  flag = readl_relaxed(&clk_hw->map_base[reg_offset]);
    uint32_t factor_n = 0, factor_m =  0;

    switch (clk_hw->clk_id) {
        case  ALLWINNER_CLK_PSI_AHB1_AHB2:
        case  ALLWINNER_CLK_AHB3:
        case  ALLWINNER_CLK_APB1:
        case  ALLWINNER_CLK_APB2:
            factor_n  =  1 << ((flag & GENMASK(9, 8)) >> 8);
            factor_m  =  (flag & GENMASK(1, 0) ) + 1;
            break;
        
        default:
            factor_m  = factor_n  =  1;
    }

    unsigned long factor = factor_m * factor_n;
    unsigned long ret = parent_rate / factor;

    return  ret;

}

static  long  _allwinner_h6_clk_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *parent_rate)
{
    // uint64_t  src_rate[ALLWINNER_CLK_SRC_MAX] = {0};
    // uint32_t  src_rate_count = 0;

    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;
    const uint32_t  flag = readl_relaxed(&clk_hw->map_base[reg_offset]);

    uint32_t  src_select_width = 0;
    if (allwinner_clk_get_src_select_mask(clk_hw->clk_id, &src_select_width)) {
        return -EINVAL;
    }
    const uint32_t src_index = (flag >> 24) & GENMASK(src_select_width -1, 0);
    if (src_index >= clk_hw_get_num_parents(hw)) {
        _PRINTF_ERROR("src_index [%u] invalid", src_index);
        return -EINVAL;
    }

    struct clk_hw * parent_clk = clk_hw_get_parent_by_index(hw, src_index);
    if (!parent_clk) {
        _PRINTF_ERROR("get parent clk %s by idx [%u] of clk %s",
                        clk_hw->hw.init->parent_names[src_index]?clk_hw->hw.init->parent_names[src_index]:
                        "null", src_index, clk_hw_get_name(hw));
        return  -EINVAL;
    }

    const unsigned long src_rate = clk_hw_get_rate(parent_clk);
    uint32_t  factorn_type, factorn_param, factorm_width;

    if (allwinner_clk_get_factor_n(clk_hw->clk_id, &factorn_type, &factorn_param)) {
        return  -EINVAL;
    }
    if (allwinner_clk_get_factor_m(clk_hw->clk_id, &factorm_width)) {
        return  -EINVAL;
    }

    long best_rate =  0;
    unsigned long rate_delta = rate;
    for (uint32_t tmp_m = 0; tmp_m < BIT(factorm_width); tmp_m++) {
        uint32_t tmp_n_max = 0, tmp_n_min = 0;
        if (factorn_type == ALLWINNER_CLK_N_FIXED) {
            tmp_n_min = factorn_param;
            tmp_n_max = factorn_param+1;
        } else {
            tmp_n_min = 0;
            tmp_n_max  =  BIT(factorn_param);
        }


        for (uint32_t tmp_n = tmp_n_min;  tmp_n < tmp_n_max; tmp_n++) {

            uint32_t  tmp_factor_n = 0;
            if (factorn_type == ALLWINNER_CLK_N_FIXED) {
                tmp_factor_n = tmp_n;
            } else if ( factorn_type == ALLWINNER_CLK_N_NORMAL) {
                tmp_factor_n = tmp_n + 1;
            } else {
                tmp_factor_n = BIT(tmp_n);
            }

            uint64_t tmp_factor =  (tmp_m + 1 ) * tmp_factor_n;
            uint64_t tmp_rate = src_rate / tmp_factor;

            unsigned long tmp_delta = tmp_rate > rate? tmp_rate - rate: rate - tmp_rate;
            if (tmp_delta < rate_delta) {
                rate_delta  =  tmp_delta;
                best_rate = tmp_rate;
            }

            if (tmp_delta == 0) {
                break;
            }

        }

        if (rate_delta == 0) {
            break;
        }
    }

    *parent_rate  =  src_rate;

    return  best_rate;
}


static int32_t _allwinner_h6_clk_set_parent(struct clk_hw *hw, uint8_t index)
{
    int32_t  ret  =  0;
    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;
    uint32_t  flag = readl_relaxed(&clk_hw->map_base[reg_offset]);

    uint32_t  src_select_width = 0;
    if (allwinner_clk_get_src_select_mask(clk_hw->clk_id, &src_select_width)) {
        return -EINVAL;
    }

    if (index >= clk_hw_get_num_parents(hw)) {
        _PRINTF_ERROR("src_index [%u] invalid", index);
        ret = -EINVAL;
    } 

    const uint32_t mask = GENMASK(23+src_select_width,  24);
    flag &= ~mask;
    flag |=  ((index << 24) & mask);

    writel_relaxed(flag,  &clk_hw->map_base[reg_offset]);
    return  0;

}


static uint8_t	 _allwinner_h6_clk_get_parent(struct clk_hw *hw)
{
    int32_t  ret  =  0;
    allwinner_clk_hw_t * clk_hw  = container_of(hw, allwinner_clk_hw_t, hw);
    uint32_t  reg_offset  =  clk_hw->offset >> 2;
    const uint32_t  flag = readl_relaxed(&clk_hw->map_base[reg_offset]);

    uint32_t  src_select_width = 0;
    if (allwinner_clk_get_src_select_mask(clk_hw->clk_id, &src_select_width)) {
        return -EINVAL;
    }

    const uint32_t src_index = (flag >> 24) & GENMASK(src_select_width -1, 0);
    if (src_index >= clk_hw_get_num_parents(hw)) {
        _PRINTF_ERROR("src_index [%u] invalid", src_index);
        ret = -EINVAL;
    } else {
        ret = src_index;
    }

    return  ret;

}


static	int32_t  _allwinner_h6_clk_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{



    return  0;

}


static const struct clk_ops normal_clk_ops = {
    .enable  = _allwinner_h6_clk_enable,
    .disable  = _allwinner_h6_pll_disable,
    .is_enabled  =  _allwinner_h6_clk_is_enabled,
    .recalc_rate  =  _allwinner_h6_clk_recalc_rate,
    .round_rate  = _allwinner_h6_clk_round_rate,
    .set_parent =  _allwinner_h6_clk_set_parent,
	.get_parent =  _allwinner_h6_clk_get_parent,
    .set_rate  =  _allwinner_h6_clk_set_rate,

};

const char * pll_peri0_12x_parents[1] = { "pll_peri0_4x" };
const char * pll_peri1_12x_parents[1] = { "pll_peri1_4x" };
const char * clk_psi_ahb1_parents[4] = {"osc24", "ccu32k", "rc16m", "pll_peri0_1x"};
const char * clk_ahb3_parents[4] = {"osc24", "ccu32k", "clk_psi_ahb1", "pll_peri0_1x"};
const char * clk_apb1_parents[4] = {"osc24", "ccu32k", "clk_psi_ahb1", "pll_peri0_1x"};
const char * clk_apb2_parents[4] = {"osc24", "ccu32k", "clk_psi_ahb1", "pll_peri0_1x"};


#define CLK_HW_INIT_DATA(name_str, clk_ops,  parent_nr,  parant_name)    {  \
            .name =  name_str,                 \
            .ops  =  clk_ops,                    \
            .num_parents  =  parent_nr,           \
            .parent_names  =  parant_name ,           \
}

#define _CLK_HW_INIT_WITH_PARENTS(clk_id, name, ops)   [clk_id] =   \
                            CLK_HW_INIT_DATA(#name, ops,  \
                            ARRAY_SIZE(name##_parents),  name##_parents )

#define _CLK_HW_INIT_NO_PARENT(clk_id, name, ops)   [clk_id] =   \
                            CLK_HW_INIT_DATA(#name, ops,  \
                            0,  NULL)

#define  CLK_INIT_DATAS     \
    _CLK_HW_INIT_NO_PARENT(ALLWINNER_PLL_CPUX, pll_cpux,  &pll_clk_ops),   \
    _CLK_HW_INIT_NO_PARENT(ALLWINNER_PLL_DDR0, pll_ddr0,  &pll_clk_ops),   \
    _CLK_HW_INIT_NO_PARENT(ALLWINNER_PLL_PERI0_4X, pll_peri0_4x,  &pll_clk_ops),   \
    _CLK_HW_INIT_NO_PARENT(ALLWINNER_PLL_PERI1_4X, pll_peri1_4x,  &pll_clk_ops),   \
                                                                   \
    _CLK_HW_INIT_WITH_PARENTS(ALLWINNER_CLK_PSI_AHB1_AHB2,  clk_psi_ahb1,  &normal_clk_ops),   \
    _CLK_HW_INIT_WITH_PARENTS(ALLWINNER_CLK_AHB3,  clk_ahb3,  &normal_clk_ops),   \
    _CLK_HW_INIT_WITH_PARENTS(ALLWINNER_CLK_APB1,  clk_apb1,  &normal_clk_ops),   \
    _CLK_HW_INIT_WITH_PARENTS(ALLWINNER_CLK_APB2,  clk_apb2,  &normal_clk_ops),


static struct clk_init_data _pll_normal_clk_init[] = {CLK_INIT_DATAS};


#define  ALLWINNER_CLK_HW(clk, reg)   {   \
    .hw.init = &_pll_normal_clk_init[clk],    \
    .clk_id  =  clk,                    \
    .offset  =  reg,            \
}

#define  ALLWINNER_CLKS              \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_CPUX,  0),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_DDR0,  0x10),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI0_4X,  0x20),            \
    ALLWINNER_CLK_HW(ALLWINNER_PLL_PERI1_4X,  0x28),            \
                                                               \
                                                               \
    ALLWINNER_CLK_HW(ALLWINNER_CLK_PSI_AHB1_AHB2,  0x510),            \
    ALLWINNER_CLK_HW(ALLWINNER_CLK_AHB3,  0x51c),            \
    ALLWINNER_CLK_HW(ALLWINNER_CLK_APB1,  0x520),            \
    ALLWINNER_CLK_HW(ALLWINNER_CLK_APB2,  0x524),


static  allwinner_clk_hw_t allwinner_clks[] = { ALLWINNER_CLKS };


typedef struct {
    char * name;
    char * parent;
    uint32_t  clk_id;
    uint32_t  fix_rate;
    uint32_t  flags;
} allwinner_fix_clk_t;

#define  FIX_CLK_DATA(clk_name, clk_parent,  id,  rate, flag)    {   \
    .name  =  clk_name,                \
    .parent  =  clk_parent,             \
    .clk_id  =  id,                  \
    .fix_rate  =  rate,              \
    .flags  =  flag,              \
}

#define  ALLWINNER_FIX_RATE_FLAG              (CLK_IGNORE_UNUSED | CLK_SET_RATE_NO_REPARENT | CLK_IS_CRITICAL )
const static allwinner_fix_clk_t allwinner_fix_clock_init[] = {
    FIX_CLK_DATA("extern_32k",  NULL,  ALLWINNER_FIXED_EXTERN_32K,  32768,  ALLWINNER_FIX_RATE_FLAG),
    FIX_CLK_DATA("RC_16M",  NULL, ALLWINNER_FIXED_RC_16M, 16000000,  ALLWINNER_FIX_RATE_FLAG),
    FIX_CLK_DATA("RXCO_24M",  NULL, ALLWINNER_FIXED_DCXO,  24000000,  ALLWINNER_FIX_RATE_FLAG),
};


static  int32_t  allwinner_fix_rate_clk_probe(void)
{
    struct clk * hw = NULL;
    const uint32_t size  =  ARRAY_SIZE(allwinner_fix_clock_init);

    for (uint32_t i =  0; i < size; i++) {

        hw  =  clk_register_fixed_rate(NULL,  allwinner_fix_clock_init[i].name, 
                            allwinner_fix_clock_init[i].parent, allwinner_fix_clock_init[i].flags,
                            allwinner_fix_clock_init[i].fix_rate);

        if ( IS_ERR(hw) ) {
            _PRINTF_ERROR("register fix rate clk %s failed! ret = %ld\n",  allwinner_fix_clock_init[i].name,
                                PTR_ERR(hw));
            return  PTR_ERR(hw);
        }

        g_clk_info.all_clks[allwinner_fix_clock_init[i].clk_id] = hw;
    }

    return  0;

}


typedef struct {
    char * name;
    char * parent;
    uint32_t  clk_id;
    uint32_t  mult;
    uint32_t  div;
    uint32_t  flags;
} allwinner_fix_factor_clk_t;

#define  FIX_FACTOR_CLK_DATA(clk_name, clk_parent, id, multi,  divisor, flag)    {   \
    .name  =  clk_name,                \
    .parent  =  clk_parent,             \
    .clk_id = id,                       \
    .mult  =  multi,                   \
    .div  =  divisor,             \
    .flags  =  flag,              \
}

#define  ALLWINNER_FIX_FACTOR_FLAG              (CLK_SET_RATE_NO_REPARENT )
const static allwinner_fix_factor_clk_t allwinner_fix_factor_clock_init[] = {
    FIX_FACTOR_CLK_DATA("pll_peri0_2x",  "pll_peri0_4x",  ALLWINNER_PLL_PERI0_2X,  
                                            1,   2,   ALLWINNER_FIX_FACTOR_FLAG),
    FIX_FACTOR_CLK_DATA("pll_peri0_1x",  "pll_peri0_4x",  ALLWINNER_PLL_PERI0_1X,
                                            1,   4,   ALLWINNER_FIX_FACTOR_FLAG),
    FIX_FACTOR_CLK_DATA("pll_peri1_2x",  "pll_peri1_4x",  ALLWINNER_PLL_PERI1_2X, 
                                                            1,   2,   ALLWINNER_FIX_FACTOR_FLAG),
    FIX_FACTOR_CLK_DATA("pll_peri1_1x",  "pll_peri1_4x",  ALLWINNER_PLL_PERI1_1X,  
                                                            1,   4,   ALLWINNER_FIX_FACTOR_FLAG),
};


static  int32_t  allwinner_fix_factor_clk_probe(void)
{
    struct device * dev  = NULL;
    struct clk * hw = NULL;
    const uint32_t size  =  ARRAY_SIZE(allwinner_fix_factor_clock_init);

    for (uint32_t i =  0; i < size; i++) {

        hw  =  clk_register_fixed_factor(dev,  allwinner_fix_factor_clock_init[i].name, 
                            allwinner_fix_factor_clock_init[i].parent, allwinner_fix_factor_clock_init[i].flags,
                            allwinner_fix_factor_clock_init[i].mult, allwinner_fix_factor_clock_init->div);
        // dev_set_name(dev,  allwinner_fix_factor_clock_init[i].name);
        if ( IS_ERR(hw) ) {
            _PRINTF_ERROR("register fix factor clk %s failed! ret = %ld\n",  allwinner_fix_factor_clock_init[i].name,
                                                    PTR_ERR(hw));
            return  PTR_ERR(hw);
        }
        g_clk_info.all_clks[allwinner_fix_factor_clock_init[i].clk_id] = hw;
    }

    return  0;

}

typedef struct {
    char * name;
    char * parent;
    uint32_t  clk_id;
    uint32_t  offset;
    uint32_t  shift;
    uint32_t  flags;
} allwinner_gate_clk_t;

#define  GATE_CLK_DATA(clk_name, clk_parent, id,  off, pos, flag)    {   \
    .name  =  clk_name,                \
    .parent  =  clk_parent,             \
    .clk_id = id,                       \
    .offset  =  off,                   \
    .shift  =  pos,             \
    .flags  =  flag,              \
}

#define  ALLWINNER_GATE_FLAG              (CLK_SET_RATE_NO_REPARENT )
const static allwinner_gate_clk_t allwinner_gate_clock_init[] = {
        GATE_CLK_DATA("uart0",  "clk_apb2",  ALLWINNER_CLK_UART0,  
                            0x90c,   0,   ALLWINNER_GATE_FLAG),
        GATE_CLK_DATA("uart1",  "clk_apb2",  ALLWINNER_CLK_UART0,  
                            0x90c,   1,   ALLWINNER_GATE_FLAG),
        GATE_CLK_DATA("uart2",  "clk_apb2",  ALLWINNER_CLK_UART0,  
                            0x90c,   2,   ALLWINNER_GATE_FLAG),
        GATE_CLK_DATA("uart3",  "clk_apb2",  ALLWINNER_CLK_UART0,  
                            0x90c,   3,   ALLWINNER_GATE_FLAG),
};

static  int32_t  allwinner_gate_clk_probe(void * map_base)
{
    struct device * dev  = NULL;
    struct clk * hw = NULL;
    const uint32_t size  =  ARRAY_SIZE(allwinner_gate_clock_init);

    for (uint32_t i =  0; i < size; i++) {

        hw  =  clk_register_gate(dev,  allwinner_gate_clock_init[i].name, 
                            allwinner_gate_clock_init[i].parent, allwinner_gate_clock_init[i].flags,
                            (allwinner_gate_clock_init[i].offset >> 2) + map_base, allwinner_gate_clock_init[i].shift,
                            0, NULL);
        if ( IS_ERR(hw) ) {
            _PRINTF_ERROR("register gate clk %s failed! ret = %ld\n",  allwinner_gate_clock_init[i].name,
                                                    PTR_ERR(hw));
            return  PTR_ERR(hw);
        }
        g_clk_info.all_clks[allwinner_gate_clock_init[i].clk_id] = hw;
    }

    return  0;

}


#if 0
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
#endif


static struct clk * allwinner_of_clk_get(struct of_phandle_args *clkspec, void * data)
{
	uint32_t idx = clkspec->args[0];
    allwinner_clk_dev_info_t * info = (allwinner_clk_dev_info_t *)data;

	if (idx > ALLWINNER_MAX_CLK_ID ) {
		_PRINTF_ERROR("invalid index %u\n", idx);
		return ERR_PTR(-EINVAL);
	}

	return  info->all_clks[idx];
}


static int32_t  allwinner_h6_ccu_init(struct device_node * node, phys_addr_t phy_addr, void * map)
{
    int32_t  ret  =  0;
    ret  =  allwinner_fix_rate_clk_probe();
    if (ret) {
        return  ret;
    }

    uint32_t size  = ARRAY_SIZE(allwinner_clks);
    for ( uint32_t i =  0; i < size; i++) {
        struct  clk * hw = NULL;

        if ( !allwinner_clks[i].hw.init ) {
            continue;
        }

        allwinner_clks[i].phy_addr  =  phy_addr;
        allwinner_clks[i].map_base  =  map;

        _PRINTF_DBG("clk_name=%s,\tclk_id=%u\n", allwinner_clks[i].hw.init->name, 
                                                    allwinner_clks[i].clk_id);
		hw = clk_register(NULL, &allwinner_clks[i].hw);
		if (IS_ERR(hw)) {
            ret  =  PTR_ERR(hw);
            _PRINTF_ERROR("register clk_hw %s failed! ret = %d\n", allwinner_clks[i].hw.init->name,  ret);
			return ret;
        }
        g_clk_info.all_clks[allwinner_clks[i].clk_id] =  hw;
    }

    ret  =  allwinner_fix_factor_clk_probe();
    if (ret) {
        return  ret;
    }

    ret  =  allwinner_gate_clk_probe(map);
    if (ret) {
        return ret;
    }

    ret  =  of_clk_add_provider(node, allwinner_of_clk_get, &g_clk_info);
    if (ret) {
        _PRINTF_ERROR("of_clk_add_provider failed! ret=%d", ret);
    }

	return   ret;

}


#define  ALLWINNER_CLK_TEST
#ifdef ALLWINNER_CLK_TEST

static int32_t  allwinner_clk_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct  device * dev =  &pdev->dev;
    struct  device_node * dev_node  =  dev->of_node;

    _PRINTF_DBG("probe for pdev [ %s ]\n", pdev->name);

    phys_addr_t  phy_addr =  0;
	struct resource res = {0}, *tmp_res = NULL;

    #if 0
    if (dev_node) {
        if (of_address_to_resource(dev_node, 0, &res)) {
            _PRINTF_ERROR("read clk base addr failed\n");
            return -EINVAL;            
        }
        tmp_res = &res;
    } else {
        tmp_res = platform_get_resource_byname( pdev, IORESOURCE_MEM, 
                                    DEVICE_PHY_ADDR_RESOURCE );
        if (!tmp_res) {
            _PRINTF_ERROR("get platform resource failed!");
        }
    }
    #endif

    tmp_res = &res;
    tmp_res->start = 0x03001000;
    tmp_res->end  = tmp_res->start + ALLWINNER_CCU_MAP_SIZE;
    
    _PRINTF_DBG("phy_addr:\t%#llx -- %#llx\n", tmp_res->start, tmp_res->end);
    phy_addr  =   tmp_res->start;

    void  * map = devm_ioremap(dev, tmp_res->start,  resource_size(tmp_res));
    if (!map) {
        _PRINTF_ERROR("map ccu phy addr to kernel failed!\n");
		return  -EINVAL;
    } else {
        _PRINTF_DBG("map phy_addr %#llx to virt 0x%p\n",   phy_addr,  map );
    }

    ret  =  allwinner_h6_ccu_init(dev_node, phy_addr,  map);

    return  ret;

}


static struct of_device_id allwinner_clk_ids[] =  {
	{.compatible = "allwinner,H6-v200-ccu"},
    {}
};


struct  platform_driver  allwinner_clk_driver = {
    .probe   =  allwinner_clk_probe,
    // .remove	 =  allwinner_clk_remove,
    .driver  =  {
        .name  =  "allwinner_ccu_driver",
        .of_match_table  =  allwinner_clk_ids,
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

#else


static void __init of_allwinner_h6_soc_ccu_setup(struct device_node *np)
{

    phys_addr_t  phy_addr =  0;
    struct resource res = {0};

    if (of_address_to_resource(np, 0, &res)) {
        _PRINTF_ERROR("read clk base addr failed\n");
        return;            
    }

    phy_addr  = res.start;

    void  * map = of_iomap(np,  0);
    if (map == NULL) {
        _PRINTF_ERROR("map ccu phy addr to kernel failed!\n");
		return;
    } else {
        _PRINTF_DBG("map phy_addr %#llx to virt 0x%p\n",   phy_addr,  map );
    }

    allwinner_h6_ccu_init(np, phy_addr,  map);

}


CLK_OF_DECLARE(allwinner_h6_soc_ccu, "allwinner,H6-v200-ccu", 
                    of_allwinner_h6_soc_ccu_setup);


#endif

MODULE_DESCRIPTION("allwinner's soc clk driver");
MODULE_LICENSE("GPL v2");


