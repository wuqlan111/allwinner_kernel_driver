#ifndef  GPIO_ALLWINNER_H6_H
#define  GPIO_ALLWINNER_H6_H

#include  <linux/bitops.h>

#define  ARRAY_V(idx, v)   [idx] = v 

#define  ALLWINNER_CLK_REG_INVALID           (0xffffffu)


// pll clk
#define  ALLWINNER_PLL_CPUX         0
#define  ALLWINNER_PLL_DDR0         1
#define  ALLWINNER_PLL_PERI0_4X     2
#define  ALLWINNER_PLL_PERI0_2X     3
#define  ALLWINNER_PLL_PERI0_1X     4
#define  ALLWINNER_PLL_PERI1_4X     5
#define  ALLWINNER_PLL_PERI1_2X     6
#define  ALLWINNER_PLL_PERI1_1X     7
#define  ALLWINNER_PLL_GPU          8
#define  ALLWINNER_PLL_VIDEO0_4X    9
#define  ALLWINNER_PLL_VIDEO0_1X    10
#define  ALLWINNER_PLL_VIDEO1_4X    11
#define  ALLWINNER_PLL_VIDEO1_1X    12
#define  ALLWINNER_PLL_VE           13
#define  ALLWINNER_PLL_DE           14
#define  ALLWINNER_PLL_HSIC         15
#define  ALLWINNER_PLL_AUDIO_4X     16
#define  ALLWINNER_PLL_AUDIO        17

// clk
#define  ALLWINNER_CLK_CPUX_AXI                  18
#define  ALLWINNER_CLK_CPUX_APB                  19
#define  ALLWINNER_CLK_PSI_AHB1_AHB2             20
#define  ALLWINNER_CLK_AHB3                      21
#define  ALLWINNER_CLK_APB1                      22
#define  ALLWINNER_CLK_APB2                      23
#define  ALLWINNER_CLK_MBUS                      24
#define  ALLWINNER_CLK_NAND0                     25
#define  ALLWINNER_CLK_NAND1                     26
#define  ALLWINNER_CLK_SMHC0                     27
#define  ALLWINNER_CLK_SMHC1                     28
#define  ALLWINNER_CLK_SMHC2                     29
#define  ALLWINNER_CLK_SPI0                      30
#define  ALLWINNER_CLK_SPI1                      31
#define  ALLWINNER_CLK_TS                        32
#define  ALLWINNER_CLK_CIR_TX                    33
#define  ALLWINNER_CLK_I2S_PCM3                  34
#define  ALLWINNER_CLK_I2S_PCM0                  35
#define  ALLWINNER_CLK_I2S_PCM1                  36
#define  ALLWINNER_CLK_I2S_PCM2                  37
#define  ALLWINNER_CLK_OWA                       38
#define  ALLWINNER_CLK_DMIC                      39
#define  ALLWINNER_CLK_AUDIO_HUB                 40
#define  ALLWINNER_CLK_USB0                      41
#define  ALLWINNER_CLK_USB1                      42
#define  ALLWINNER_CLK_USB3                      43
#define  ALLWINNER_CLK_PCIE_REF                  44
#define  ALLWINNER_CLK_PCIE_MAXI                 45
#define  ALLWINNER_CLK_PCIE_AUX                  46
#define  ALLWINNER_CLK_HDMI                      47
#define  ALLWINNER_CLK_HDMI_SLOW                 48
#define  ALLWINNER_CLK_HDMI_CEC                  49
#define  ALLWINNER_CLK_HDMI_HDCP                 50
#define  ALLWINNER_CLK_TCON_LED                  51
#define  ALLWINNER_CLK_TCON_TV                   52
#define  ALLWINNER_CLK_CSI_MISC                  53
#define  ALLWINNER_CLK_CSI_TOP                   54
#define  ALLWINNER_CLK_CSI_MASTER                55
#define  ALLWINNER_CLK_UART0                     56
#define  ALLWINNER_CLK_UART1                     57
#define  ALLWINNER_CLK_UART2                     58
#define  ALLWINNER_CLK_UART3                     59

#define  ALLWINNER_PLL_MAX         ALLWINNER_PLL_AUDIO
#define  ALLWINNER_CLK_UARTX       ALLWINNER_CLK_APB2

#define  foreach_array_clk_ctrl    \
    ARRAY_V(ALLWINNER_PLL_CPUX,            0),       \
    ARRAY_V(ALLWINNER_PLL_DDR0,            0x10),       \
    ARRAY_V(ALLWINNER_PLL_PERI0_4X,        0x20),       \
    ARRAY_V(ALLWINNER_PLL_PERI0_2X,        0x20),       \
    ARRAY_V(ALLWINNER_PLL_PERI0_1X,        0x20),       \
    ARRAY_V(ALLWINNER_PLL_PERI1_4X,        0x28),       \
    ARRAY_V(ALLWINNER_PLL_PERI1_2X,        0x28),       \
    ARRAY_V(ALLWINNER_PLL_PERI1_1X,        0x28),       \
    ARRAY_V(ALLWINNER_PLL_GPU,             0x30),       \
    ARRAY_V(ALLWINNER_PLL_VIDEO0_4X,       0x40),       \
    ARRAY_V(ALLWINNER_PLL_VIDEO0_1X,       0x40),       \
    ARRAY_V(ALLWINNER_PLL_VIDEO1_4X,       0x48),       \
    ARRAY_V(ALLWINNER_PLL_VIDEO1_1X,       0x48),       \
    ARRAY_V(ALLWINNER_PLL_VE,              0x58),       \
    ARRAY_V(ALLWINNER_PLL_DE,              0x60),       \
    ARRAY_V(ALLWINNER_PLL_HSIC,            0x70),       \
    ARRAY_V(ALLWINNER_PLL_AUDIO_4X,        0x78),       \
    ARRAY_V(ALLWINNER_PLL_AUDIO,           0x78),       \
    ARRAY_V(ALLWINNER_CLK_CPUX_AXI,        0x500),       \
    ARRAY_V(ALLWINNER_CLK_PSI_AHB1_AHB2,         0x510),        \
    ARRAY_V(ALLWINNER_CLK_AHB3,         0x51c),        \
    ARRAY_V(ALLWINNER_CLK_APB1,         0x520),        \
    ARRAY_V(ALLWINNER_CLK_APB2,         0x524),        \
    ARRAY_V(ALLWINNER_CLK_MBUS,         0x540),        \
    ARRAY_V(ALLWINNER_CLK_NAND0,        0x810),        \
    ARRAY_V(ALLWINNER_CLK_NAND1,        0x814),        \
    ARRAY_V(ALLWINNER_CLK_SMHC0,        0x830),        \
    ARRAY_V(ALLWINNER_CLK_SMHC1,        0x834),        \
    ARRAY_V(ALLWINNER_CLK_SMHC2,        0x838),        \
    ARRAY_V(ALLWINNER_CLK_SPI0,         0x940),        \
    ARRAY_V(ALLWINNER_CLK_SPI1,         0x944),
    // ARRAY_V(ALLWINNER_CLK_SMHC2,        0x838),
    // ARRAY_V(ALLWINNER_CLK_SMHC2,        0x838),
    // ARRAY_V(ALLWINNER_CLK_SMHC2,        0x838),
    // ARRAY_V(ALLWINNER_CLK_SMHC2,        0x838),
    // ARRAY_V(ALLWINNER_CLK_SMHC2,        0x838),


// #define  foreach_array_clk_pattern0   
//     ARRAY_V(ALLWINNER_PLL_CPUX,         ALLWINNER_CLK_REG_INVALID),       
//     ARRAY_V(ALLWINNER_PLL_DDR0,         0x110),       
//     ARRAY_V(ALLWINNER_PLL_PERI0,        ALLWINNER_CLK_REG_INVALID),      
//     ARRAY_V(ALLWINNER_PLL_PERI1,        0x128),       
//     ARRAY_V(ALLWINNER_PLL_GPU,          0x130),       
//     ARRAY_V(ALLWINNER_PLL_VIDEO0,       0x140),       
//     ARRAY_V(ALLWINNER_PLL_VIDEO1,       0x148),       
//     ARRAY_V(ALLWINNER_PLL_VE,           0x158),       
//     ARRAY_V(ALLWINNER_PLL_DE,           0x160),       
//     ARRAY_V(ALLWINNER_PLL_HSIC,         0x170),       
//     ARRAY_V(ALLWINNER_PLL_AUDIO,        0x178),


#define  ALLWINNER_H6_PLLX_ENABLE              BIT(31)
#define  ALLWINNER_H6_PLLX_LOCK_ENABLE         BIT(29)
#define  ALLWINNER_H6_PLLX_LOCK                BIT(28)
#define  ALLWINNER_H6_PLLX_FACTOR_N            GENMASK(15,   8)






#endif
