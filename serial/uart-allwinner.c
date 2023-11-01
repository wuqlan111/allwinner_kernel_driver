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
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/bitops.h>
#include <linux/compiler-gcc.h>

#include "uart-allwinner.h"

#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)

typedef struct {
    union {
        u32   rbr;
        u32   thr;
        u32   dll;
    };

    union {
        u32   dlh;
	    u32   ier;        
    };

    union {
	    u32   iir;
	    u32   fcr;        
    };
    u32  lcr;
	u32  mcr;
	u32  lsr;
	u32  msr;
	u32  sch;
    u32  rsv1[23];
	u32  usr;
	u32  tfl;
	u32  rfl;
	u32  hsk;
    u32  rsv2[6];
	u32  halt;
    u32  rsv3[2];
    u32  dbg_dll;
    u32  dbg_dlh;
    u32  rsv4[2];
    u32  ctl_485;
    u32  addr_match485;
    u32  bus_ilde_check;
    u32  tx_dly;
} __packed allwinner_h6_uart_t;


typedef struct {
    struct uart_port port;
    phys_addr_t  base_addr;
    struct clk * uart_clk;
} allwinner_uart_port_t;




static struct uart_ops allwinner_uart_ops = {
    // .tx_empty	=  allwinner_uart_tx_empty,
	// .set_mctrl	=  allwinner_uart_set_mctrl,
	// .get_mctrl	=  allwinner_uart_get_mctrl,
	// .stop_tx	=  allwinner_uart_stop_tx,
	// .start_tx	=  allwinner_uart_start_tx,
	// .stop_rx	=  allwinner_uart_stop_rx,
	// .enable_ms	=  allwinner_uart_enable_ms,
	// .break_ctl	=  allwinner_uart_break_ctl,
	// .startup	=  allwinner_uart_startup,
	// .shutdown	=  allwinner_uart_shutdown,
	// .set_termios	=  allwinner_uart_set_termios,
	// .type		= allwinner_uart_type,
	// .release_port	=  allwinner_uart_release_port,
	// .request_port	=  allwinner_uart_request_port,
	// .config_port	=  allwinner_uart_config_port,
	// .verify_port	=  allwinner_uart_verify_port,
};




static struct  uart_driver  uart_driver_reg  =  {
    .owner  =  THIS_MODULE,
	.driver_name  =  "ttyAllwinner",
	.dev_name    =  "ttyAllwinner",
	.major	 =  UART_ALLWINNER_MAJOR,
	.minor	 =  UART_ALLWINNER_MINOR_START,
	.nr	 =  UART_ALLWINNNER_NR_PORTS,
};


static int32_t  allwinner_uart_probe(struct platform_device * pdev)
{
    int32_t ret =  0;
    struct  device * dev =  &pdev->dev;
    struct  device_node * dev_node  =  dev->of_node;

    uint32_t  uart_id, phy_addr;
    if (of_property_read_u32(dev_node, "uart_id", &uart_id)) {
        _PRINTF_ERROR("get uart id failed!\n");
        return  -EINVAL;
    }

    if (uart_id >= UART_ALLWINNNER_NR_PORTS) {
        _PRINTF_ERROR("uart_id %u invalid\n", uart_id);
        return  -EINVAL;
    }

    if (of_property_read_u32(dev_node, "uart_id", &phy_addr)) {
        _PRINTF_ERROR("get uart base phy addr failed!\n");
        return  -EINVAL;
    }

    allwinner_uart_port_t * port = devm_kzalloc(dev, sizeof(allwinner_uart_port_t), GFP_KERNEL);
    if (!port) {
        _PRINTF_ERROR("alloc uart port failed\n");
        return -ENOMEM;
    }

    memset(port,  0,  sizeof(allwinner_uart_port_t));

    port->uart_clk  =  of_clk_get(dev_node, 0);
    if (port->uart_clk == ERR_PTR(-ENOENT)) {
        return -ENOENT;
    }

    port->base_addr  =  phy_addr;
    port->port.line  =  uart_id;
    port->port.ops   =  &allwinner_uart_ops;
    port->port.dev   =  dev;
    port->port.membase  =  devm_ioremap(dev,  phy_addr,  UART_ALLWINNER_MAP_SIZE);
    port->port.mapbase  =  phy_addr;
    port->port.mapsize  =  UART_ALLWINNER_MAP_SIZE;

    ret  =  uart_add_one_port(&uart_driver_reg,  &port->port);
    if (ret) {
        _PRINTF_ERROR("add uart port to uart driver failed! ret = %d\n",  ret);
    }

    return   ret;

}


static int32_t allwinner_uart_remove(struct platform_device * pdev)
{
    int32_t  ret  =  0;
	allwinner_uart_port_t * port = platform_get_drvdata(pdev);

	if (port) {
		ret = uart_remove_one_port(&uart_driver_reg, &port->port);        
    }

	return  ret;
}


static struct of_device_id allwinner_uart_ids[] =  {
	{.compatible = "allwinner,H6-v200-uart"},
    {}
};


struct  platform_driver  allwinner_uart_driver = {
    .probe   =  allwinner_uart_probe,
    .remove	 =  allwinner_uart_remove,
    .driver  =  {
        .name  =  "allwinner_uart_driver",
        .of_match_table  =  allwinner_uart_ids,
    }

};


static int32_t  __init  allwinner_uart_init(void)
{
    int32_t ret =  0;

    ret   =  uart_register_driver(&uart_driver_reg);
    if (ret) {
        _PRINTF_ERROR("register allwinner uart core driver failed! ret = %d\n", ret);
        return  ret;
    }

    ret = platform_driver_register(&allwinner_uart_driver);
    if (ret) {
        _PRINTF_ERROR("register uart driver failed! ret = %d\n", ret);
        uart_unregister_driver(&uart_driver_reg);
    } else {
        _PRINTF_INFO("register allwinner uart driver success!\n");
    }

    return  ret;

}


static void  __exit  allwinner_uart_exit(void)
{
    platform_driver_unregister(&allwinner_uart_driver);
    uart_unregister_driver(&uart_driver_reg);
    _PRINTF_INFO("allwinner uart driver remove!\n");

}


module_init(allwinner_uart_init);
module_exit(allwinner_uart_exit);

MODULE_DESCRIPTION("allwinner's soc uart driver");
MODULE_LICENSE("GPL v2");


