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
    struct uart_port port;
} allwinner_uart_port_t;



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

    return   ret;

}


static struct of_device_id allwinner_uart_ids[] =  {
	{.compatible = "allwinner,H6-v200-uart"},
    {}
};


struct  platform_driver  allwinner_uart_driver = {
    .probe   =  allwinner_uart_probe,
    .remove  =  NULL,
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


