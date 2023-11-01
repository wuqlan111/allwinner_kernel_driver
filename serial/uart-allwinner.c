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


static void _allwinner_serial_set_or_clean_dlab(allwinner_h6_uart_t * const uart_reg, const uint32_t set)
{
	while(readl_relaxed(&uart_reg->usr) & ALLWINNER_UART_SR_BUSY) ;
    uint32_t  flag  =  readl_relaxed(&uart_reg->lcr);

	if (set) {
		flag  |= ALLWINNER_UART_LCR_DLAB;
	} else {
		flag  &= ~ALLWINNER_UART_LCR_DLAB;
	}
    writel_relaxed(flag,  &uart_reg->lcr);
}


static int _allwinner_h6_serial_setbrg(allwinner_h6_uart_t * const uart_reg, 
										const uint64_t clk_rate, const int32_t baudrate)
{

	uint64_t  tmp_baudrate  =  baudrate  << 4;
	uint64_t  divisor  =  clk_rate  / tmp_baudrate;
	uint64_t  integer_part  =  divisor;

	if ( (baudrate <= 0)  ||  !divisor ) {
		_PRINTF_ERROR("baudrate[%d] invalid!\n", baudrate);
		return  -EINVAL;
	}

	if (clk_rate % tmp_baudrate) {
		ulong tmp2 =  baudrate * (divisor + 1) * (divisor << 5);
		integer_part  =  clk_rate >= tmp2 ? divisor+1: divisor;
	}

	if (integer_part > 0xffff) {
		_PRINTF_ERROR("baudrate[%d] is too small!\n", baudrate);
		return  -EINVAL;
	}

	_allwinner_serial_set_or_clean_dlab(uart_reg,  1);

	writel_relaxed(integer_part & 0xff,  &uart_reg->dll);
	writel_relaxed(integer_part >> 8,   &uart_reg->dlh);

	_allwinner_serial_set_or_clean_dlab(uart_reg,  0);

	return 0;

}

#define  UARTX_FIFO_INIT_FLAG    (ALLWINNER_UART_FCR_FIFOE | ALLWINNER_UART_FCR_RFIFOR \
									| ALLWINNER_UART_FCR_XFIFOR)
#define  UARTX_MCR_INIT_FLAG     (ALLWINNER_UART_MCR_DTR | ALLWINNER_UART_MCR_RTS)

static  int32_t  _allwinner_h6_serial_init(allwinner_h6_uart_t * const uart_reg,
			const ulong usart_clk_rate, const int32_t baudrate)
{
	int32_t  ret = 0;

	_allwinner_serial_set_or_clean_dlab(uart_reg,  0);

	writel_relaxed(0, &uart_reg->ier);
	writel_relaxed(UARTX_FIFO_INIT_FLAG, &uart_reg->fcr);
	writel_relaxed(0x3,  &uart_reg->lcr);
	writel_relaxed(UARTX_MCR_INIT_FLAG,  &uart_reg->mcr);

	ret = _allwinner_h6_serial_setbrg(uart_reg, usart_clk_rate, baudrate);

	return  ret;

}

static int32_t _allwinner_h6_serial_getc(allwinner_h6_uart_t * const uart_reg)
{
	uint32_t  lsr = readl_relaxed(&uart_reg->lsr);

	if ( ! (lsr & ALLWINNER_UART_LSR_DR)) {
		return  -EAGAIN;
	}

	return  readl_relaxed(&uart_reg->rbr);
}



static int _allwinner_h6_serial_putc(allwinner_h6_uart_t * const uart_reg, const char ch)
{
	uint32_t  lsr = readl_relaxed(&uart_reg->lsr);

	if ( ! (lsr & ALLWINNER_UART_LSR_THRE)) {
		return  -EAGAIN;
	}

    _allwinner_serial_set_or_clean_dlab(uart_reg,  0);
	writel(ch,  &uart_reg->thr);

	return  0;
}


static int _allwinner_h6_serial_pending(allwinner_h6_uart_t *  const uart_reg, const uint32_t input)
{
	uint32_t  pending =  0;
	uint32_t  lsr = readl_relaxed(&uart_reg->lsr); 
	if (input) {
		pending = lsr & ALLWINNER_UART_LSR_DR ? 1 : 0;	
	} else {
		pending = lsr & ALLWINNER_UART_LSR_THRE ? 0 : 1;	
	}

	return  pending;

}

static uint32_t allwinner_uart_tx_empty(struct uart_port *port)
{
	allwinner_uart_port_t * uart_port = container_of(port, allwinner_uart_port_t, port);
    allwinner_h6_uart_t * regs = (allwinner_h6_uart_t *)uart_port->port.membase;

    uint32_t  ret  =  readl_relaxed(&regs->lsr) & ALLWINNER_UART_LSR_THRE? 1: 0;
	return  ret;
}

static uint32_t allwinner_uart_get_mctrl(struct uart_port *port)
{
    allwinner_uart_port_t * uart_port = container_of(port, allwinner_uart_port_t, port);
    allwinner_h6_uart_t * regs = (allwinner_h6_uart_t *)uart_port->port.membase;

    return  0;
}

static void allwinner_uart_set_mctrl(struct uart_port *port, uint32_t mctrl)
{


}

static void allwinner_uart_enable_ms(struct uart_port *port)
{

}


static void allwinner_uart_break_ctl(struct uart_port *port, int32_t break_state)
{
    allwinner_uart_port_t * uart_port = container_of(port, allwinner_uart_port_t, port);
    allwinner_h6_uart_t * regs = (allwinner_h6_uart_t *)uart_port->port.membase;

    uint32_t  flag  = readl_relaxed(&regs->lcr);
	if (break_state == -1) {
		flag |= ALLWINNER_UART_LCR_BC;        
    } else {
		flag &= ~ALLWINNER_UART_LCR_BC;        
    }
    writel_relaxed(flag,  &regs->lcr);

}

static int32_t allwinner_uart_startup(struct uart_port *port)
{
    allwinner_uart_port_t * uart_port = container_of(port, allwinner_uart_port_t, port);
    allwinner_h6_uart_t * regs = (allwinner_h6_uart_t *)uart_port->port.membase;



    return  0;
}

static void allwinner_uart_shutdown(struct uart_port *port)
{
    allwinner_uart_port_t * uart_port = container_of(port, allwinner_uart_port_t, port);
    allwinner_h6_uart_t * regs = (allwinner_h6_uart_t *)uart_port->port.membase;

}

static void  allwinner_uart_set_termios(struct uart_port *port, struct ktermios *termios,
		   struct ktermios *old)
{
    allwinner_uart_port_t * uart_port = container_of(port, allwinner_uart_port_t, port);
    allwinner_h6_uart_t * regs = (allwinner_h6_uart_t *)uart_port->port.membase;

}

static const char * allwinner_uart_type(struct uart_port *port)
{
    allwinner_uart_port_t * uart_port = container_of(port, allwinner_uart_port_t, port);

	return  uart_port->port.type == PORT_SUNIX ? "Allwinner" : NULL;
}


static void allwinner_uart_release_port(struct uart_port *port)
{

}

static int allwinner_uart_request_port(struct uart_port *port)
{

}


static void allwinner_uart_config_port(struct uart_port *port, int32_t flags)
{
    allwinner_uart_port_t * uart_port = container_of(port, allwinner_uart_port_t, port);
    allwinner_h6_uart_t * regs = (allwinner_h6_uart_t *)uart_port->port.membase;

	if (flags & UART_CONFIG_TYPE ) {
		uart_port->port.type = PORT_SUNIX;        
    }

}

static int32_t allwinner_uart_verify_port(struct uart_port *port, struct serial_struct *ser)
{
    allwinner_uart_port_t * uart_port = container_of(port, allwinner_uart_port_t, port);
    allwinner_h6_uart_t * regs = (allwinner_h6_uart_t *)uart_port->port.membase;
	int32_t ret = 0;

	if (ser->type != PORT_UNKNOWN && ser->type != PORT_SUNIX) {
		ret = -EINVAL;
        goto  end; 
    }

end:
	return  ret;
}

static struct uart_ops allwinner_uart_ops = {
    .tx_empty	=  allwinner_uart_tx_empty,
	.set_mctrl	=  allwinner_uart_set_mctrl,
	.get_mctrl	=  allwinner_uart_get_mctrl,
	// .stop_tx	=  allwinner_uart_stop_tx,
	// .start_tx	=  allwinner_uart_start_tx,
	// .stop_rx	=  allwinner_uart_stop_rx,
	.enable_ms	=  allwinner_uart_enable_ms,
	.break_ctl	=  allwinner_uart_break_ctl,
	.startup	=  allwinner_uart_startup,
	.shutdown	=  allwinner_uart_shutdown,
	.set_termios	=  allwinner_uart_set_termios,
	.type		= allwinner_uart_type,
	.release_port	=  allwinner_uart_release_port,
	.request_port	=  allwinner_uart_request_port,
	.config_port	=  allwinner_uart_config_port,
	.verify_port	=  allwinner_uart_verify_port,
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


