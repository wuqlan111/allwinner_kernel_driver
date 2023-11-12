#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
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
#include <linux/clockchips.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "timer-allwinner.h"

#define  _PRINTF_DBG(fmt, args...)       pr_debug("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)


typedef  struct {
    uint32_t  ctrl;
    uint32_t  intv;
    uint32_t  cur;
} __packed  allwinner_timer_cfg_t;

typedef  struct {
    uint32_t  irq_en;
    uint32_t  irq_sr;
    allwinner_timer_cfg_t  timers[ALLWINNER_TIMER_NUMBER];
} __packed  allwinner_timer_t;


typedef  struct {
    struct  clock_event_device  clk_dev;
    uint32_t  timer_id;
    void * plat;
} allwinner_clock_event_device_t;


typedef  struct {
    phys_addr_t  phy_addr;
    uint32_t  * map_base;
    allwinner_clock_event_device_t  timer_devs[ALLWINNER_TIMER_NUMBER];
} allwinner_timer_plat_t;


static allwinner_clock_event_device_t * allwinner_timer_dev_get(struct clock_event_device * dev)
{
    return  container_of(dev, allwinner_clock_event_device_t,  clk_dev); 
}

// static int32_t			(*set_state_periodic)(struct clock_event_device *);
	// int			(*set_state_oneshot)(struct clock_event_device *);
	// int			(*set_state_oneshot_stopped)(struct clock_event_device *);
static 	int32_t   allwinner_timer_set_state_shutdown(struct clock_event_device * dev)
{
    allwinner_clock_event_device_t * timer_dev  =  allwinner_timer_dev_get(dev);
    allwinner_timer_plat_t * plat = timer_dev->plat;
    allwinner_timer_t  * regs =  (allwinner_timer_t  *)plat->map_base;
    const uint32_t timer_id  =  timer_dev->timer_id;

    uint32_t  flag  =  readl_relaxed(&regs->timers[timer_id].ctrl);
    flag &= ~ALLWINNER_TIMER_CTRL_EN;
    writel_relaxed(flag,  &regs->timers[timer_id].ctrl);
    return  0;

}

static  int32_t	 allwinner_timer_tick_resume(struct clock_event_device * dev)
{
    allwinner_clock_event_device_t * timer_dev  =  allwinner_timer_dev_get(dev);
    allwinner_timer_plat_t * plat = timer_dev->plat;
    allwinner_timer_t  * regs =  (allwinner_timer_t  *)plat->map_base;
    const uint32_t timer_id  =  timer_dev->timer_id;

    uint32_t  flag  =  readl_relaxed(&regs->timers[timer_id].ctrl);
    flag &= ~ALLWINNER_TIMER_CTRL_RELOAD;
    flag |= ALLWINNER_TIMER_CTRL_EN;
    writel_relaxed(flag,  &regs->timers[timer_id].ctrl);
    return  0;

}


static  void  allwinner_timer_suspend(struct clock_event_device * dev)
{
    allwinner_clock_event_device_t * timer_dev  =  allwinner_timer_dev_get(dev);
    allwinner_timer_plat_t * plat = timer_dev->plat;
    allwinner_timer_t  * regs =  (allwinner_timer_t  *)plat->map_base;
    const uint32_t timer_id  =  timer_dev->timer_id;

    uint32_t  flag  =  readl_relaxed(&regs->timers[timer_id].ctrl);
    flag &= ~ALLWINNER_TIMER_CTRL_EN;
    writel_relaxed(flag,  &regs->timers[timer_id].ctrl);

}


static  void  allwinner_timer_resume(struct clock_event_device * dev)
{
    allwinner_clock_event_device_t * timer_dev  =  allwinner_timer_dev_get(dev);
    allwinner_timer_plat_t * plat = timer_dev->plat;
    allwinner_timer_t  * regs =  (allwinner_timer_t  *)plat->map_base;
    const uint32_t timer_id  =  timer_dev->timer_id;

    uint32_t  flag  =  readl_relaxed(&regs->timers[timer_id].ctrl);
    flag &= ~ALLWINNER_TIMER_CTRL_RELOAD;
    flag |= ALLWINNER_TIMER_CTRL_EN;
    writel_relaxed(flag,  &regs->timers[timer_id].ctrl);

}



static int32_t __init allwinner_timer_init(struct device_node *np)
{
    int32_t ret =  0;
    uint32_t  phy_addr =  0;
    uint32_t  * map_base = NULL;

    if (of_property_read_u32(np, "addr", &phy_addr)) {
        return -EINVAL;
    }

    map_base  =  (uint32_t *)ioremap(phy_addr,  ALLWINNER_TIMER_MAP_SIZE);
    if (!map_base) {
        _PRINTF_ERROR("ioremap phy_addr %#08x to virt failed!\n", phy_addr);
        return  -EINVAL;
    }

    allwinner_timer_plat_t  * timer_plat  = kzalloc(sizeof(allwinner_timer_plat_t), GFP_KERNEL);
    memset(timer_plat, 0, sizeof(allwinner_timer_plat_t));

    for (int32_t i = 0; i < ALLWINNER_TIMER_NUMBER; i++) {
        char timer_name[64] = {0};
        sprintf(timer_name, "allwinner-timer%d", i);
        timer_plat->timer_devs[i].timer_id  =  i;
        timer_plat->timer_devs[i].clk_dev.name  =  kstrdup(timer_name, GFP_KERNEL);
        timer_plat->timer_devs[i].plat  =  timer_plat;

        timer_plat->timer_devs[i].clk_dev.suspend  =  allwinner_timer_suspend;
        timer_plat->timer_devs[i].clk_dev.resume  =  allwinner_timer_resume;

        clockevents_config_and_register(&timer_plat->timer_devs[i].clk_dev,  0,  0, 0);
    }

	return   ret;

}


MODULE_DESCRIPTION("allwinner's soc timer driver");
MODULE_LICENSE("GPL v2");

TIMER_OF_DECLARE(allwinner_timer, "allwinner,H6-v200-timer",
		       allwinner_timer_init);


