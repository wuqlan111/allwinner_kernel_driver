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
#include <linux/clocksource.h>
#include <linux/compiler-gcc.h>
#include <linux/delay.h>

#include "timer-allwinner.h"
#include "dbg_log.h"


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
    uint32_t  ctrl;
    uint32_t  low;
    uint32_t  high;
} __packed  allwinner_counter64_t;


typedef  struct {
    struct  clock_event_device  clk_dev;
    uint32_t  timer_id;
    void * plat;
} allwinner_clock_event_device_t;

typedef  struct {
    struct  clocksource  cs;
    void * plat;
} allwinner_clocksource_t;

typedef  struct {
    phys_addr_t  phy_addr;
    uint32_t  * map_base;
    allwinner_clock_event_device_t  timer_devs[ALLWINNER_TIMER_NUMBER];
    allwinner_clocksource_t  counter64;
    uint32_t  timers_offset;
    uint32_t  counter64_offset;
} allwinner_timer_plat_t;


static allwinner_clock_event_device_t * allwinner_timer_dev_get(struct clock_event_device * dev)
{
    return  container_of(dev, allwinner_clock_event_device_t,  clk_dev); 
}

static allwinner_clocksource_t * allwinner_clocksource_get(struct clocksource * sc)
{
    return  container_of(sc, allwinner_clocksource_t,  cs); 
}


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

static void _timer_set_mode(allwinner_timer_cfg_t * cfg, uint32_t oneshot)
{
    uint32_t flag = readl_relaxed(&cfg->ctrl);
    if (oneshot) {
        flag |= ALLWINNER_TIMER_CTRL_MODE;        
    } else {
        flag &= ~ALLWINNER_TIMER_CTRL_MODE;
    }
    writel_relaxed(flag,  &cfg->ctrl);
}


static void _timer_set_interval(allwinner_timer_cfg_t * cfg, unsigned long cycles)
{
    uint32_t ctrl_flag = readl_relaxed(&cfg->ctrl);

    writel_relaxed(cycles, &cfg->intv);
    ctrl_flag |= ALLWINNER_TIMER_CTRL_RELOAD;
    writel_relaxed(ctrl_flag,  &cfg->ctrl);
}

static int32_t allwinner_timer_set_state_periodic(struct clock_event_device *evt)
{
    allwinner_clock_event_device_t * dev = allwinner_timer_dev_get(evt);
	allwinner_timer_plat_t * plat = dev->plat;

    const uint32_t timer_id = dev->timer_id;
    const uint32_t offset = plat->timers_offset >> 2;
    allwinner_timer_t * regs = (allwinner_timer_t *)&plat->map_base[offset];
    _timer_set_mode(&regs->timers[timer_id], 0);
	return  0;
}

static int32_t allwinner_timer_set_state_oneshot(struct clock_event_device *evt)
{
    allwinner_clock_event_device_t * dev = allwinner_timer_dev_get(evt);
	allwinner_timer_plat_t * plat = dev->plat;

    const uint32_t timer_id = dev->timer_id;
    const uint32_t offset = plat->timers_offset >> 2;
    allwinner_timer_t * regs = (allwinner_timer_t *)&plat->map_base[offset];
    _timer_set_mode(&regs->timers[timer_id], 1);
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


static int32_t allwinner_timer_set_next_event(unsigned long cycles,
					struct clock_event_device *evt)
{
    allwinner_clock_event_device_t * timer_dev  =  allwinner_timer_dev_get(evt);
    allwinner_timer_plat_t * plat = timer_dev->plat;
    const uint32_t offset = plat->timers_offset >> 2;
    allwinner_timer_t  * regs =  (allwinner_timer_t  *)&plat->map_base[offset];
    const uint32_t timer_id  =  timer_dev->timer_id;

    _timer_set_interval(&regs->timers[timer_id], cycles);

	return 0;
}


static int32_t __init allwinner_setup_clockevent(allwinner_timer_plat_t * timer_plat, uint32_t timer_id)
{
    const uint32_t offset = timer_plat->timers_offset >> 2;
    allwinner_timer_t  * regs =  (allwinner_timer_t  *)&timer_plat->map_base[offset];

    char timer_name[64] = {0};
    sprintf(timer_name, "allwinner-timer%d", timer_id);
    timer_plat->timer_devs[timer_id].timer_id  =  timer_id;
    timer_plat->timer_devs[timer_id].clk_dev.name  =  kstrdup(timer_name, GFP_KERNEL);
    timer_plat->timer_devs[timer_id].plat  =  timer_plat;

    timer_plat->timer_devs[timer_id].clk_dev.features  = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;
    timer_plat->timer_devs[timer_id].clk_dev.suspend  =  allwinner_timer_suspend;
    timer_plat->timer_devs[timer_id].clk_dev.resume  =  allwinner_timer_resume;
    timer_plat->timer_devs[timer_id].clk_dev.tick_resume  =  allwinner_timer_tick_resume;
    timer_plat->timer_devs[timer_id].clk_dev.set_state_oneshot  =  allwinner_timer_set_state_oneshot;
    timer_plat->timer_devs[timer_id].clk_dev.set_state_periodic  =  allwinner_timer_set_state_periodic;
    timer_plat->timer_devs[timer_id].clk_dev.set_state_shutdown  =  allwinner_timer_set_state_shutdown;
    timer_plat->timer_devs[timer_id].clk_dev.rating = 24000000;

    clockevents_config_and_register(&timer_plat->timer_devs[timer_id].clk_dev,  24000000,  1, 0xffffffff);

    return  0;
}

static u64 allwinner_counter64_clocksource_read(struct clocksource *cs)
{
	allwinner_clocksource_t * allwinner_cs = allwinner_clocksource_get(cs);
    allwinner_timer_plat_t * plat  = allwinner_cs->plat;

    const uint32_t offset = plat->counter64_offset >> 2;
    allwinner_counter64_t * regs = (allwinner_counter64_t *)&plat->map_base[offset];

    u64  low   =  readl_relaxed(&regs->low);
    u64  high  =  readl_relaxed(&regs->high);

	return   (high << 32) | low;
}

static int32_t __init allwinner_setup_counter64(allwinner_timer_plat_t * timer_plat)
{

    int32_t  ret  =  0;

    timer_plat->counter64.cs.name = "allwinner_counter64_clocksource";
    timer_plat->counter64.cs.rating  =  24000000;
	timer_plat->counter64.cs.read = allwinner_counter64_clocksource_read;
    timer_plat->counter64.cs.mask  =  CLOCKSOURCE_MASK(64);
    timer_plat->counter64.cs.flags  =  CLOCK_SOURCE_IS_CONTINUOUS;

    timer_plat->counter64.plat  =  timer_plat;

    const uint32_t offset = timer_plat->counter64_offset >> 2;
    allwinner_counter64_t * regs = (allwinner_counter64_t * )&timer_plat->map_base[offset];

	ret = clocksource_register_hz(&timer_plat->counter64.cs, 24000000);
	if (ret) {
        _PRINTF_ERROR("register clocksource for counter64 failed! ret=%d\n", ret);
	}

    writel_relaxed(ALLWINNER_CNT64_CTRL_CLEAR, &regs->ctrl);

	return  ret;

}


static int32_t __init allwinner_h6_timer_init(struct device_node *np)
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

    timer_plat->counter64_offset  = ALLWINNER_H6_COUNTER64_OFFSET;
    timer_plat->timers_offset  = ALLWINNER_H6_TIMER_OFFSET;
    timer_plat->phy_addr  =  phy_addr;
    timer_plat->map_base  =  map_base;

    for (int32_t i = 0; i < ALLWINNER_TIMER_NUMBER; i++) {
        ret  =  allwinner_setup_clockevent(timer_plat, i);
        if (ret) {
            _PRINTF_ERROR("allwinner setup clockevent failed! ret=%d", ret);
            return ret;
        }
    }

    ret  = allwinner_setup_counter64(timer_plat);

	return   ret;

}


MODULE_DESCRIPTION("allwinner's soc timer driver");
MODULE_LICENSE("GPL v2");

TIMER_OF_DECLARE(allwinner_h6_timer, "allwinner,H6-v200-timer",
		       allwinner_h6_timer_init);


