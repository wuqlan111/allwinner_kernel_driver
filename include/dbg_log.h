#ifndef  ALLWINNER_DBG_LOG_H
#define  ALLWINNER_DBG_LOG_H


#include <linux/init.h>
#include <linux/module.h>
#include <linux/err.h>

#define DEVICE_PHY_ADDR_RESOURCE     "phy_addr"

#define  _PRINTF_DBG(fmt, args...)       printk(KERN_DEBUG "[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_INFO(fmt, args...)      pr_info("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_NOTICE(fmt, args...)    pr_notice("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_WARN(fmt, args...)      pr_warn("[%s: %u] - " fmt, __func__,  __LINE__, ##args)
#define  _PRINTF_ERROR(fmt, args...)     pr_err("[%s: %u] - " fmt, __func__,  __LINE__, ##args)


#ifdef   ALLWINNER_DRIVER_DBG
#undef  readl_relaxed
#undef  writel_relaxed
#define  readl_relaxed(addr)           _PRINTF_DBG("read reg at 0x%p\n", addr)
#define  writel_relaxed(val, addr)     _PRINTF_DBG("write [ %#x ] to reg at 0x%p\n", val, addr)
#endif

#define  TYPE_CASE(x)         case x: return #x;


#endif


