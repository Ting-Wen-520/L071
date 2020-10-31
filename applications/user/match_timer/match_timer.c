/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-27     Admin       the first version
 */

#include "match_timer.h"

#include <rtthread.h>
#include <rtdevice.h>

#define DBG_TAG "MatTime"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define MT_TIM_NAME   "timer2"     /* tim name */

/* tim timeout callback */
static rt_err_t MtTimeoutCb(rt_device_t dev, rt_size_t size)
{
    TimRegAdd();
    return 0;
}

static int MtInit(int argc, char *argv[])
{
    rt_err_t ret = RT_EOK;
    rt_hwtimerval_t timeoutVal;      /* timeout value */
    rt_device_t timerDev = RT_NULL;   /* timer device */
    rt_hwtimer_mode_t timerMode;         /* timer mode */

    /* search timer */
    timerDev = rt_device_find(MT_TIM_NAME);
    if (timerDev == RT_NULL)
    {
        rt_kprintf("hwtimer sample run failed! can't find %s device!\n", MT_TIM_NAME);
        return RT_ERROR;
    }

    /* 以读写方式打开设备 */
    ret = rt_device_open(timerDev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK)
    {
        rt_kprintf("open %s device failed!\n", MT_TIM_NAME);
        return ret;
    }

    /* 设置超时回调函数 */
    rt_device_set_rx_indicate(timerDev, MtTimeoutCb);

    /* 设置模式为周期性定时器 */
    timerMode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(timerDev, HWTIMER_CTRL_MODE_SET, &timerMode);
    if (ret != RT_EOK)
    {
        rt_kprintf("set mode failed! ret is :%d\n", ret);
        return ret;
    }

    /* 设置定时器超时值为5s并启动定时器 */
    timeoutVal.sec = 5;      /* 秒 */
    timeoutVal.usec = 0;     /* 微秒 */

    if (rt_device_write(timerDev, 0, &timeoutVal, sizeof(timeoutVal)) != sizeof(timeoutVal))
    {
        rt_kprintf("set timeout value failed\n");
        return RT_ERROR;
    }

    /* 延时3500ms */
    rt_thread_mdelay(3500);

    /* 读取定时器当前值 */
    rt_device_read(timerDev, 0, &timeoutVal, sizeof(timeoutVal));
    rt_kprintf("Read: Sec = %d, Usec = %d\n", timeoutVal.sec, timeoutVal.usec);

    return ret;
}
/* 导出到 msh 命令列表中 */
MSH_CMD_EXPORT(MtInit, hwtimer sample);
