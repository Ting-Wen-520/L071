/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-28     Admin       the first version
 */
#include "sta_led.h"



#define DBG_TAG "sta_led"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


static rt_err_t SlLedInit(uint8_t led,const char *pwmName,uint8_t chanal,sl_ledConfig_s* config);

static char slThStack[SL_TH_STACK_SIZE];
static struct rt_thread slTh;



/* led config */
typedef struct sl_led
{
    uint8_t mode;
    int16_t maxLum;          /* 0 ~ 1000 */
    int16_t minLum;          /* 0 ~ 1000 && < maxLum */
    int16_t stepLum;
    int16_t certenLum;
    uint8_t chanal;
    struct rt_device_pwm *pwmDev;      /* PWM dev */
    struct rt_mutex mut;
}sl_led_s;
/* led configs */
sl_led_s  slLed[SL_LED_E_END] = {0};


static void SlThEntry(void *param);

static int StaLedInit(void)
{

    sl_ledConfig_s ledConfig = {.mode =SL_DOWN,
            .certenLum = 0,
            .durationTime = 300,
            .maxLum = 500,
            .minLum = 0,
            };
    SlLedInit(SL_CM_LED,SL_PWM_NAME,SL_CM_LED_CHANAL,&ledConfig);
    ledConfig.mode =SL_UP|SL_BREATHE;
    ledConfig.durationTime = 700;
    ledConfig.maxLum = 700,
    ledConfig.minLum = 0,
    ledConfig.certenLum = 0;
    SlLedInit(SL_SYSRUN_LED,SL_PWM_NAME,SL_SYSRUN_LED_CHANAL,&ledConfig);

    /* initialize 'slTh'，entry 'SlThEntry' */
    rt_thread_init(&slTh,
                   "slTh",
                   SlThEntry,
                   RT_NULL,
                   slThStack,
                   sizeof(slThStack),
                   SL_TH_PRIORITY, SL_TH_TIMESLICE);
    rt_thread_startup(&slTh);

    return RT_EOK;
}
INIT_APP_EXPORT(StaLedInit);


/*
 * @brief led initialize (thread unsafe)
 * @param led: led index
 * @param chanal: pwm chanal
 * @param config: led config
 * @param name: pwm name
 */
static rt_err_t SlLedInit(uint8_t led,const char *pwmName,uint8_t chanal,sl_ledConfig_s* config)
{
    slLed[led].chanal = chanal;
    char ledName[16] = "";
    rt_sprintf(ledName,"led%03d",led);
    rt_mutex_init(&(slLed[led].mut),ledName,RT_IPC_FLAG_PRIO);

    /* find dev */
    slLed[led].pwmDev = (struct rt_device_pwm *)rt_device_find(pwmName);
    if (slLed[led].pwmDev == RT_NULL)
    {
        LOG_E("pwm sample run failed! can't find %s device!\n", SL_PWM_NAME);
        return RT_ERROR;
    }
    slLed[led].mode = config->mode;
    slLed[led].certenLum = config->certenLum;
    slLed[led].maxLum = config->maxLum;
    slLed[led].minLum = config->minLum;
    slLed[led].stepLum = (config->maxLum - config->minLum)/(config->durationTime/SL_LUM_CHANGE_INTERVAL);
    rt_pwm_set(slLed[led].pwmDev, slLed[led].chanal, SL_PWM_PERIOD, ((uint32_t)slLed[led].certenLum)*1000);
    rt_pwm_enable(slLed[led].pwmDev, slLed[led].chanal);
    return RT_EOK;
}



/*
 * @brief led initialize (thread safe)
 * @param led: led index
 * @param chanal: pwm chanal
 * @param name: pwm name
 */
rt_err_t SlLedConfig(uint8_t led,sl_ledConfig_s* config,int32_t timeOut)
{
    rt_err_t ret = RT_EOK;

    if(rt_mutex_take(&(slLed[led].mut), timeOut) != RT_EOK ) return ret;
    slLed[led].mode = config->mode;
    slLed[led].certenLum = config->certenLum;
    slLed[led].maxLum = config->maxLum;
    slLed[led].minLum = config->minLum;
    slLed[led].stepLum = (config->maxLum - config->minLum)/(config->durationTime/SL_LUM_CHANGE_INTERVAL);
    rt_mutex_release(&slLed[led].mut);
    return ret;
}


/* 'slTh' entry */
static void SlThEntry(void *param)
{
    while(1)
    {
        rt_thread_mdelay(SL_LUM_CHANGE_INTERVAL);
        for(uint8_t i = 0; i < SL_LED_E_END ;i++)
        {
            if(rt_mutex_take(&slLed[i].mut,0) != RT_EOK)
                continue;
            if(slLed[i].mode & SL_UP)
            {
                if(slLed[i].certenLum + slLed[i].stepLum > slLed[i].maxLum)
                    if(slLed[i].mode & SL_BREATHE)
                        slLed[i].mode &= ~SL_UP;
                    else
                        slLed[i].certenLum = slLed[i].maxLum;
                else
                    slLed[i].certenLum += slLed[i].stepLum;
            }
            else
            {
                if(slLed[i].certenLum - slLed[i].stepLum < slLed[i].minLum)
                    if(slLed[i].mode & SL_BREATHE)
                        slLed[i].mode |= SL_UP;
                    else
                        slLed[i].certenLum = slLed[i].minLum;
                else
                    slLed[i].certenLum -= slLed[i].stepLum;
            }

            rt_pwm_set(slLed[i].pwmDev, slLed[i].chanal, SL_PWM_PERIOD, ((uint32_t)slLed[i].certenLum)*1000);
            rt_mutex_release(&slLed[i].mut);
        }
    }
}

//static int SlSet(int argc, char *argv[])
//{
//    return 0;
//}
//MSH_CMD_EXPORT(SlSet, led sample);
