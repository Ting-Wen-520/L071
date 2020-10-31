/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-28     Admin       the first version
 */
#ifndef __STA_LED_H_
#define __STA_LED_H_

#include <rtthread.h>
#include <rtdevice.h>
#include <stm32l0xx_hal.h>

/* pwm config */
#define SL_PWM_NAME        "pwm3"
#define SL_PWM_PERIOD       1000000   /* ns */

/* led config */
#define SL_CM_LED_CHANAL        4
#define SL_SYSRUN_LED_CHANAL    3
#define SL_LUM_CHANGE_INTERVAL  10 /* ms */

/* thread config */
#define SL_TH_STACK_SIZE  512
#define SL_TH_PRIORITY    30
#define SL_TH_TIMESLICE   10

/*led mode*/
#define SL_UP        1U
#define SL_DOWN      0U
#define SL_BREATHE   (1<<7)

/*led index*/
typedef enum sl_led_ind
{
    SL_CM_LED,
    SL_SYSRUN_LED,

    SL_LED_E_END,  /**< at the end of enmm modify is prohibited */
}sl_led_ind_e;

/*led config*/
typedef struct sl_ledConfig
{
    uint8_t mode;             /* refer 'led mode' use '|' enable multiple mode */
    int16_t maxLum;           /* 0 ~ 1000*/
    int16_t minLum;           /* 0 ~ 1000*/
    int32_t durationTime;     /* ms */
    int16_t certenLum;        /* 0 ~ 1000*/
}sl_ledConfig_s;

extern rt_err_t SlLedConfig(uint8_t led,sl_ledConfig_s* config,int32_t timeOut);

#endif /* __STA_LED_H_ */
