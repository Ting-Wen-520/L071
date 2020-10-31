/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-29     Admin       the first version
 */


#include <board.h>
#include<rtthread.h>
#include<rtdevice.h>
#include "drivers/rt_drv_pwm.h"
//#ifdef RT_USING_UPWM

//#define DRV_DEBUG
#define DBG_TAG "udrv_pwm"
#define DBG_LVL DBG_WARNING
#include <rtdbg.h>

#define MAX_PERIOD 65535
#define MIN_PERIOD 3
#define MIN_PULSE 2

extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

enum
{
    PWM2_INDEX,
    PWM3_INDEX,
    PWM21_INDEX,
    PWM22_INDEX,
};

struct stm32_pwm
{
    struct rt_device_pwm pwm_device;
    TIM_HandleTypeDef    tim_handle;
    rt_uint8_t channel;
    char *name;
};


#define PWM2_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = TIM2,         \
       .name                    = "pwm2",       \
       .channel                 = 0             \
    }
#define PWM3_CONFIG                             \
    {                                           \
       .tim_handle.Instance     = TIM3,         \
       .name                    = "pwm3",       \
       .channel                 = 0             \
    }

static struct stm32_pwm pwm3 = PWM3_CONFIG;


static rt_err_t drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg);
static struct rt_pwm_ops drv_ops =
{
    drv_pwm_control
};

/* get pwm control */
static rt_err_t drv_pwm_enable(TIM_HandleTypeDef *htim, struct rt_pwm_configuration *configuration, rt_bool_t enable)
{
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t channel = 0x04 * (configuration->channel - 1);

    if (!enable)
    {
        HAL_TIM_PWM_Stop(htim, channel);
    }
    else
    {
        HAL_TIM_PWM_Start(htim, channel);
    }
    return RT_EOK;
}

/* get pwm timer config */
static rt_err_t drv_pwm_get(TIM_HandleTypeDef *htim, struct rt_pwm_configuration *configuration)
{
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t channel = 0x04 * (configuration->channel - 1);
    rt_uint64_t tim_clock;
    if (htim->Instance == TIM3 || htim->Instance == TIM2)
    {
        tim_clock = HAL_RCC_GetPCLK1Freq();
    }
    else
    {
        tim_clock = HAL_RCC_GetPCLK2Freq();
    }

    if (__HAL_TIM_GET_CLOCKDIVISION(htim) == TIM_CLOCKDIVISION_DIV2)
    {
        tim_clock = tim_clock / 2;
    }
    else if (__HAL_TIM_GET_CLOCKDIVISION(htim) == TIM_CLOCKDIVISION_DIV4)
    {
        tim_clock = tim_clock / 4;
    }
    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    tim_clock /= 1000000UL;
    configuration->period = (__HAL_TIM_GET_AUTORELOAD(htim) + 1) * (htim->Instance->PSC + 1) * 1000UL / tim_clock;
    configuration->pulse = (__HAL_TIM_GET_COMPARE(htim, channel) + 1) * (htim->Instance->PSC + 1) * 1000UL / tim_clock;
    return RT_EOK;
}

TIM_OC_InitTypeDef sConfigOC = {0};
static rt_err_t drv_pwm_set(TIM_HandleTypeDef *htim, struct rt_pwm_configuration *configuration)
{
    LOG_I("pwm set");
    rt_uint32_t period, pulse;
    rt_uint64_t tim_clock, psc;
    /* Converts the channel number to the channel number of Hal library */
    rt_uint32_t channel = 0x04 * (configuration->channel - 1);

    if (htim->Instance == TIM2 || htim->Instance == TIM3)
    {
        tim_clock = HAL_RCC_GetPCLK1Freq();
    }
    else if (htim->Instance == TIM21 || htim->Instance == TIM22)
    {
        tim_clock = HAL_RCC_GetPCLK2Freq();
    }else
    {
        LOG_E("err tim instance");
    }
    

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    tim_clock /= 1000000UL;
    period = (unsigned long long)configuration->period * tim_clock / 1000ULL ;
    psc = period / MAX_PERIOD + 1;
    period = period / psc;
    __HAL_TIM_SET_PRESCALER(htim, psc - 1);

    if (period < MIN_PERIOD)
    {
        period = MIN_PERIOD;
    }
    __HAL_TIM_SET_AUTORELOAD(htim, period - 1);

    pulse = (unsigned long long)configuration->pulse * tim_clock / psc / 1000ULL;
    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse > period)
    {
        pulse = period;
    }

    HAL_TIM_PWM_Stop(htim,channel);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.Pulse = pulse;
    HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel);
    HAL_TIM_PWM_Start(htim,channel);

    return RT_EOK;
}

static rt_err_t drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg)
{
    LOG_I("pwm control");
    struct rt_pwm_configuration *configuration = (struct rt_pwm_configuration *)arg;
    TIM_HandleTypeDef *htim = (TIM_HandleTypeDef *)device->parent.user_data;

    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        return drv_pwm_enable(htim, configuration, RT_TRUE);
    case PWM_CMD_DISABLE:
        return drv_pwm_enable(htim, configuration, RT_FALSE);
    case PWM_CMD_SET:
        return drv_pwm_set(htim, configuration);
    case PWM_CMD_GET:
        return drv_pwm_get(htim, configuration);
    default:
        return RT_EINVAL;
    }
}

static rt_err_t stm32_hw_pwm_init(struct stm32_pwm *device)
{
    /* USER CODE BEGIN TIM3_Init 0 */

    TIM_HandleTypeDef* htim = &device->tim_handle;
    /* USER CODE END TIM3_Init 0 */
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim->Instance = TIM3;
    htim->Init.Prescaler = 8;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 1000;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(htim) != HAL_OK)
    {
      Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
    {
      Error_Handler();
    }
    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
    {
      Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
    sConfigOC.Pulse = 100;
    if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
      Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(htim);
    return RT_EOK;
}
//#endif /* RT_USING_UPWM */

static void pwm_get_channel(void)
{
    LOG_I("pwm get channel");
    pwm3.channel |= 1 << 2;
    pwm3.channel |= 1 << 3;
}

static int stm32_pwm_init(void)
{
    int result = RT_EOK;

    pwm_get_channel();
    LOG_I("pwm init");
        /* pwm init */
    if (stm32_hw_pwm_init(&pwm3) != RT_EOK)
    {
        LOG_E("%s init failed", pwm3.name);
        result = -RT_ERROR;
        goto __exit;
    }
    else
    {
        LOG_D("%s init success", pwm3.name);

        /* register pwm device */
        if (rt_device_pwm_register(&pwm3.pwm_device, pwm3.name, &drv_ops, &pwm3.tim_handle) == RT_EOK)
        {
            LOG_D("%s register success", pwm3.name);
        }
        else
        {
            LOG_E("%s register failed", pwm3.name);
            result = -RT_ERROR;
        }
    }
__exit:
    return result;
}
INIT_DEVICE_EXPORT(stm32_pwm_init);
