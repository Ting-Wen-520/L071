/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-10     Admin       the first version
 */
#ifndef __REG_H
#define __REG_H

#include <rtthread.h>
#include "rtdevice.h"
#include "stm32l0xx.h"
#include <board.h>

#define REG_RW_NUM        64       /* 0x0000~0x7FFF */
#define REG_RO_NUM        64       /* 0x0000~0x7FFF */

#define REG_OPT_TIMEOUT   100      /* register operation timeout */

/*
 * register function return status
 */
typedef enum regRet
{
    REG_ROK,                       /* OK */
    REG_RERR,                      /* Error */
    REG_RPARAM_ERR,                /* Parameter error */
    REG_RADDR_ERR,                 /* Address error */
    REG_RBUFF_TOO_SMALL,           /* Buffer to small */
}retRet_e;

/*
 * register type
 */
typedef uint16_t reg;

/*
 * extern register function
 */
extern int RegInit(void);
extern retRet_e RegWrite(uint16_t sAddr, reg data);
extern retRet_e RegRead(uint16_t sAddr, reg* buff);
extern retRet_e TimRegAdd(void);
extern retRet_e RegsRead(uint16_t sAddr,uint32_t regNum,uint32_t buffNum,reg* buff);
extern retRet_e RegsWrite(uint16_t sAddr,uint16_t regNum ,reg* data);

/* RO reg map*/

/* common reg */
/*     |  reg brief         |   address  |   */
#define   REG_DEV_UID_W0H           0x0000
#define   REG_DEV_UID_W0L           0x0001
#define   REG_DEV_UID_W1H           0x0002
#define   REG_DEV_UID_W1L           0x0003
#define   REG_DEV_UID_W2H           0x0004
#define   REG_DEV_UID_W2L           0x0005
#define   REG_DEV_TYPE              0x0006           //0x0000:gun;0x0001:gate;0x0002:target;
#define   REG_TRI_STA               0x0007           //0x0000:notrigger;0x0001:triggered;
#define   REG_TRI_TIMH              0x0008           //trigger time high
#define   REG_TRI_TIML              0x0009           //trigger time low
/* dev special reg */
/*     |  reg brief         |   address  |   */


/* RW reg map*/
/* common reg */
/*     |  reg brief         |   address  |   */
#define   REG_SYS_TIMH              0x8000
#define   REG_SYS_TIML              0x8001
#define   REG_MATCH_STATUS          0x8002           //0x0000:idle; 0x0001:matching
#define   REG_TRI_TIM_CL            0x8003           //trigger time control reg 1:clear trigger time;other value:invalid
#define   REG_START_TIMH            0x0004           //start time high
#define   REG_START_TIML            0x0005           //start time low



#endif /* __REG_H */
