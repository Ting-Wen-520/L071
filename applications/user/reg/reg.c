/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-10     Admin       the first version
 */

#include "reg.h"

#define DBG_TAG "reg"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

static RT_USED __IO reg regRO[REG_RO_NUM]={0}; // 0x0000~0x7FFF
static RT_USED __IO reg regRW[REG_RW_NUM]={0}; // 0x8000~0xFFFF

int RegInit(void)
{
    int ret = RT_EOK;
    uint32_t temp = 0;

    /* init uid reg */
    uint16_t w0h = 0;
    uint16_t w0l = 0;
    uint16_t w1h = 0;
    uint16_t w1l = 0;
    uint16_t w2h = 0;
    uint16_t w2l = 0;
    temp = HAL_GetUIDw0();
    w0h = temp>>16;
    w0l = temp;
    temp = HAL_GetUIDw1();
    w1h = temp>>16;
    w1l = temp;
    temp = HAL_GetUIDw2();
    w2h = temp>>16;
    w2l = temp;
    RegWrite(REG_DEV_UID_W0H, w0h);
    RegWrite(REG_DEV_UID_W0L, w0l);
    RegWrite(REG_DEV_UID_W1H, w1h);
    RegWrite(REG_DEV_UID_W1L, w1l);
    RegWrite(REG_DEV_UID_W2H, w2h);
    RegWrite(REG_DEV_UID_W2L, w2l);

    uint16_t uid[6] = {0};

    RegsRead(REG_DEV_UID_W0H, sizeof(uid)/2, sizeof(uid)/2, uid);

    rt_kprintf("OwnUid:");
    for(int i = 0; i < sizeof(uid)/2; i++)
    {
        rt_kprintf("%04X ",uid[i]);
    }
    rt_kprintf("\r\n");

    return ret;
}
INIT_APP_EXPORT(RegInit);


/**
* @brief Register read
* @param sAddr: Start address
* @param buff: Data buffer
* @retval Refer 'retRet_e'
*/
retRet_e RegRead(uint16_t sAddr, reg* buff)
{
    RT_ASSERT(buff != RT_NULL);
    if(sAddr >= REG_RO_NUM && sAddr < 0x8000) return REG_RADDR_ERR;
    if(sAddr >= 0x8000 + REG_RW_NUM) return REG_RADDR_ERR;


    rt_base_t level;
    /* enter critical  */
    uint8_t criticalProtectType = 0;
    if((sAddr == REG_SYS_TIMH) || (sAddr == REG_SYS_TIML))
        criticalProtectType = 1;
    if(criticalProtectType)
        level = rt_hw_interrupt_disable();
    else
        rt_enter_critical();

    if(sAddr < 0x8000)
    {
        *buff = regRO[sAddr];
    }
    else
    {
        sAddr &= 0x7FFF;
        *buff =regRW[sAddr];
    }

    /* exit critical  */
    if(criticalProtectType)
        rt_hw_interrupt_enable(level);
    else
        rt_exit_critical();

    return REG_ROK;
}



/**
* @brief Register write
* @param sAddr: Start address
* @param buff: Write data
* @retval Refer 'retRet_e'
*/
retRet_e RegWrite(uint16_t sAddr, reg data)
{
    if(sAddr >= REG_RO_NUM && sAddr < 0x8000) return REG_RADDR_ERR;
    if(sAddr >= 0x8000 + REG_RW_NUM) return REG_RADDR_ERR;

    rt_base_t level;
    /* enter critical  */
    uint8_t criticalProtectType = 0;
    if((sAddr == REG_SYS_TIMH) || (sAddr == REG_SYS_TIML))
        criticalProtectType = 1;
    if(criticalProtectType)
        level = rt_hw_interrupt_disable();
    else
        rt_enter_critical();

    if(sAddr < 0x8000)
    {
        regRO[sAddr] = data;
    }
    else
    {
        sAddr &= 0x7FFF;
        regRW[sAddr] = data;
    }

    /* exit critical  */
    if(criticalProtectType)
        rt_hw_interrupt_enable(level);
    else
        rt_exit_critical();

    return REG_ROK;
}


/**
* @brief Time register add, only used at timer irqhandler
* @param None
* @retval Refer 'retRet_e'
*/
retRet_e TimRegAdd(void)
{
    uint16_t temp = regRW[REG_SYS_TIML]++;
    if(regRW[REG_SYS_TIML] < temp)
    {
        ++regRW[REG_SYS_TIMH];
    }
    return REG_ROK;
}



/**
* @brief Registers read
* @param[in] sAddr: Start address
* @param[in] regNum: Read registers number
* @param[in] buffNum: Data buffer number
* @param[out] buff: Data buffer
* @retval Refer 'retRet_e'
*/
retRet_e RegsRead(uint16_t sAddr,uint32_t regNum,uint32_t buffNum,reg* buff)
{
    RT_ASSERT(buff != RT_NULL);
    RT_ASSERT(buffNum != 0);
    if(sAddr >= REG_RO_NUM && sAddr < 0x8000) return REG_RADDR_ERR;
    if(sAddr+regNum-1 >= REG_RO_NUM && sAddr+regNum-1 < 0x8000) return REG_RADDR_ERR;
    if(sAddr >= 0x8000 + REG_RW_NUM) return REG_RADDR_ERR;
    if(sAddr+regNum-1 >= 0x8000 + REG_RW_NUM) return REG_RADDR_ERR;
    if(buffNum < regNum) return REG_RBUFF_TOO_SMALL;

    uint32_t buffI = 0;
    rt_base_t level;


    /* enter critical  */
    uint8_t criticalProtectType = 0;
    if( (sAddr <= REG_SYS_TIMH && REG_SYS_TIMH <= sAddr+REG_RW_NUM-1) ||
        (sAddr <= REG_SYS_TIML && REG_SYS_TIML <= sAddr+REG_RW_NUM-1) )
        criticalProtectType = 1;
    if(criticalProtectType)
        level = rt_hw_interrupt_disable();
    else
        rt_enter_critical();

    if(sAddr < 0x8000)
    {
        for(uint32_t regI = 0; regI < regNum; regI++)
            buff[buffI++] = regRO[sAddr + regI];
    }
    else
    {
        sAddr &= 0x7FFF;
        for(uint32_t regI = 0; regI < regNum; regI++)
            buff[buffI++] = regRW[sAddr + regI];
    }
    /* exit critical  */
    if(criticalProtectType)
        rt_hw_interrupt_enable(level);
    else
        rt_exit_critical();

    return REG_ROK;
}



/**
* @brief Registers write
* @param[in] sAddr: Start address
* @param[in] regNum: Write registers number
* @param[in] data: Writed data
* @retval Refer 'retRet_e'
*/
retRet_e RegsWrite(uint16_t sAddr,uint16_t regNum ,reg* data)
{
    RT_ASSERT(data != RT_NULL);
    if(sAddr >= REG_RO_NUM && sAddr < 0x8000) return REG_RADDR_ERR;
    if(sAddr+regNum-1 >= REG_RO_NUM && sAddr+regNum-1 < 0x8000) return REG_RADDR_ERR;
    if(sAddr >= 0x8000 + REG_RW_NUM) return REG_RADDR_ERR;
    if(sAddr+regNum-1 >= 0x8000 + REG_RW_NUM) return REG_RADDR_ERR;

    uint32_t dataI = 0;
    rt_base_t level;

    /* enter critical  */
    uint8_t criticalProtectType = 0;
    if( (sAddr <= REG_SYS_TIMH && REG_SYS_TIMH <= sAddr+REG_RW_NUM-1) ||
        (sAddr <= REG_SYS_TIML && REG_SYS_TIML <= sAddr+REG_RW_NUM-1) )
        criticalProtectType = 1;
    if(criticalProtectType)
        level = rt_hw_interrupt_disable();
    else
        rt_enter_critical();

    if(sAddr < 0x8000)
    {
        for(uint32_t regI = 0; regI < regNum; regI++)
            regRO[sAddr + regI] = data[dataI++];
    }
    else
    {
        sAddr &= 0x7FFF;
        for(uint32_t regI = 0; regI < regNum; regI++)
            regRW[sAddr + regI] = data[dataI++];
    }

    /* exit critical  */
    if(criticalProtectType)
        rt_hw_interrupt_enable(level);
    else
        rt_exit_critical();

    return REG_ROK;
}





/* ------------------- output cmds ------------------- */
#include <stdlib.h>
#define BUFF_SIZE  8

/**
* @brief Capital to lower
* @param[in] c: Capital
* @retval lower
*/
static int tolower(int c)
{
    if (c >= 'A' && c <= 'Z')
        return c + 'a' - 'A';
    else
        return c;
}



/**
* @brief Hex to int
* @param[in] s: Hex string
* @retval int
*/
static int htoi(char s[])
{
    RT_ASSERT(s!=RT_NULL);
    int i;
    int n = 0;
    if (s[0] == '0' && (s[1]=='x' || s[1]=='X'))
    {
        i = 2;
    }
    else
    {
        i = 0;
    }
    for (; (s[i] >= '0' && s[i] <= '9') || (s[i] >= 'a' && s[i] <= 'z') || (s[i] >='A' && s[i] <= 'Z');++i)
    {
        if (tolower(s[i]) > '9')
        {
            n = 16 * n + (10 + tolower(s[i]) - 'a');
        }
        else
        {
            n = 16 * n + (tolower(s[i]) - '0');
        }
    }
    return n;
}




/**
* @brief Read registers cmd
* @param[in] argc: Arguments number
* @param[in] argv: Arguments
* @retval int
*/
static int ReadRegsCmd(int argc, char *argv[])
{
    if(argc < 3)
    {
        rt_kprintf("Param error\r\n");
        rt_kprintf("Example:ReadRegsCmd <startAddr> <regNum>\r\n");
        rt_kprintf("  Addr(hex):0000~%04X,%04X~%04X\r\n",REG_RO_NUM-1,0x8000,0x8000+REG_RW_NUM-1);
        rt_kprintf("  regNum(dec):1~%d\r\n",BUFF_SIZE);
        return 0;
    }

    uint32_t sAddr = htoi(argv[1]);
    uint32_t regNum = atoi(argv[2]);

    if(regNum == 0 || regNum > BUFF_SIZE)
    {
        rt_kprintf("regNum too big \r\n");
        rt_kprintf("  regNum(dec):1~%d\r\n",BUFF_SIZE);
        return 0;
    }

    uint16_t buff[BUFF_SIZE] = {0};

    if(RegsRead(sAddr, regNum, sizeof(buff)/2, buff) == REG_RADDR_ERR)
    {
        rt_kprintf("Addr error \r\n");
        rt_kprintf("  Addr(hex):0000~%04X,%04X~%04X\r\n",REG_RO_NUM-1,0x8000,0x8000+REG_RW_NUM-1);
        return 0;
    }

    rt_kprintf("Addr(hex):");
    for(int i = 0; i < regNum; i++)
    {
        rt_kprintf("%4X ",sAddr+i);
    }
    rt_kprintf("\r\n");
    rt_kprintf("Data(hex):");
    for(int i = 0; i < regNum; i++)
    {
        rt_kprintf("%04X ",buff[i]);
    }
    rt_kprintf("\r\n");
    return 0;
}
MSH_CMD_EXPORT(ReadRegsCmd,read regs Example:ReadRegsCmd <startAddr> <regNum>);



/**
* @brief Write registers cmd
* @param[in] argc: Arguments number
* @param[in] argv: Arguments
* @retval int
*/
static int WriteRegsCmd(int argc, char *argv[])
{
    if(argc < 4)
    {
        rt_kprintf("Param error\r\n");
        rt_kprintf("Example:WriteRegsCmd <startAddr> <regNum> <data1> <data2> ... <dataN>\r\n");
        rt_kprintf("  Addr(hex):0000~%04X,%04X~%04X\r\n",REG_RO_NUM-1,0x8000,0x8000+REG_RW_NUM-1);
        rt_kprintf("  regNum(dec):1~%d\r\n",BUFF_SIZE);
        rt_kprintf("  data(HEX)\r\n");
        return 0;
    }

    uint32_t sAddr = htoi(argv[1]);
    uint32_t regNum = atoi(argv[2]);

    if(regNum == 0 || regNum > BUFF_SIZE)
    {
        rt_kprintf("regNum too big \r\n");
        rt_kprintf("  regNum(dec):1~%d\r\n",BUFF_SIZE);
        return 0;
    }

    uint16_t buff[BUFF_SIZE] = {0};

    for(uint32_t i=0; i<regNum; i++)
    {
        buff[i] = htoi(argv[i+3]);
    }

    if(RegsWrite(sAddr, regNum, buff) == REG_RADDR_ERR)
    {
        rt_kprintf("Addr error \r\n");
        rt_kprintf("  Addr(hex):0000~%04X,%04X~%04X\r\n",REG_RO_NUM-1,0x8000,0x8000+REG_RW_NUM-1);
        return 0;
    }

    rt_kprintf("Addr(hex):");
    for(int i = 0; i < regNum; i++)
    {
        rt_kprintf("%4X ",sAddr+i);
    }
    rt_kprintf("\r\n");
    rt_kprintf("Data(hex):");
    for(int i = 0; i < regNum; i++)
    {
        rt_kprintf("%04X ",buff[i]);
    }
    rt_kprintf("\r\n");
    return 0;
}
MSH_CMD_EXPORT(WriteRegsCmd,Example:WriteRegsCmd <startAddr> <regNum> <data1> <data2> ... <dataN>);
#undef BUFF_SIZE
