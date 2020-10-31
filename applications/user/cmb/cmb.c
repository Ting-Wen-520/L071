/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-07     Admin       the first version
 */

#include <User/cmb/cmb.h>

#define DBG_TAG "cmb"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>


/* Typedef start ----------------------------------------------------------- */

typedef uint8_t cmbAddr;
typedef uint8_t cmbEnd;
typedef uint32_t cmbFuncCode;
typedef uint32_t cmbCrc;



/* bus status */
typedef enum cmbSta
{
    FREE = 0,
    BUSY = 1,
}cmbSta_e;

/* ret status */
typedef enum cmbRet
{
    CMB_ROK = 0,                    /* OK */
    CMB_RERR,                       /* Error */
    CMB_RPARAM_ERR,                 /* Parameter error */
    CMB_RBUFF_TO_SMALL,             /* Error: Buffer to small */
    CMB_RDATA_TO_SHORT,             /* Error: Rx data to short */
    CMB_RCRC_MISMATCH,              /* Inf: CRC mismatch */
    CMB_RADDR_MISMATCH,             /* Inf: Address mismatch */
    CMB_RSHORT_FRAME,               /* Inf: Frame to short */
    CMB_RBROADCAST,                 /* Inf: broadcast no response */
}cmbRet_e;

/* exception code */
typedef enum cmbExcC
{
    CMB_ILLEGAL_FUNCTION = 0x01,
    CMB_ILLEGAL_DATA_ADDRESS = 0x02,
    CMB_ILLEGAL_DATA_VALUE = 0x03,
    CMB_SERVER_DEVICE_FAILURE = 0x04,
    CMB_ACKNOWLEDGE = 0x05,
    CMB_SERVER_DEVICE_BUSY = 0x06,
    CMB_MEMORY_PARITY_ERROR = 0x08,
    CMB_GATWAN_PATH_UNAVALABLE = 0x0A,
    CMB_GATWAN_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0B,
}cmbExcC_e;

/* data buff */
typedef struct cmbBuff
{
    uint8_t* data;
    uint32_t dataLen;
    const uint32_t buffLen;
}cmbBuff_s;

/* uart param */
typedef struct cmbUart
{
    rt_device_t cmbUart;             /**< UART handle */
    struct rt_semaphore rxSem;       /**< UART rx data sem*/
    struct rt_semaphore rxCpltSem;   /**< UART rx data complete sem*/
    cmbSta_e busSta;                 /**< UART status*/
    cmbBuff_s txBuff;
    cmbBuff_s rxBuff;
}cmbUart_s;

typedef cmbRet_e (*cmbFunc)(const cmbBuff_s* in, cmbBuff_s* out);

/* custom modbus handle */
typedef struct cmb
{
    cmbUart_s* uart;
    cmbAddr ownAddr[CMB_ADDR_LEN];
    cmbEnd endBytes[CMB_END_LEN];
    cmbFunc functionP[CMB_FUNC_NUM];
}cmb_s;

/* Typedef end ------------------------------------------------------------- */
/* Var define start -------------------------------------------------------- */

static const uint8_t auchCRCHi[] = {
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00,
0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81,
0x40
} ;

static const uint8_t auchCRCLo[] = {
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB,
0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE,
0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2,
0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E,
0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B,
0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27,
0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD,
0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8,
0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4,
0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94,
0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59,
0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D,
0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
0x41, 0x81, 0x80,
0x40
};

static uint8_t cmbUartRBuff[CMB_UART_BUFF_SIZE];
static uint8_t cmbUartTBuff[CMB_UART_BUFF_SIZE];
RT_USED static cmbUart_s uart =
{
    .cmbUart = RT_NULL,
    .busSta = FREE,
    .rxBuff = {.data = cmbUartRBuff,
               .dataLen = 0,
               .buffLen = CMB_UART_BUFF_SIZE},
    .txBuff = {.data = cmbUartTBuff,
               .dataLen = 0,
               .buffLen = CMB_UART_BUFF_SIZE},
};

static cmb_s cmb =
{
    .ownAddr = {0},
    .uart = &uart,
    .endBytes = CMB_END_BYTES,
};

/* Var define end ---------------------------------------------------------- */
/* Func declare start ------------------------------------------------------ */
static void CmbThEntry(void *parameter);
static rt_err_t CmbUartCb(rt_device_t dev, rt_size_t size);

static cmbRet_e CmbAduProcess(const cmbBuff_s* in, cmbBuff_s* out);
static cmbRet_e CmbPduProcess(const cmbBuff_s* in, cmbBuff_s* out);
static cmbRet_e CmbAduExtAddr(const cmbBuff_s* in, cmbBuff_s* addr);
static cmbRet_e CmbAduExtCrc(const cmbBuff_s* in, cmbBuff_s* crc);
static cmbRet_e CmbPduExtFuncC(const cmbBuff_s* in, cmbFuncCode* funcC);

static cmbCrc CmbFastCRC16ModBus (const void *puchMsgP,uint32_t  usDataLen);
static cmbRet_e CmbFillData(const cmbBuff_s* data, cmbBuff_s* buff);
static uint32_t CmbDataCompare(const void* d1, const void* d2, uint32_t size);
static cmbRet_e CmbFillInt (uint32_t intger,uint32_t size,uint8_t endian, cmbBuff_s* buff);

/* function */
static cmbRet_e CmbReadHoldRegs(const cmbBuff_s* in, cmbBuff_s* out);
static cmbRet_e CmbReadInputRegs(const cmbBuff_s* in, cmbBuff_s* out);
static cmbRet_e CmbWriteMulRegs(const cmbBuff_s* in, cmbBuff_s* out);

#if DBG_LVL >= DBG_LOG
#define CMB_PRINT_BUFF(msg,buff)   CmbPrintBuff(msg,buff);
#else
#define CMB_PRINT_BUFF(msg,buff)
#endif

#if DBG_LVL >= DBG_LOG
static void CmbPrintBuff (const char* msg ,const cmbBuff_s* data);
#endif
/* Func declare end -------------------------------------------------------- */
/* Function start ---------------------------------------------------------- */
/**
* @brief Init function, application resources
* @param none
* @retval RT_ERROR: Error occur
* @retval RT_EOK: Process complete
*/
int CmbInit(void)
{
    // get ownAddr
    uint32_t temp = 0;
    uint32_t ownAddrI = 0;
    temp = HAL_GetUIDw0();
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 3);
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 2);
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 1);
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 0);
    temp = HAL_GetUIDw1();
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 3);
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 2);
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 1);
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 0);
    temp = HAL_GetUIDw2();
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 3);
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 2);
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 1);
    cmb.ownAddr[ownAddrI++] = temp >> (8 * 0);

    // add function
    cmb.functionP[CMB_READ_HOLD_REGS_FUNCC] = CmbReadHoldRegs;
    cmb.functionP[CMB_READ_INPUT_REGS_FUNCC] = CmbReadInputRegs;
    cmb.functionP[CMB_WRITE_MUL_REGS_FUNCC] = CmbWriteMulRegs;

    cmb.uart->cmbUart = rt_device_find(CMB_UART_NAME);
    if (cmb.uart->cmbUart == RT_NULL)
    {
        LOG_E("find '%s' failed!\n", CMB_UART_NAME);
        return RT_ERROR;
    }

    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /** Uart config */
    /* change baudrate*/
    config.baud_rate = CMB_UART_BAUD_RATE;
    rt_device_control(cmb.uart->cmbUart, RT_DEVICE_CTRL_CONFIG, &config);

    /* init sem */
    rt_sem_init(&cmb.uart->rxSem, "cmbUartRxSem", 0, RT_IPC_FLAG_FIFO);

    /* open uart with interrupt mode */
    rt_device_open(cmb.uart->cmbUart, RT_DEVICE_FLAG_INT_RX);

    /* set rx callback */
    rt_device_set_rx_indicate(cmb.uart->cmbUart, CmbUartCb);


//    /* create serial thread  */
//    rt_thread_t thread = rt_thread_create("cmbTh", //name
//        CmbThEntry, //entrance
//        RT_NULL,
//        CMB_THREAD_STACK_SIZE,
//        CMB_THREAD_PRIORITY,
//        CMB_THREAD_TICK);

    static char cmbThStack[CMB_THREAD_STACK_SIZE];
    static struct rt_thread cmbTh;
    rt_thread_init(&cmbTh,
                   "cmbTh",
                   CmbThEntry,
                   RT_NULL,
                   &cmbThStack[0],
                   sizeof(cmbThStack),
                   CMB_THREAD_PRIORITY, CMB_THREAD_TICK);

    rt_thread_startup(&cmbTh);
//    if (thread != RT_NULL)
//    {
//        rt_thread_startup(&thread);
//    }
//    else
//    {
//        LOG_E("Thread '%s' create failed!\n", thread->name);
//        return RT_ERROR;
//    }
    return RT_EOK;
}
INIT_APP_EXPORT(CmbInit);



/**
* @brief Communicate thread entrance function
* @param parameter:parameter
* @retval none
*/
static void CmbThEntry(void *parameter)
{
    uint32_t endInd = 0;
    while (1)
    {
        /* wait RX Semaphore */
        rt_sem_take(&cmb.uart->rxSem, RT_WAITING_FOREVER);
        /* Occupy uart */
        cmb.uart->busSta = BUSY;

        if(cmb.uart->rxBuff.dataLen+1 < cmb.uart->rxBuff.buffLen)
            rt_device_read(cmb.uart->cmbUart, -1, &cmb.uart->rxBuff.data[cmb.uart->rxBuff.dataLen++], 1);
        while (1)
        {
            if(rt_sem_take(&cmb.uart->rxSem, CMB_UART_RX_TIMEOUT) == RT_EOK)
            {
                /* rx one byte */
                if(cmb.uart->rxBuff.dataLen < cmb.uart->rxBuff.buffLen)
                {
                    rt_device_read(cmb.uart->cmbUart, -1, &cmb.uart->rxBuff.data[cmb.uart->rxBuff.dataLen++], 1);
                    for( endInd = CMB_END_LEN; endInd > 0; endInd--)
                    {
                        if(cmb.uart->rxBuff.data[cmb.uart->rxBuff.dataLen-endInd] == cmb.endBytes[CMB_END_LEN - endInd]);
                        else break;
                    }
                    if(endInd == 0)/* rx completed */
                    {
                        cmb.uart->busSta = FREE;
                        if(CmbAduProcess(&cmb.uart->rxBuff, &cmb.uart->txBuff) == CMB_ROK)
                            rt_device_write(cmb.uart->cmbUart, 0, &cmb.uart->txBuff.data[0], cmb.uart->txBuff.dataLen);
                        cmb.uart->txBuff.dataLen = 0;// clear buff
                        cmb.uart->rxBuff.dataLen = 0;// clear buff
                        break;
                    }
                }
            }else
            {
                LOG_I("Rx time out");
                cmb.uart->rxBuff.dataLen = 0;// clear buff
                break;
            }
        }
        /* Free uart */
        cmb.uart->busSta = FREE;
    }
}



/**
* @brief Uart RX callback function
* @param dev:RX device
* @param size:RX size
* @retval none
*/
static rt_err_t CmbUartCb(rt_device_t dev, rt_size_t size)
{
    /* this callback founction is called by uart RX interrupt */
    /* send Semaphore */
    rt_sem_release(&cmb.uart->rxSem);
    return RT_EOK;
}




/**
* @brief Adu Process
* @param in:  input data
* @param out: output data
* @retval 0
*/
static cmbRet_e CmbAduProcess(const cmbBuff_s* in, cmbBuff_s* out)
{
    RT_ASSERT(in != RT_NULL)
    RT_ASSERT(out != RT_NULL)

#if DBG_LVL >= DBG_LOG

    int var = 0;
    _DBG_LOG_HDR("D", "0");
    rt_kprintf("RxADU:");
    rt_kprintf("\033[32m");
    for (; var < in->dataLen && var < CMB_ADDR_END; ++var) {
        rt_kprintf("%02X ",in->data[var]);
    }
    rt_kprintf("\033[33m");
    for (; var < in->dataLen && var < CMB_FUNCC_END; ++var) {
        rt_kprintf("%02X ",in->data[var]);
    }
    rt_kprintf("\033[0m");
    for (; var < in->dataLen - CMB_CRC_LEN - CMB_END_LEN; ++var) {
        rt_kprintf("%02X ",in->data[var]);
    }
    rt_kprintf("\033[34m");
    for (; var < in->dataLen - CMB_END_LEN; ++var) {
        rt_kprintf("%02X ",in->data[var]);
    }
    rt_kprintf("\033[35m");
    for (; var < in->dataLen; ++var) {
        rt_kprintf("%02X ",in->data[var]);
    }
    _DBG_LOG_X_END;
#endif
    cmbRet_e ret = CMB_ROK;
    if (in->dataLen < CMB_ADDR_LEN + CMB_FUNCC_LEN + CMB_CRC_LEN + CMB_END_LEN) {
        LOG_I("Short frame ignore.");
        return CMB_RSHORT_FRAME;
    }

    /* CRC */
    uint8_t fCrcBuff[CMB_CRC_LEN];
    cmbBuff_s fCrc = {.data = fCrcBuff, .dataLen = 0, .buffLen = sizeof(fCrcBuff)};
    uint8_t cCrcBuff[CMB_CRC_LEN];
    cmbBuff_s cCrc = {.data = cCrcBuff, .dataLen = 0, .buffLen = sizeof(cCrcBuff)};
    cmbCrc cCrcI;
    ret = CmbAduExtCrc(in,&fCrc);
    if(ret != CMB_ROK) return ret;
    cCrcI = CmbFastCRC16ModBus(in->data, in->dataLen - CMB_END_LEN - CMB_CRC_LEN);
    ret = CmbFillInt(cCrcI, CMB_CRC_LEN, 1, &cCrc);
    if(ret != CMB_ROK) return ret;
    CMB_PRINT_BUFF("  FrameCrc",&fCrc);
    CMB_PRINT_BUFF("  CalcCrc",&cCrc);
    if(!CmbDataCompare(fCrc.data, cCrc.data, CMB_CRC_LEN))
    {
        LOG_I("CRC mismatch ignore.");
        return CMB_RCRC_MISMATCH;
    }
    sl_ledConfig_s ledConfig = {.mode =SL_DOWN,
            .certenLum = 300,
            .durationTime = 300,
            .maxLum = 500,
            .minLum = 0,
            };
    SlLedConfig(SL_CM_LED,&ledConfig,0);


    /* addr */
    uint8_t addrFlag = 0; // 1:BroadCastAddr 0:normalAddr other:mismatchAddr
    uint8_t addrBuff[CMB_ADDR_LEN];
    cmbBuff_s addr = {.data = addrBuff, .buffLen = sizeof(addrBuff)};

    uint8_t boardcastBuff[CMB_ADDR_LEN] = {0};
    cmbBuff_s boardcast = {.data = boardcastBuff, .buffLen = sizeof(boardcastBuff)};

    ret = CmbAduExtAddr(in,&addr);
    if(ret != CMB_ROK)return ret;

    if(!CmbDataCompare(addr.data, cmb.ownAddr, CMB_ADDR_LEN))
    {
        if(CmbDataCompare(boardcast.data, cmb.ownAddr, CMB_ADDR_LEN))/* broad cast */
        {
            addrFlag = 1;
        }else;
        {
            addrFlag = 3;
            LOG_I("Addr mismatch ignore.");
            return CMB_RADDR_MISMATCH;
        }
    }

    cmbBuff_s ownAddr = {.data = cmb.ownAddr,
                         .dataLen = CMB_ADDR_LEN,
                         .buffLen = CMB_ADDR_LEN};
    /* Fill addr */
    ret = CmbFillData(&ownAddr,out);
    if(ret != CMB_ROK) return ret;

    /* PDU */
    cmbBuff_s pduIn ={.data = &in->data[CMB_ADDR_END],
            .buffLen = in->dataLen - CMB_ADDR_LEN - CMB_END_LEN - CMB_CRC_LEN,
            .dataLen = in->dataLen - CMB_ADDR_LEN - CMB_END_LEN - CMB_CRC_LEN};
    cmbBuff_s pduOut ={.data = &out->data[CMB_ADDR_END],
            .buffLen = out->buffLen - CMB_ADDR_LEN - CMB_END_LEN - CMB_CRC_LEN,
            .dataLen = 0};
    ret = CmbPduProcess(&pduIn,&pduOut);
    if(ret != CMB_ROK) return ret;
    ret = CmbFillData(&pduOut,out);
    if(ret != CMB_ROK) return ret;


    /* Fill crc */
    cmbCrc outCrcI;
    outCrcI = CmbFastCRC16ModBus(out->data, out->dataLen);
    ret = CmbFillInt (outCrcI, CMB_CRC_LEN, 1, out);
    if(ret != CMB_ROK) return ret;

    /* Fill end */
    cmbBuff_s end = {.data = cmb.endBytes,
            .buffLen = CMB_END_LEN,
            .dataLen = CMB_END_LEN};
    ret = CmbFillData(&end,out);
    if(ret != CMB_ROK) return ret;

    if(addrFlag == 1)// borad cast no response
    {
        out->dataLen = 0;
        return CMB_RBROADCAST;
    }

#if DBG_LVL >= DBG_LOG
    var = 0;
    _DBG_LOG_HDR("D", "0");
    rt_kprintf("TxADU:");
    rt_kprintf("\033[32m");
    for (; var < out->dataLen && var < CMB_ADDR_END; ++var) {
        rt_kprintf("%02X ",out->data[var]);
    }
    rt_kprintf("\033[33m");
    for (; var < out->dataLen && var < CMB_FUNCC_END; ++var) {
        rt_kprintf("%02X ",out->data[var]);
    }
    rt_kprintf("\033[0m");
    for (; var < out->dataLen - CMB_CRC_LEN - CMB_END_LEN; ++var) {
        rt_kprintf("%02X ",out->data[var]);
    }
    rt_kprintf("\033[34m");
    for (; var < out->dataLen - CMB_END_LEN; ++var) {
        rt_kprintf("%02X ",out->data[var]);
    }
    rt_kprintf("\033[35m");
    for (; var < out->dataLen; ++var) {
        rt_kprintf("%02X ",out->data[var]);
    }
    _DBG_LOG_X_END;
#endif
    return ret;
}



/**
* @brief Adu Process
* @param in:  input data
* @param out: output data
* @retval 0
*/
static cmbRet_e CmbPduProcess(const cmbBuff_s* in, cmbBuff_s* out)
{
#if DBG_LVL >= DBG_LOG

    int var = 0;
    _DBG_LOG_HDR("D", "0");
    rt_kprintf("RxPDU:");
    rt_kprintf("\033[33m");
    for (; var < in->dataLen && var < CMB_FUNCC_LEN; ++var) {
        rt_kprintf("%02X ",in->data[var]);
    }
    rt_kprintf("\033[0m");
    for (; var < in->dataLen; ++var) {
        rt_kprintf("%02X ",in->data[var]);
    }
    _DBG_LOG_X_END;
#endif
    cmbRet_e ret = CMB_ROK;

    cmbFuncCode funcC;
    ret = CmbPduExtFuncC(in, &funcC);
    if(ret != CMB_ROK) return ret;
    rt_kprintf("  FunctionCode: %02X\r\n",funcC);

    cmbBuff_s funcIn={.buffLen = in->buffLen - CMB_FUNCC_LEN,
                      .dataLen = in->dataLen - CMB_FUNCC_LEN,
                      .data = &in->data[CMB_FUNCC_LEN]};
    cmbBuff_s funcOut={.buffLen = out->buffLen - CMB_FUNCC_LEN,
                      .dataLen = 0,
                      .data = &out->data[CMB_FUNCC_LEN]};
    if(funcC < CMB_FUNC_NUM && cmb.functionP[funcC])
    {/* function code supported */
        ret = cmb.functionP[funcC](&funcIn,&funcOut);
        if(ret != CMB_ROK)
        {/*  exception */
            ret = CmbFillInt (funcC | 1 << (8*CMB_FUNCC_LEN-1), CMB_FUNCC_LEN, 0, out);
            if(ret != CMB_ROK) return ret;
        }else
        {/* normal */
            ret = CmbFillInt (funcC, CMB_FUNCC_LEN, 0, out);
            if(ret != CMB_ROK) return ret;
        }
        ret = CmbFillData(&funcOut,out);
        if(ret != CMB_ROK) return ret;
    }else
    {/* function code unsupported */
        ret = CmbFillInt (funcC | 1 << (8*CMB_FUNCC_LEN-1), CMB_FUNCC_LEN, 0, out);
        if(ret != CMB_ROK) return ret;
        ret = CmbFillInt (CMB_ILLEGAL_FUNCTION, CMB_EXCC_LEN, 0, out);
        if(ret != CMB_ROK) return ret;
    }

#if DBG_LVL >= DBG_LOG
    var = 0;
    _DBG_LOG_HDR("D", "0");
    rt_kprintf("TxPDU:");
    rt_kprintf("\033[33m");
    for (; var < out->dataLen && var < CMB_FUNCC_LEN; ++var) {
        rt_kprintf("%02X ",out->data[var]);
    }
    rt_kprintf("\033[0m");
    for (; var < out->dataLen; ++var) {
        rt_kprintf("%02X ",out->data[var]);
    }
    _DBG_LOG_X_END;
#endif

    return ret;
}



/**
* @brief Adu extract address
* @param in:  input data
* @param addr: out addr
* @retval refer 'cmbRet_e'
*/
static cmbRet_e CmbAduExtAddr(const cmbBuff_s* in, cmbBuff_s* addr)
{
    RT_ASSERT(in != RT_NULL)
    RT_ASSERT(addr != RT_NULL)

    if (in->dataLen < CMB_ADDR_END)
        return CMB_RDATA_TO_SHORT;
    if (addr->buffLen < CMB_ADDR_LEN)
        return CMB_RBUFF_TO_SMALL;

    addr->dataLen = 0;
    for(uint32_t i = CMB_ADDR_STA;i < CMB_ADDR_END ;i++)
    {
        addr->data[addr->dataLen++] = in->data[i];
    }
    return CMB_ROK;
}



/**
* @brief Pdu extract function code
* @param in:  input data
* @param funcC: function code
* @retval refer 'cmbRet_e'
*/
static cmbRet_e CmbPduExtFuncC(const cmbBuff_s* in, cmbFuncCode* funcC)
{
    RT_ASSERT(in != RT_NULL)
    RT_ASSERT(funcC != RT_NULL)

    if (in->dataLen < CMB_FUNCC_LEN)
        return CMB_RDATA_TO_SHORT;

    *funcC = 0;
    for(uint32_t i = 0;i < CMB_FUNCC_LEN ;i++)
    {
        *funcC |= in->data[i] << (i*8);
    }
    return CMB_ROK;
}



/**
* @brief Adu extract CRC
* @param in:  input data
* @param crc: out CRC
* @retval refer 'cmbRet_e'
*/
static cmbRet_e CmbAduExtCrc(const cmbBuff_s* in, cmbBuff_s* crc)
{
    RT_ASSERT(in != RT_NULL)
    RT_ASSERT(crc != RT_NULL)

    if (crc->buffLen < CMB_CRC_LEN)
        return CMB_RBUFF_TO_SMALL;

    crc->dataLen = 0;
    for(uint32_t i = in->dataLen - CMB_CRC_LEN - CMB_END_LEN;
        i < in->dataLen - CMB_END_LEN;
        i++)
    {
        crc->data[crc->dataLen++] = in->data[i];
    }
    return CMB_ROK;
}



/**
* @brief Fill data
* @param data: Data will be filled
* @param buff: Target buffer
* @retval refer 'cmbRet_e'
*/
static cmbRet_e CmbFillData(const cmbBuff_s* data, cmbBuff_s* buff)
{
    RT_ASSERT(data != RT_NULL)
    RT_ASSERT(buff != RT_NULL)

    if (data->dataLen > buff->buffLen - buff->dataLen)
        return CMB_RBUFF_TO_SMALL;

    for(uint32_t i = 0;i < data->dataLen ;i++)
    {
        buff->data[buff->dataLen++] = data->data[i];
    }
    return CMB_ROK;
}



/**
* @brief Data compare address
* @param d1: data 1
* @param d2: data 2
* @param size: compare size
* @retval 1: equal
*         0: different
*/
static uint32_t CmbDataCompare(const void* d1, const void* d2, uint32_t size)
{
    RT_ASSERT(d1 != RT_NULL)
    RT_ASSERT(d2 != RT_NULL)
    RT_ASSERT(size != 0)

    uint8_t* d1d = (uint8_t*)d1;
    uint8_t* d2d = (uint8_t*)d2;
    for(uint32_t i = 0; i < size; i++)
    {
        if(*(d1d++) != *(d2d++))
            return 0;
    }
    return 1;
}



/**
* @brief Modbus CRC calculat
* @param puchMsgP: data calculated
* @param usDataLen: data length
* @retval crc value
*/
static cmbCrc CmbFastCRC16ModBus (const void *puchMsgP,uint32_t  usDataLen)
{
    uint8_t* puchMsg = (uint8_t*)puchMsgP;
    cmbCrc uchCRCHi = 0xFF ;
    cmbCrc uchCRCLo = 0xFF ;
    uint32_t uIndex = 0;
    while (usDataLen--)
    {
        uIndex = uchCRCLo ^ *puchMsg++ ;
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
        uchCRCHi = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}


/**
* @brief Fill intger
* @param intger: intger
* @param size: intger size
* @param endian: 0:big endian, other value: little endian
* @retval crc value
*/
static cmbRet_e CmbFillInt (uint32_t intger,uint32_t size,uint8_t endian, cmbBuff_s* buff)
{
    RT_ASSERT(buff != RT_NULL);

    if(buff->buffLen < CMB_CRC_LEN) return CMB_RBUFF_TO_SMALL;
    if(endian)//little
        for(uint32_t i = 0; i < size; ++i)
        {
            buff->data[buff->dataLen++] = intger;
            intger >>= 8;
        }
    else//big
        for(uint32_t i = 0; i < size; ++i)
        {
            buff->data[buff->dataLen++] = intger >> ((size - i - 1)*8);
        }
    return CMB_ROK;
}



#if DBG_LVL >= DBG_LOG
static void CmbPrintBuff (const char* msg ,const cmbBuff_s* data)
{
    RT_ASSERT(msg != RT_NULL);
    RT_ASSERT(data != RT_NULL);

    int var = 0;
    rt_kprintf("%s:",msg);
    for (; var < data->dataLen && var < data->dataLen; ++var) {
        rt_kprintf("%02X ",data->data[var]);
    }
    _DBG_LOG_X_END;
}
#endif


//写多个寄存器 16
/**
* @brief function read hold regs(read/write funcC 0x03)
* @param in: input data
* @param out: output data
* @retval refer 'cmbRet_e'
*/
cmbRet_e CmbReadHoldRegs(const cmbBuff_s* in, cmbBuff_s* out)
{
    RT_ASSERT(in != RT_NULL);
    RT_ASSERT(out != RT_NULL);

    if(in->dataLen < 4 )
    {
        CmbFillInt(CMB_ILLEGAL_DATA_VALUE,1,0,out);
        return CMB_RERR;
    }
    retRet_e ret = REG_ROK;
    uint32_t sAddr = 0;
    uint32_t regNum = 0;
    uint32_t dI = 0;
    uint16_t regBuff[CMB_MAX_OPR_NUM] = {0};
    out->dataLen = 0;
    sAddr |= in->data[dI++] << (8*1);
    sAddr |= in->data[dI++] << (8*0);
    regNum |= in->data[dI++] << (8*1);
    regNum |= in->data[dI++] << (8*0);

    /* reg quanity legal ? */
    if(regNum == 0 || regNum > CMB_MAX_OPR_NUM )
    {
        CmbFillInt(CMB_ILLEGAL_DATA_VALUE,1,0,out);
        return CMB_RERR;
    }
    /* addr legal ? */
    if( sAddr < 0x8000 || sAddr >= REG_RW_NUM + 0x8000)
    {
        CmbFillInt(CMB_ILLEGAL_DATA_ADDRESS,1,0,out);
        return CMB_RERR;
    }
    if(sAddr+regNum-1 < 0x8000 || sAddr+regNum-1 >= REG_RW_NUM + 0x8000)
    {
        CmbFillInt(CMB_ILLEGAL_DATA_ADDRESS,1,0,out);
        return CMB_RERR;
    }

    /* request process ? */
    ret = RegsRead(sAddr,regNum,sizeof(regBuff)/sizeof(regBuff[0]),regBuff);
    if(ret != REG_ROK)
    {
        CmbFillInt(CMB_SERVER_DEVICE_FAILURE,1,0,out);
        return CMB_RERR;
    }
    CmbFillInt(regNum*2,1,0,out);
    for (uint32_t i = 0; i < regNum; ++i)
    {
        CmbFillInt(regBuff[i],2,0,out);
    }
    return CMB_ROK;
}

/**
* @brief function read input regs(read only funcC 0x04)
* @param in: input data
* @param out: output data
* @retval refer 'cmbRet_e'
*/
cmbRet_e CmbReadInputRegs(const cmbBuff_s* in, cmbBuff_s* out)
{
    RT_ASSERT(in != RT_NULL);
    RT_ASSERT(out != RT_NULL);

    if(in->dataLen < 4 )
    {
        CmbFillInt(CMB_ILLEGAL_DATA_VALUE,1,0,out);
        return CMB_RERR;
    }
    retRet_e ret = REG_ROK;
    uint32_t sAddr = 0;
    uint32_t regNum = 0;
    uint32_t dI = 0;
    uint16_t regBuff[CMB_MAX_OPR_NUM] = {0};
    out->dataLen = 0;
    sAddr |= in->data[dI++] << (8*1);
    sAddr |= in->data[dI++] << (8*0);
    regNum |= in->data[dI++] << (8*1);
    regNum |= in->data[dI++] << (8*0);

    /* reg quanity legal ? */
    if(regNum == 0 || regNum > CMB_MAX_OPR_NUM )
    {
        CmbFillInt(CMB_ILLEGAL_DATA_VALUE,1,0,out);
        return CMB_RERR;
    }
    /* addr legal ? */
    if(sAddr >= REG_RO_NUM)
    {
        CmbFillInt(CMB_ILLEGAL_DATA_ADDRESS,1,0,out);
        return CMB_RERR;
    }
    if(sAddr+regNum-1 == 0 || sAddr+regNum-1 >= REG_RO_NUM)
    {
        CmbFillInt(CMB_ILLEGAL_DATA_ADDRESS,1,0,out);
        return CMB_RERR;
    }

    /* request process ? */
    ret = RegsRead(sAddr,regNum,sizeof(regBuff)/sizeof(regBuff[0]),regBuff);
    if(ret != REG_ROK)
    {
        CmbFillInt(CMB_SERVER_DEVICE_FAILURE,1,0,out);
        return CMB_RERR;
    }
    CmbFillInt(regNum*2,1,0,out);
    for (uint32_t i = 0; i < regNum; ++i)
    {
        CmbFillInt(regBuff[i],2,0,out);
    }
    return CMB_ROK;
}



cmbRet_e CmbWriteMulRegs(const cmbBuff_s* in, cmbBuff_s* out)
{
    RT_ASSERT(in != RT_NULL);
    RT_ASSERT(out != RT_NULL);
    if(in->dataLen < 5 )
    {
        CmbFillInt(CMB_ILLEGAL_DATA_VALUE,1,0,out);
        return CMB_RERR;
    }
    retRet_e ret = REG_ROK;
    uint32_t sAddr = 0;
    uint32_t regNum = 0;
    uint8_t byteCount = 0;
    uint32_t dI = 0;
    uint16_t regBuff[CMB_MAX_OPR_NUM] = {0};
    out->dataLen = 0;
    sAddr |= in->data[dI++] << (8*1);
    sAddr |= in->data[dI++] << (8*0);
    regNum |= in->data[dI++] << (8*1);
    regNum |= in->data[dI++] << (8*0);
    byteCount = in->data[dI++];

    /* reg quanity legal ? */
    if(byteCount != in->dataLen - 5)
    {
        CmbFillInt(CMB_ILLEGAL_DATA_VALUE,1,0,out);
        return CMB_RERR;
    }
    if(regNum == 0 || regNum > CMB_MAX_OPR_NUM || byteCount != regNum * 2)
    {
        CmbFillInt(CMB_ILLEGAL_DATA_VALUE,1,0,out);
        return CMB_RERR;
    }
    /* addr legal ? */
    if( sAddr < 0x8000 || sAddr >= REG_RW_NUM + 0x8000)
    {
        CmbFillInt(CMB_ILLEGAL_DATA_ADDRESS,1,0,out);
        return CMB_RERR;
    }
    if(sAddr+regNum-1 < 0x8000 || sAddr+regNum-1 >= REG_RW_NUM + 0x8000)
    {
        CmbFillInt(CMB_ILLEGAL_DATA_ADDRESS,1,0,out);
        return CMB_RERR;
    }

    for(uint16_t i = 0; i < regNum; i++)
    {
        regBuff[i] |= ((uint16_t)in->data[dI++])<<8;
        regBuff[i] |= in->data[dI++];
    }
    /* request process ? */
    ret = RegsWrite(sAddr, regNum, regBuff);
    if(ret != REG_ROK)
    {
        CmbFillInt(CMB_SERVER_DEVICE_FAILURE,1,0,out);
        return CMB_RERR;
    }
    CmbFillInt(sAddr,1,0,out);
    CmbFillInt(regNum,1,0,out);
    return CMB_ROK;
}
/* Function end ------------------------------------------------------------ */



