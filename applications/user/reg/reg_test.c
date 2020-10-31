/* test ------------------------------------------------------------ */
#include "utest.h"
#include "reg.h"
static void Test_RegRW(void)
{
    reg temp;
    reg data = 0;
    uassert_true(RegWrite(0,++data)==REG_ROK);
    uassert_true(RegRead(0,&temp)==REG_ROK);
    uassert_true(temp == data);

    uassert_true(RegWrite(REG_RO_NUM-1,++data)==REG_ROK);
    uassert_true(RegRead(REG_RO_NUM-1,&temp)==REG_ROK);
    uassert_true(temp == data);

    uassert_true(RegWrite(0x8000,++data)==REG_ROK);
    uassert_true(RegRead(0x8000,&temp)==REG_ROK);
    uassert_true(temp == data);

    uassert_true(RegWrite(0x8000+REG_RO_NUM-1,++data)==REG_ROK);
    uassert_true(RegRead(0x8000+REG_RO_NUM-1,&temp)==REG_ROK);
    uassert_true(temp == data);

    uassert_true(RegWrite(REG_RO_NUM,++data)==REG_RADDR_ERR);
    uassert_true(RegRead(REG_RO_NUM,&temp)==REG_RADDR_ERR);

    uassert_true(RegWrite(0x7FFF,++data)==REG_RADDR_ERR);
    uassert_true(RegRead(0x7FFF,&temp)==REG_RADDR_ERR);

    uassert_true(RegWrite(0x8000+REG_RO_NUM,++data)==REG_RADDR_ERR);
    uassert_true(RegRead(0x8000+REG_RO_NUM,&temp)==REG_RADDR_ERR);

}

static void Test_RegsRW(void)
{
    uint16_t datas[16] ={0xDD00,0xDD01,0xDD02,0xDD03,0xDD04,0xDD05,0xDD06,0xDD07,0xDD08,0xDD09,0xDD0a,0xDD0b,0xDD0c,0xDD0e,0xDD0f};
    uint16_t rData[16] ={0};

    uassert_true(RegsRead(0, 16, 15, rData)==REG_RBUFF_TOO_SMALL);

    uassert_true(RegsWrite(0,16,datas)==REG_ROK);
    uassert_true(RegsRead(0, 16, 16, rData)==REG_ROK);
    uassert_true(datas[0] == rData[0]);
    uassert_true(datas[15] == rData[15]);

    uassert_true(RegsWrite(REG_RO_NUM-16,16,datas)==REG_ROK);
    uassert_true(RegsRead(REG_RO_NUM-16, 16, 16, rData)==REG_ROK);
    uassert_true(datas[0] == rData[0]);
    uassert_true(datas[15] == rData[15]);

    uassert_true(RegsWrite(0x8000,16,datas)==REG_ROK);
    uassert_true(RegsRead(0x8000, 16, 16, rData)==REG_ROK);
    uassert_true(datas[0] == rData[0]);
    uassert_true(datas[15] == rData[15]);

    uassert_true(RegsWrite(0x8000+REG_RW_NUM-16,16,datas)==REG_ROK);
    uassert_true(RegsRead(0x8000+REG_RW_NUM-16, 16, 16, rData)==REG_ROK);
    uassert_true(datas[0] == rData[0]);
    uassert_true(datas[15] == rData[15]);

    uassert_true(RegsWrite(REG_RO_NUM-15,16,datas)==REG_RADDR_ERR);
    uassert_true(RegsRead(REG_RO_NUM-15, 16, 16, rData)==REG_RADDR_ERR);

    uassert_true(RegsWrite(0x7FFF,16,datas)==REG_RADDR_ERR);
    uassert_true(RegsRead(0x7FFF, 16, 16, rData)==REG_RADDR_ERR);

    uassert_true(RegsWrite(0x8000+REG_RW_NUM-15,16,datas)==REG_RADDR_ERR);
    uassert_true(RegsRead(0x8000+REG_RW_NUM-15, 16, 16, rData)==REG_RADDR_ERR);

}



static rt_err_t RegTestInit(void)
{
    return RT_EOK;
}

static rt_err_t RegTestCleanup(void)
{
    return RT_EOK;
}

static void RegTestCase(void)
{
    UTEST_UNIT_RUN(Test_RegRW);
    UTEST_UNIT_RUN(Test_RegsRW);
}
UTEST_TC_EXPORT(RegTestCase, "Reg.reg_test", RegTestInit, RegTestCleanup, 10);
