#ifndef __CMB_H
#define __CMB_H

#include <rtthread.h>
#include "rtdevice.h"
#include "stm32l0xx.h"
#include <board.h>
#include "reg.h"
#include "sta_led.h"

/* config option */
#define CMB_UART_NAME        "uart2" /**< Uart buff size */
#define CMB_UART_BAUD_RATE   9600
#define CMB_UART_BUFF_SIZE   128    /**< Uart buff size */
#define CMB_UART_RX_TIMEOUT  500     /**< Uart buff size */

/* thread config option */
#define CMB_THREAD_STACK_SIZE       768   /**< Thread stack size */
#define CMB_THREAD_PRIORITY         10    /**< Thread  priority  */
#define CMB_THREAD_TICK             10

/* protocol config option */

/* addr config */
#define CMB_ADDR_LEN      12
#define CMB_ADDR_STA      0
#define CMB_ADDR_END      (CMB_ADDR_STA + CMB_ADDR_LEN)
/* func code config */
#define CMB_FUNCC_LEN     1
#define CMB_FUNCC_STA     (CMB_ADDR_END)
#define CMB_FUNCC_END     (CMB_FUNCC_STA + CMB_FUNCC_LEN)

/* crc config */
#define CMB_CRC_LEN       2

/* end config */
#define CMB_END_LEN       4
#define CMB_END_BYTES     {0xEE,0xEE,0xDD,0xDD}

/* once max operte regs number */
#define CMB_MAX_OPR_NUM   16

/* function config */
#define CMB_FUNC_NUM      127
/* func code*/
#define CMB_READ_HOLD_REGS_FUNCC       3
#define CMB_READ_INPUT_REGS_FUNCC      4
#define CMB_WRITE_MUL_REGS_FUNCC       16

/* exception code config */
#define CMB_EXCC_LEN      1



#endif /* __CMB_H */


