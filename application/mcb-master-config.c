/**
 * @file mcb-master-config.c
 * @brief This file contains application of mcb-master-config project
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2020. All rights reserved.
 */

#include "application.h"
#include "registers.h"

#include <string.h>
#include "mcb.h"
#include "mcb_al.h"

/** MCBus timeout, in ms */
#define MCB_TIMEOUT     (uint32_t)500UL

/** Error codes */
#define NO_ERROR        (int16_t)0
#define EXIT_APP        (int16_t)-100

/** Blinking time, in ms */
#define LED_BLINK_MS    (uint32_t)250UL

/** LED blinking process */
static void
LEDProcess(void);

/** MCB instances */
static Mcb_TInst ptMcbInst[MCB_NMB_INST];

/** Config message */
static Mcb_TMsg tMcbMsg;

void AppInit(void)
{
    /** Initialize MCB instance */
    McbAL_Init(MCB_INST0);
    Mcb_Init(&(ptMcbInst[MCB_INST0]), MCB_BLOCKING, MCB_INST0, false, MCB_TIMEOUT);

    HAL_Delay((uint32_t)1UL);

    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_VALUE_HIGH);
}

void AppStart(void)
{
    /** Construct MCB get info message */
    Mcb_TInfoMsg tMcbInfoMsg;

    tMcbInfoMsg.u16Addr = REG_ADDR_SW_VERSION;
    tMcbInfoMsg.eStatus = MCB_STANDBY;
    ptMcbInst[MCB_INST0].Mcb_GetInfo(&(ptMcbInst[MCB_INST0]), &(tMcbInfoMsg));

    /** Construct MCB read message */
    tMcbMsg.u16Addr = REG_ADDR_STATUS_WORD;
    tMcbMsg.eStatus = MCB_STANDBY;
    memset((void*)tMcbMsg.u16Data, (uint16_t)0U, (MCB_MAX_DATA_SZ * sizeof(tMcbMsg.u16Data[(uint16_t)0U])));
    ptMcbInst[MCB_INST0].Mcb_Read(&(ptMcbInst[MCB_INST0]), &(tMcbMsg));
}

int32_t AppLoop(void)
{
    int32_t i32RetErr = NO_ERROR;

    /** Construct MCB read message */
    tMcbMsg.u16Addr = REG_ADDR_SW_VERSION;
    tMcbMsg.eStatus = MCB_STANDBY;
    memset((void*)tMcbMsg.u16Data, (uint16_t)0U, (MCB_MAX_DATA_SZ * sizeof(tMcbMsg.u16Data[(uint16_t)0U])));
    ptMcbInst[MCB_INST0].Mcb_Read(&(ptMcbInst[MCB_INST0]), &(tMcbMsg));

    HAL_Delay((uint32_t)1UL);

    LEDProcess();

    return i32RetErr;
}

static void LEDProcess(void)
{
    static uint32_t u32LastBlinkTime = (uint32_t)0UL;

    if ((HAL_GetTick() - u32LastBlinkTime) > LED_BLINK_MS)
    {
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, !HAL_GPIO_ReadPin(LD4_GPIO_Port, LD4_Pin));
        u32LastBlinkTime = HAL_GetTick();
    }
}
