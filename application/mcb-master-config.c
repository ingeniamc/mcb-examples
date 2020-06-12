/**
 * @file mcb-master-config.c
 * @brief This file contains application of mcb-master-config project
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2020. All rights reserved.
 */

#include "application.h"

#include <string.h>
#include "mcb.h"
#include "mcb_al.h"

#define MCB_NMB_INST    (uint16_t)1U
#define MCB_TIMEOUT     (uint32_t)500UL

/** Error codes */
#define NO_ERROR                    (int16_t)0

Mcb_TInst ptMcbInst[MCB_NMB_INST];

/** MCB read message */
Mcb_TMsg tMcbMsg;

void AppInit(void)
{
    /** Initialize mcb instance */
    McbAL_Init(MCB_INST0);
    Mcb_Init(&(ptMcbInst[MCB_INST0]), MCB_BLOCKING, MCB_INST0, false, MCB_TIMEOUT);
    HAL_Delay((uint32_t)1UL);

    /** Construct mcb get info message */
    Mcb_TInfoMsg tMcbInfoMsg;
    /** Software version */
    tMcbInfoMsg.u16Addr = 0x6E4;
    tMcbInfoMsg.eStatus = MCB_STANDBY;
    ptMcbInst[MCB_INST0].Mcb_GetInfo(&(ptMcbInst[MCB_INST0]), &(tMcbInfoMsg));
}

void AppStart(void)
{
    /** Construct MCB read message */
    /** Software version */
    tMcbMsg.u16Addr = 0x6E4;
    tMcbMsg.eStatus = MCB_STANDBY;
    memset((void*)tMcbMsg.u16Data, (uint16_t)0U, (MCB_MAX_DATA_SZ * sizeof(tMcbMsg.u16Data[(uint16_t)0U])));
}

int32_t AppLoop(void)
{
    int32_t i32RetErr = NO_ERROR;

    HAL_Delay((uint32_t)1UL);
    ptMcbInst[MCB_INST0].Mcb_Read(&(ptMcbInst[MCB_INST0]), &(tMcbMsg));

    return i32RetErr;
}
