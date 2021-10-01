/**
 * @file mcb-master-cyclic.c
 * @brief This file contains application of mcb-master-cyclic project
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
#define MCB_CURRENT_STATUS_ERROR    (int16_t)-1
#define MCB_MAPPING_ERROR           (int16_t)-2

#define MCB_TX_MAP_NMB              (uint16_t)2U
#define MCB_RX_MAP_NMB              (uint16_t)2U

int16_t
SetMcb0CyclicMode(void);

Mcb_TInst ptMcbInst[MCB_NMB_INST];

void* ppTxDatPoint[MCB_TX_MAP_NMB];
void* ppRxDatPoint[MCB_RX_MAP_NMB];

/** MCB read message */
Mcb_TMsg tMcbMsg;

void AppInit(void)
{
    /** Initialize mcb instance */
    McbAL_Init(MCB_INST0);
    Mcb_Init(&(ptMcbInst[MCB_INST0]), MCB_BLOCKING, MCB_INST0, false, MCB_TIMEOUT);
    HAL_Delay((uint32_t)1UL);
}

void AppStart(void)
{
    /** Construct MCB read message */
    /** Software version */
    tMcbMsg.u16Addr = 0x6E4;
    tMcbMsg.eStatus = MCB_STANDBY;
    memset((void*)tMcbMsg.u16Data, (uint16_t)0U, (MCB_MAX_DATA_SZ * sizeof(tMcbMsg.u16Data[(uint16_t)0U])));

    /** Set mapping and move MCB to cyclic state */
    int16_t i16CycSt = SetMcb0CyclicMode();

    if (i16CycSt != NO_ERROR)
    {
        return -1;
    }


    uint32_t u32CycCnt = (uint32_t)0UL;
    Mcb_EStatus eResult = MCB_STANDBY;
    /** Set a new current Q setpoint */
    float fCurrentQSP = (float)1.1f;
    memcpy(ppRxDatPoint[1], (const void*)&fCurrentQSP, sizeof(float));

    float fCurQRead = (float)0.0f;
}

void AppLoop(void)
{
    int32_t i32Ret = NO_ERROR;
    /** Perform a cyclic transfer */
    Mcb_CyclicProcess(&(ptMcbInst[MCB_INST0]), &eResult);
    u32CycCnt++;

    /** Copy current Q to local */
    memcpy((void*)&fCurQRead, (const void*)ppTxDatPoint[1], sizeof(float));
    /** The config message is managed by the cyclic process function,
    *   a single config message needs a set of multiple cyclic process to
    *   receive the config message answer.  */
    if (eResult == MCB_READ_SUCCESS)
    {
        ptMcbInst[MCB_INST0].Mcb_Read(&(ptMcbInst[MCB_INST0]), &(tMcbMsg));
    }

    /** After several iteration disable cyclic state and finish program */
    if (u32CycCnt > (uint32_t)0xFFFFUL)
    {
        Mcb_DisableCyclic(ptMcbInst);
    }
    if (ptMcbInst[MCB_INST0].isCyclic == false)
    {
        i32Ret = MCB_CURRENT_STATUS_ERROR;
    }

    return i32Ret;
}

int16_t SetMcb0CyclicMode(void)
{
    int16_t i16Ret = NO_ERROR;

    do
    {
        if (ptMcbInst[MCB_INST0].isCyclic != false)
        {
            i16Ret = MCB_CURRENT_STATUS_ERROR;
            break;
        }

        Mcb_UnmapAll(&(ptMcbInst[MCB_INST0]));

        /** Set as Tx Map:
         *   Statusword : Key 0x011, Type unt16_t
         *   Current quadrature value : Key 0x03B, Type float */
        ppTxDatPoint[0] = Mcb_TxMap(&(ptMcbInst[MCB_INST0]),
                                    (uint16_t)0x0011, sizeof(uint16_t));
        ppTxDatPoint[1] = Mcb_TxMap(&(ptMcbInst[MCB_INST0]),
                                    (uint16_t)0x003B, sizeof(float));

        /** Set as Rx Map:
         *   Controlword : Key 0x010, Type unt16_t
         *   Current quadrature set-point : Key 0x01A, Type float */
        ppRxDatPoint[0] = Mcb_RxMap(&(ptMcbInst[MCB_INST0]),
                                    (uint16_t)0x0010, sizeof(uint16_t));
        ppRxDatPoint[1] = Mcb_RxMap(&(ptMcbInst[MCB_INST0]),
                                    (uint16_t)0x001A, sizeof(float));
        for (uint8_t u8Idx = (uint8_t)0; u8Idx < MCB_TX_MAP_NMB; ++u8Idx)
        {
            /** Check that all the registers are correctly mapped,
             *  otherwise return an error. */
            if ((ppTxDatPoint[u8Idx] == NULL) || (ppRxDatPoint[u8Idx] == NULL))
            {
                i16Ret = MCB_MAPPING_ERROR;
            }
        }

        if ((ptMcbInst[MCB_INST0].tCyclicTxList.u8Mapped != MCB_TX_MAP_NMB)
            || (ptMcbInst[MCB_INST0].tCyclicRxList.u8Mapped != MCB_RX_MAP_NMB))
        {
            i16Ret = MCB_MAPPING_ERROR;
        }

        i16Ret = Mcb_EnableCyclic(&(ptMcbInst[MCB_INST0]));

    } while (false);

    return i16Ret;
}
