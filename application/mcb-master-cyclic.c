/**
 * @file mcb-master-cyclic.c
 * @brief This file contains application of mcb-master-cyclic project
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2020. All rights reserved.
 */

#include "application.h"
#include "registers.h"

#include <string.h>
#include "mcb.h"
#include "mcb_al.h"

/** MCBus timeout (in ms) */
#define MCB_TIMEOUT     (uint32_t)500UL

/** Error codes */
#define NO_ERROR                    (int16_t)0
#define MCB_CURRENT_STATUS_ERROR    (int16_t)-1
#define MCB_MAPPING_ERROR           (int16_t)-2
#define EXIT_APP                    (int16_t)-100

/** Number of mapped registers in TX direction */
#define MCB_TX_MAP_NMB              (uint16_t)2U
/** Number of mapped registers in RX direction */
#define MCB_RX_MAP_NMB              (uint16_t)2U

/**
 * Sets mapping and move MCB to cyclic state.
 *
 * @retval NO_ERROR if all ok, error code otherwise.
 */
static int16_t
SetMcb0CyclicMode(void);

/** MCB instances */
Mcb_TInst ptMcbInst[MCB_NMB_INST];

/** Cyclic data buffers */
static void* ppTxDatPoint[MCB_TX_MAP_NMB];
static void* ppRxDatPoint[MCB_RX_MAP_NMB];

/** Config over cyclic message status */
static Mcb_EStatus eCoCResult;

/** Counter of number of cyclic transactions */
static volatile uint32_t u32CycCnt;

/** Vbus local variable being read */
static volatile float fVBusRead;

void AppInit(void)
{
    /** Initialize mcb instance */
    McbAL_Init(MCB_INST0);
    Mcb_Init(&(ptMcbInst[MCB_INST0]), MCB_BLOCKING, MCB_INST0, false, MCB_TIMEOUT);
    HAL_Delay((uint32_t)1UL);

    /** Initialize variables */
    u32CycCnt = (uint32_t)0UL;
    eCoCResult = MCB_STANDBY;

    fVBusRead = (float)0.0f;
}

void AppStart(void)
{
    /** Set mapping and move MCB to cyclic state.
     * Return number can be:
     *    -> errorcode if failed
     *    -> cyclic size if successful */
    int16_t i16CycSt = SetMcb0CyclicMode();

    if (i16CycSt > NO_ERROR)
    {
        /** Cyclic state has been reached successfully */
        /** Set a new current Q setpoint */
        float fCurrentQSP = (float)1.1f;
        memcpy(ppRxDatPoint[1], (const void*)&fCurrentQSP, sizeof(fCurrentQSP));
    }
}

int32_t AppLoop(void)
{
    int32_t i32Ret = NO_ERROR;

    /** Perform a cyclic transfer */
    Mcb_CyclicProcess(&(ptMcbInst[MCB_INST0]), &eCoCResult);
    u32CycCnt++;

    /** Copy VBus variable from cyclic buffer to local variable */
    memcpy((void*)&fVBusRead, (const void*)ppTxDatPoint[1], sizeof(fVBusRead));

    /** After several iteration disable cyclic state and finish program */
    if (u32CycCnt > (uint32_t)0xFFFFUL)
    {
        Mcb_DisableCyclic(ptMcbInst);
        u32CycCnt = (uint32_t)0UL;
    }
    if (ptMcbInst[MCB_INST0].isCyclic == false)
    {
        i32Ret = EXIT_APP;
    }

    return i32Ret;
}

static int16_t SetMcb0CyclicMode(void)
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
         *   Statusword
         *   Bus Voltage value */
        ppTxDatPoint[0] = Mcb_TxMap(&(ptMcbInst[MCB_INST0]),
                                    REG_ADDR_STATUS_WORD, REG_SIZE_STATUS_WORD);
        ppTxDatPoint[1] = Mcb_TxMap(&(ptMcbInst[MCB_INST0]),
                                    REG_ADDR_BUS_VOLT_VALUE, REG_SIZE_BUS_VOLT_VALUE);

        /** Set as Rx Map:
         *   Controlword
         *   Current quadrature set-point */
        ppRxDatPoint[0] = Mcb_RxMap(&(ptMcbInst[MCB_INST0]),
                                    REG_ADDR_CONTROL_WORD, REG_SIZE_CONTROL_WORD);
        ppRxDatPoint[1] = Mcb_RxMap(&(ptMcbInst[MCB_INST0]),
                                    REG_ADDR_CURR_Q_SETPOINT, REG_SIZE_CURR_Q_SETPOINT);

        /** Check that all the TX registers are correctly mapped */
        for (uint8_t u8Idx = (uint8_t)0; u8Idx < MCB_TX_MAP_NMB; u8Idx++)
        {
            if (ppTxDatPoint[u8Idx] == NULL)
            {
                i16Ret = MCB_MAPPING_ERROR;
                break;
            }
        }
        /** Check that all the RX registers are correctly mapped */
        for (uint8_t u8Idx = (uint8_t)0; u8Idx < MCB_RX_MAP_NMB; u8Idx++)
        {
            if (ppRxDatPoint[u8Idx] == NULL)
            {
                i16Ret = MCB_MAPPING_ERROR;
                break;
            }
        }

        if ((ptMcbInst[MCB_INST0].tCyclicTxList.u8Mapped != MCB_TX_MAP_NMB)
            || (ptMcbInst[MCB_INST0].tCyclicRxList.u8Mapped != MCB_RX_MAP_NMB))
        {
            i16Ret = MCB_MAPPING_ERROR;
            break;
        }

        i16Ret = Mcb_EnableCyclic(&(ptMcbInst[MCB_INST0]));

    } while (false);

    return i16Ret;
}
