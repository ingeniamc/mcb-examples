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

/* MCBus timeout (in ms).
 * Set to FF's for debugging so it never timeouts.
 * Set to e.g. 500 ms for real application. */
#define MCB_TIMEOUT     (uint32_t)0xFFFFFFFFUL /* (uint32_t)500UL */

/** Error codes */
#define NO_ERROR                    (int16_t)0
#define MCB_CURRENT_STATUS_ERROR    (int16_t)-1
#define MCB_MAPPING_ERROR           (int16_t)-2

#define MCB_TX_MAP_NMB              (uint16_t)2U
#define MCB_RX_MAP_NMB              (uint16_t)2U

/**
 * Sets mapping and move MCB to cyclic state.
 *
 * @retval NO_ERROR if all ok, error code otherwise.
 */
static int16_t
SetMcb0CyclicMode(void);

/** MCBus instance */
static Mcb_TInst ptMcbInst[MCB_NMB_INST];

/** Cyclic data buffers */
static void* ppTxDatPoint[MCB_TX_MAP_NMB];
static void* ppRxDatPoint[MCB_RX_MAP_NMB];

/** Config message */
static Mcb_TMsg tMcbMsg;
/** Config over cyclic message status */
static Mcb_EStatus eCoCResult;

/** Counter of number of cyclic transactions */
static volatile uint32_t u32CycCnt;

/** Vbus local variable being read */
static volatile uint32_t u32VBusRead;

void AppInit(void)
{
    /** Initialize MCB instance */
    McbAL_Init(MCB_INST0);
    Mcb_Init(&(ptMcbInst[MCB_INST0]), MCB_BLOCKING, MCB_INST0, false, MCB_TIMEOUT);
    HAL_Delay((uint32_t)1UL);

    /** Initialize variables */
    u32CycCnt = (uint32_t)0UL;
    eCoCResult = MCB_STANDBY;

    u32VBusRead = (uint32_t)0UL;

    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
}

void AppStart(void)
{
    /** Construct MCB read message.
     * Address 0x6E0 is vendor ID register */
    tMcbMsg.u16Addr = 0x6E0;
    tMcbMsg.eStatus = MCB_STANDBY;
    memset((void*)tMcbMsg.u16Data, (uint16_t)0U, (MCB_MAX_DATA_SZ * sizeof(tMcbMsg.u16Data[(uint16_t)0U])));

    /** Initial MCB config read */
    ptMcbInst[MCB_INST0].Mcb_Read(&(ptMcbInst[MCB_INST0]), &(tMcbMsg));

    /** More MCB config reads */
    do
    {
        tMcbMsg.eStatus = MCB_STANDBY;
        memset((void*)tMcbMsg.u16Data, (uint16_t)0U, (MCB_MAX_DATA_SZ * sizeof(tMcbMsg.u16Data[(uint16_t)0U])));
        ptMcbInst[MCB_INST0].Mcb_Read(&(ptMcbInst[MCB_INST0]), &(tMcbMsg));

        HAL_Delay(100);
    } while (0);

    /** Set mapping and move MCB to cyclic state.
     * Return number can be:
     *    -> errorcode if failed
     *    -> cyclic size if successful */
    int16_t i16CycSt = SetMcb0CyclicMode();

    if (i16CycSt > NO_ERROR)
    {
    	/** Cyclic state has been reached successfully */
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);

		/** Set a new current Q set-point */
		float fCurrentQSP = (float)1.1f;
		memcpy(ppRxDatPoint[1], (const void*)&fCurrentQSP, sizeof(float));
    }
}

int32_t AppLoop(void)
{
    int32_t i32Ret = NO_ERROR;

    /** The config message is managed by the cyclic process function,
    *   a single config message needs a set of multiple cyclic process to
    *   receive the config message answer. */
    if ((eCoCResult == MCB_READ_SUCCESS) || (eCoCResult == MCB_STANDBY))
    {
    	/** Clear the message and send a new request */
    	tMcbMsg.eStatus = MCB_STANDBY;
    	memset((void*)tMcbMsg.u16Data, (uint16_t)0U, (MCB_MAX_DATA_SZ * sizeof(tMcbMsg.u16Data[(uint16_t)0U])));
        ptMcbInst[MCB_INST0].Mcb_Read(&(ptMcbInst[MCB_INST0]), &(tMcbMsg));
    }
    else if (eCoCResult == MCB_READ_ERROR)
    {
        eCoCResult = MCB_STANDBY;
    }

    /** After several iterations disable cyclic state and finish program */
    if (u32CycCnt > (uint32_t)10000UL)
    {
        Mcb_DisableCyclic(ptMcbInst);
        u32CycCnt = (uint32_t)0UL;
        HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    }
    if (ptMcbInst[MCB_INST0].isCyclic == false)
    {
        i32Ret = (int32_t)-1L;
    }

    return i32Ret;
}

void AppCyclicProcess(void)
{
    if (ptMcbInst[MCB_INST0].isCyclic != false)
    {
        /** Perform a cyclic transfer */
        Mcb_CyclicProcess(&(ptMcbInst[MCB_INST0]), &eCoCResult);
        u32CycCnt++;

        /** Copy VBus variable to local */
        memcpy((void*)&u32VBusRead, (const void*)ppTxDatPoint[1], sizeof(u32VBusRead));
    }
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
         *   Statusword : Address 0x011, Type unt16_t
         *   Bus Voltage value : Address 0x060, Type uint32_t */
        ppTxDatPoint[0] = Mcb_TxMap(&(ptMcbInst[MCB_INST0]),
                                    (uint16_t)0x0011, sizeof(uint16_t));
        ppTxDatPoint[1] = Mcb_TxMap(&(ptMcbInst[MCB_INST0]),
                                    (uint16_t)0x0060, sizeof(uint32_t));

        /** Set as Rx Map:
         *   Controlword : Address 0x010, Type unt16_t
         *   Current quadrature set-point : Address 0x022, Type int16_t */
        ppRxDatPoint[0] = Mcb_RxMap(&(ptMcbInst[MCB_INST0]),
                                    (uint16_t)0x0010, sizeof(uint16_t));
        ppRxDatPoint[1] = Mcb_RxMap(&(ptMcbInst[MCB_INST0]),
                                    (uint16_t)0x0022, sizeof(int16_t));
        for (uint8_t u8Idx = (uint8_t)0; u8Idx < MCB_TX_MAP_NMB; ++u8Idx)
        {
            /** Check that all the registers are correctly mapped,
             *  otherwise return an error. */
            if ((ppTxDatPoint[u8Idx] == NULL) || (ppRxDatPoint[u8Idx] == NULL))
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
