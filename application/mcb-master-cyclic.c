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
#define MCB_TIMEOUT                 (uint32_t)500UL

/** Error codes */
#define NO_ERROR                    (int16_t)0
#define MCB_CURRENT_STATUS_ERROR    (int16_t)-1
#define MCB_MAPPING_ERROR           (int16_t)-2
#define EXIT_APP                    (int16_t)-100

/** Blinking time, in ms */
#define LED_BLINK_MS                (uint32_t)250UL

/** Number of mapped registers in RX direction */
#define MCB_CYC_RX_NUM              (uint16_t)2U
/** Number of mapped registers in TX direction */
#define MCB_CYC_TX_NUM              (uint16_t)2U

#define MCB_CYC_RX_SLOT0            0
#define MCB_CYC_RX_SLOT1            1
#define MCB_CYC_TX_SLOT0            0
#define MCB_CYC_TX_SLOT1            1

#define MCB_CYC_RX_IDX_CONTROL_WORD MCB_CYC_RX_SLOT0
#define MCB_CYC_RX_IDX_CURR_Q_SP    MCB_CYC_RX_SLOT1
#define MCB_CYC_TX_IDX_STATUS_WORD  MCB_CYC_TX_SLOT0
#define MCB_CYC_TX_IDX_V_BUS        MCB_CYC_TX_SLOT1


/** LED blinking process */
static void
LEDProcess(void);

/**
 * Sets mapping and move MCB to cyclic state.
 *
 * @retval NO_ERROR if all ok, error code otherwise.
 */
static int16_t
SetMcb0CyclicMode(void);

/** MCB instances */
static Mcb_TInst ptMcbInst[MCB_NMB_INST];

/** Cyclic data buffers */
static void* ppRxDataPoint[MCB_CYC_RX_NUM];
static void* ppTxDataPoint[MCB_CYC_TX_NUM];

/** Config message */
static Mcb_TMsg tMcbMsg;

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

    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_VALUE_HIGH);
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
        HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_VALUE_HIGH);

        /** Set a new current Q set-point */
        float fCurrentQSP = (float)1.25f;
        memcpy(ppRxDataPoint[MCB_CYC_RX_IDX_CURR_Q_SP], (const void*)&fCurrentQSP,
                sizeof(fCurrentQSP));
    }
}

int32_t AppLoop(void)
{
    int32_t i32Ret = NO_ERROR;

    /** Perform a cyclic transfer */
    Mcb_CyclicProcess(&(ptMcbInst[MCB_INST0]), &eCoCResult);
    u32CycCnt++;

    /** Copy VBus variable from cyclic buffer to local variable */
    memcpy((void*)&fVBusRead, (const void*)ppTxDataPoint[MCB_CYC_TX_IDX_V_BUS],
            sizeof(fVBusRead));

    LEDProcess();

    /** After several iteration disable cyclic state and finish program */
    if (u32CycCnt > (uint32_t)10000UL)
    {
        Mcb_DisableCyclic(ptMcbInst);
        u32CycCnt = (uint32_t)0UL;
    }
    if (ptMcbInst[MCB_INST0].isCyclic == false)
    {
        i32Ret = EXIT_APP;

        /** Construct MCB read message */
        tMcbMsg.u16Addr = REG_ADDR_SW_VERSION;
        tMcbMsg.eStatus = MCB_STANDBY;
        memset((void*)tMcbMsg.u16Data, (uint16_t)0U, (MCB_MAX_DATA_SZ * sizeof(tMcbMsg.u16Data[(uint16_t)0U])));
        ptMcbInst[MCB_INST0].Mcb_Read(&(ptMcbInst[MCB_INST0]), &(tMcbMsg));

        HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_VALUE_HIGH);
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
        ppTxDataPoint[MCB_CYC_TX_IDX_STATUS_WORD] = Mcb_TxMap(&(ptMcbInst[MCB_INST0]),
                                    REG_ADDR_STATUS_WORD, REG_SIZE_STATUS_WORD);
        ppTxDataPoint[MCB_CYC_TX_IDX_V_BUS] = Mcb_TxMap(&(ptMcbInst[MCB_INST0]),
                                    REG_ADDR_BUS_VOLT_VALUE, REG_SIZE_BUS_VOLT_VALUE);

        /** Set as Rx Map:
         *   Controlword
         *   Current quadrature set-point */
        ppRxDataPoint[MCB_CYC_RX_IDX_CONTROL_WORD] = Mcb_RxMap(&(ptMcbInst[MCB_INST0]),
                                    REG_ADDR_CONTROL_WORD, REG_SIZE_CONTROL_WORD);
        ppRxDataPoint[MCB_CYC_RX_IDX_CURR_Q_SP] = Mcb_RxMap(&(ptMcbInst[MCB_INST0]),
                                    REG_ADDR_CURR_Q_SETPOINT, REG_SIZE_CURR_Q_SETPOINT);

        /** Check that all the TX registers are correctly mapped */
        for (uint8_t u8Idx = (uint8_t)0; u8Idx < MCB_CYC_TX_NUM; u8Idx++)
        {
            if (ppTxDataPoint[u8Idx] == NULL)
            {
                i16Ret = MCB_MAPPING_ERROR;
                break;
            }
        }
        /** Check that all the RX registers are correctly mapped */
        for (uint8_t u8Idx = (uint8_t)0; u8Idx < MCB_CYC_RX_NUM; u8Idx++)
        {
            if (ppRxDataPoint[u8Idx] == NULL)
            {
                i16Ret = MCB_MAPPING_ERROR;
                break;
            }
        }

        if ((ptMcbInst[MCB_INST0].tCyclicTxList.u8Mapped != MCB_CYC_TX_NUM)
            || (ptMcbInst[MCB_INST0].tCyclicRxList.u8Mapped != MCB_CYC_RX_NUM))
        {
            i16Ret = MCB_MAPPING_ERROR;
            break;
        }

        i16Ret = Mcb_EnableCyclic(&(ptMcbInst[MCB_INST0]));

    } while (false);

    return i16Ret;
}

static void LEDProcess(void)
{
    static uint32_t u32LastBlinkTime = (uint32_t)0UL;

    if ((HAL_GetTick() - u32LastBlinkTime) > LED_BLINK_MS)
    {
        HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, !HAL_GPIO_ReadPin(LD6_GPIO_Port, LD6_Pin));
        u32LastBlinkTime = HAL_GetTick();
    }
}
