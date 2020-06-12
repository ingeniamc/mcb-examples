/**
 * @file mcb_al.c
 * @brief This file contains the adaption layer of mcb
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2020. All rights reserved.
 */

#include "mcb_al.h"
#include "mcb_usr.h"

#include <stdbool.h>

/** Default SPI transmission timeout */
#define SPI_TRANSMISSION_TIMEOUT    (uint32_t)100UL

void McbAL_Init(uint16_t u16Id)
{
    switch (u16Id)
    {
        case MCB_INST0:
            /** Set Moco into reset state during instance initialization */
            HAL_GPIO_WritePin(MOCO1_RESET_GPIO_Port, MOCO1_RESET_Pin, GPIO_PIN_VALUE_LOW);

            while (Mcb_IntfReadIRQ(u16Id) == (uint8_t)0x01);
            HAL_GPIO_WritePin(MOCO1_RESET_GPIO_Port, MOCO1_RESET_Pin, GPIO_PIN_VALUE_HIGH);
            while (Mcb_IntfReadIRQ(u16Id) == (uint8_t)0x00);
            break;
        default:
            /* Nothing */
            break;
    }

}

uint8_t Mcb_IntfReadIRQ(uint16_t u16Id)
{
    uint8_t u8Ret = (uint8_t)0;

    switch (u16Id)
    {
        case MCB_INST0:
            u8Ret = (HAL_GPIO_ReadPin(MCB1_IRQ_GPIO_Port, MCB1_IRQ_Pin) == GPIO_PIN_VALUE_HIGH);
            break;
        default:
            /* Nothing */
            break;
    }

    return u8Ret;
}

void Mcb_IntfSPITransfer(uint16_t u16Id, uint16_t* pu16In,
                         uint16_t* pu16Out, uint16_t u16Sz)
{
    switch (u16Id)
    {
        case MCB_INST0:
#if defined (MCB1_CS_GPIO_Port)
            HAL_GPIO_WritePin(MCB1_CS_GPIO_Port, MCB1_CS_Pin, GPIO_PIN_VALUE_LOW);
#endif
            HAL_SPI_TransmitReceive(&tSpiInst1, (uint8_t*)pu16In,
                                    (uint8_t*)pu16Out, u16Sz, SPI_TRANSMISSION_TIMEOUT);
            break;
        default:
            /* Nothing */
            break;
    }
}

uint16_t Mcb_IntfComputeCrc(const uint16_t* pu16Buf, uint16_t u16Sz)
{
    return (uint16_t)0U;
}

bool Mcb_IntfCheckCrc(uint16_t u16Id, const uint16_t* pu16Buf, uint16_t u16Sz)
{
    bool bCrc = (bool)false;

    switch (u16Id)
    {
        case MCB_INST0:
            bCrc = !(HAL_SPI_GetError(&tSpiInst1) == HAL_SPI_ERROR_CRC);
            break;
    }
    return bCrc;
}

uint32_t Mcb_GetMillis(void)
{
    /** Return milliseconds */
    return HAL_GetTick();
}

bool Mcb_IntfIsReady(uint16_t u16Id)
{
    bool isReady = false;

    switch (u16Id)
    {
        case MCB_INST0:
            isReady = (tSpiInst1.State == HAL_SPI_STATE_READY);
            break;
        default:
            /* Nothing */
            break;
    }
    return isReady;
}

