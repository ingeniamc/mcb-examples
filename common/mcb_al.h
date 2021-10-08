/**
 * @file mcb_al.h
 * @brief This file contains the adaptation layer of MCB.
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2020. All rights reserved.
 */

#ifndef MCB_AL_H_
#define MCB_AL_H_

#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif /* STM32G474xx */

#include "main.h"

/* System static defined spi2 instance */
extern SPI_HandleTypeDef hspi1;

/** Link between system spi instance and MCB */
#define tMCBSpiInst     hspi1

/** Number of instances of MCB */
#define MCB_NMB_INST    (uint16_t)1U
/** Instance ID for MCB 0 */
#define MCB_INST0       (uint16_t)0U

/** GPIO macros */
#define GPIO_PIN_VALUE_LOW          GPIO_PIN_RESET
#define GPIO_PIN_VALUE_HIGH         GPIO_PIN_SET

/**
 * Mcb adaptation layer initialization
 *
 * @param u16Id[in] ID of the MCB, useful when using multiple MCB instances.
 */
void
McbAL_Init(uint16_t u16Id);

#endif /* MCB_AL_H_ */
