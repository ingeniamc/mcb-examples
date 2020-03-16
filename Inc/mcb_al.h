/**
 * @file mcb_al.h
 * @brief This file contains the adaption layer of mcb
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2020. All rights reserved.
 */

#ifndef MCB_AL_H_
#define MCB_AL_H_

#ifdef STM32G474xx
#include "stm32g4xx_hal.h"
#endif

#include "main.h"

/* System static defined spi2 instance */
extern SPI_HandleTypeDef hspi1;

/** Link between system spi instance and mcb */
#define tSpiInst1       hspi1

/** GPIO macros */
#define GPIO_PIN_VALUE_LOW          GPIO_PIN_RESET
#define GPIO_PIN_VALUE_HIGH         GPIO_PIN_SET


/**
 * Mcb adaption layer initialization
 */
void
McbAL_Init(uint16_t u16Id);

#endif /* MCB_AL_H_ */
