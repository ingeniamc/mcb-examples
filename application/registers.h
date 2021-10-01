/**
 * @file registers.h
 * @brief This file contains the drive registers
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2021. All rights reserved.
 */

#ifndef REGISTERS_H_
#define REGISTERS_H_

#include <stdint.h>

/** Control word */
#define REG_ADDR_CONTROL_WORD       (uint16_t)0x010U
#define REG_SIZE_CONTROL_WORD       sizeof(uint16_t)

/** Status word */
#define REG_ADDR_STATUS_WORD        (uint16_t)0x011U
#define REG_SIZE_STATUS_WORD        sizeof(uint16_t)

/** Current quadrature set-point */
#define REG_ADDR_CURR_Q_SETPOINT    (uint16_t)0x01AU
#define REG_SIZE_CURR_Q_SETPOINT    sizeof(float)

/** Bus Voltage value */
#define REG_ADDR_BUS_VOLT_VALUE     (uint16_t)0x060U
#define REG_SIZE_BUS_VOLT_VALUE     sizeof(float)

/** Vendor ID */
#define REG_ADDR_VENDOR_ID          (uint16_t)0x6E0U
#define REG_SIZE_VENDOR_ID          sizeof(uint32_t)

/** Software version */
#define REG_ADDR_SW_VERSION         (uint16_t)0x6E4U

#endif /* REGISTERS_H_ */
