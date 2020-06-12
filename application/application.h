/**
 * @file application.h
 * @brief This file contains the application header function for mcb-examples
 *
 * @author  Firmware department
 * @copyright Ingenia Motion Control (c) 2020. All rights reserved.
 */

#include <stdint.h>

 /**
  * Initialize the application.
  */
void
AppInit(void);

/**
 * Start the application.
 */
void
AppStart(void);

/**
 * The application loop function called periodically.
 *
 * @retval NO_ERROR if all ok, error code otherwise.
 */
int32_t
AppLoop(void);
