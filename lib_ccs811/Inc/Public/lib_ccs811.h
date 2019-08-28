/*
 * Driver for AMS CCS811 digital gas sensor used with Azure Sphere developer
 * kit.
 *
 * This is a platform dependent wrapper for CCS811 driver.
 *
 * ---------------------------------------------------------------------------
 *
 * The BSD License (3-clause license)
 *
 * Copyright (c) 2019 Jaroslav Groman (https://github.com/jgroman)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

 /***************************************************************************//**
 * @file    lib_ccs811.h
 * @version 1.0.0
 *
 * @brief Header file for CCS811 platform dependent wrapper.
 *
 * @par Description
 *    .
 * @author
 *
 * @date
 *
 *******************************************************************************/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <applibs/log.h>
#include <applibs/i2c.h>
#include <applibs/gpio.h>

#include "ccs811.h"

// Platform dependent GPIO pin numbers

// Avnet Azure Sphere Starter Kit GPIO:
// Note: on Mikroe Air Quality 3 Click module CCS811 /WAKE signal is routed 
// to the mikroBUS CS pin.

#define SK_SOCKET12_INT_GPIO	2	// Starter Kit Socket 1 & 2 INT pin GPIO
#define SK_SOCKET1_RST_GPIO		16	// Starter Kit Socket 1 RST pin GPIO
#define SK_SOCKET2_RST_GPIO		17	// Starter Kit Socket 2 RST pin GPIO
#define SK_SOCKET1_CS_GPIO		34	// Starter Kit Socket 1 CS pin GPIO
#define SK_SOCKET2_CS_GPIO		35	// Starter Kit Socket 2 CS pin GPIO


/** @brief	Initialize CCS811 sensor.
 *
 * Allocates resources and optionally activates sensor /WAKE signal.
 *
 * @param i2c_fd_arg    File descriptor of the I2C interface.
 * @param i2c_addr      I2C address of CCS811 sensor.
 * @param cs_gpio       GPIO pin of /WAKE signal or -1 if not used.
 *
 * @return Pointer to the sensor device data structure.
 */
ccs811_t
*ccs811_open(int i2c_fd_arg, I2C_DeviceAddress i2c_addr, GPIO_Id cs_gpio);

/** @brief	Perform CCS811 hardware reset using /RST pin.
*
* @param  rst_gpio    GPIO pin connected to CCS811 /RST input.
*/
void
ccs811_hw_reset(GPIO_Id rst_gpio);

/** @brief	Free CCS811 driver associated resources.
*
* @param  p_ccs    Pointer to the sensor device data structure.
*/
void
ccs811_close(ccs811_t *p_ccs);

#ifdef __cplusplus
}
#endif

/* [] END OF FILE */
