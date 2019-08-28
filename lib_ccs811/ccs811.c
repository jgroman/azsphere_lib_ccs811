/*
 * Driver for AMS CCS811 digital gas sensor connected to I2C.
 *
 * This driver was adopted from https://github.com/gschorcht/ccs811-esp-idf
 * and further modified. This is a platform independent part of CSS811 driver
 * library.
 *
 * ---------------------------------------------------------------------------
 *
 * The BSD License (3-clause license)
 *
 * Copyright (c) 2017 Gunar Schorcht (https://github.com/gschorcht)
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
 * @file    ccs811.c
 * @version 1.0.0
 *
 * @brief Driver for AMS CCS811 digital gas sensor connected to I2C.
 *
 * @par Target device
 *    CCS811
 *
 * @author Gunar Schort
 *
 * @author Jaroslav Groman
 *
 * @date
 *
 ******************************************************************************/

#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <stdarg.h>

#include "ccs811.h"

#if defined(CCS811_DEBUG_LEVEL_1) || defined(CCS811_DEBUG_LEVEL_2)
#define ERROR(s, f, ...) ccs811_printf(p_ccs, "%s %s: " s "\n", "CCS811", f, ## __VA_ARGS__)
#define ERROR_DEV(s, f, d, ...) ccs811_printf(p_ccs, "%s %s: addr 0x%02X - " s "\n", "CCS811", f, d->i2c_addr, ## __VA_ARGS__)
#else
#define ERROR(s, f, ...)
#define ERROR_DEV(s, f, d, ...)
#endif

#if defined(CCS811_DEBUG_LEVEL_2)
#define DEBUG(s, f, ...) ccs811_printf(p_ccs, "%s %s: " s "\n", "CCS811", f, ## __VA_ARGS__)
#define DEBUG_DEV(s, f, d, ...) ccs811_printf(p_ccs, "%s %s (0x%02X): " s "\n", "CCS811", f, d->i2c_addr, ## __VA_ARGS__)
#else
#define DEBUG(s, f, ...)
#define DEBUG_DEV(s, f, d, ...)
#endif

#define CCS811_HW_ID				0x81	// CCS81x Hardware Identifier

// CCS811 registers
#define CCS811_REG_STATUS          0x00
#define CCS811_REG_MEAS_MODE       0x01
#define CCS811_REG_ALG_RESULT_DATA 0x02
#define CCS811_REG_RAW_DATA        0x03
#define CCS811_REG_ENV_DATA        0x05
#define CCS811_REG_NTC             0x06
#define CCS811_REG_THRESHOLDS      0x10
#define CCS811_REG_BASELINE        0x11

#define CCS811_REG_HW_ID           0x20
#define CCS811_REG_HW_VER          0x21
#define CCS811_REG_FW_BOOT_VER     0x23
#define CCS811_REG_FW_APP_VER      0x24

#define CCS811_REG_ERROR_ID        0xE0

#define CCS811_REG_APP_ERASE       0xF1
#define CCS811_REG_APP_DATA        0xF2
#define CCS811_REG_APP_VERIFY      0xF3
#define CCS811_REG_APP_START       0xF4
#define CCS811_REG_SW_RESET        0xFF

// Status register bits
#define CCS811_STATUS_ERROR        0x01  // Error, details in CCS811_REG_ERROR
#define CCS811_STATUS_DATA_RDY     0x08  // New data sample in ALG_RESULT_DATA
#define CCS811_STATUS_APP_VALID    0x10  // Valid application firmware loaded
#define CCS811_STATUS_FW_MODE      0x80  // Firmware is in application mode

// Error register bits
#define CCS811_ERR_WRITE_REG_INV   0x01  // Invalid register address on write
#define CCS811_ERR_READ_REG_INV    0x02  // Invalid register address on read
#define CCS811_ERR_MEASMODE_INV    0x04  // Invalid requested measurement mode
#define CCS811_ERR_MAX_RESISTANCE  0x08  // Maximum sensor resistance exceeded 
#define CCS811_ERR_HEATER_FAULT    0x10  // Heater current not in range
#define CCS811_ERR_HEATER_SUPPLY   0x20  // Heater voltage not applied correctly

/*******************************************************************************
* Type declarations
*******************************************************************************/

typedef struct {
	uint8_t reserved_1  : 2;
	uint8_t int_thresh  : 1; // Interrupt on ALG_RESULT_DAT over set thresholds
	uint8_t int_datardy : 1; // Interrupt on new sample ready in ALG_RESULT_DAT 
	uint8_t drive_mode  : 3; // Mode number binary coded
} ccs811_meas_mode_reg_t;

/*******************************************************************************
* Forward declarations of private functions
*******************************************************************************/

/** @brief CCS811 Register Read.
 *
 * Reads register addr @p r_addr contents into @p p_data.
 *
 * @param p_ccs		Pointer to CCS811 data descriptor
 * @param reg_addr	Register address
 * @param p_data	Pointer to register value storage
 * @param data_len	Register value length
 *
 * @return	True if read successful, false otherwise.
 */
static bool
ccs811_reg_read(ccs811_t *p_ccs, uint8_t reg_addr, uint8_t *p_data,
    uint32_t data_len);

/** @brief CCS811 Register Write.
 *
 * Write to register at addr @p r_addr contents of @p p_data.
 *
 * @param p_ccs		Pointer to CCS811 data descriptor
 * @param reg_addr	Register address
 * @param p_data	Pointer to register value storage
 * @param data_len	Register value length
 *
 * @return	True if read successful, false otherwise.
 */
static bool
ccs811_reg_write(ccs811_t* p_ccs, uint8_t reg_addr, uint8_t *p_data,
    uint32_t data_len);

/** @brief Check CCS811 error status.
 *
 * Reads CCS811 STATUS register. If the ERROR bit is set, checks ERROR_ID
 * register for detailed error information. Error code is stored to CCS811
 * data descriptor struct.
 *
 * @param p_ccs		Pointer to CCS811 data descriptor
 *
 * @return	True if no error, false otherwise.
 */
static bool
ccs811_check_error_status(ccs811_t *p_ccs);

/** @brief Set CCS811 threshold interrupt mode.
 *
 * @param p_ccs		                Pointer to CCS811 data descriptor
 * @param b_is_int_mode_enabled     Interrupt enabled flag
 *
 * @return	True if no error, false otherwise.
 */
static bool
ccs811_enable_threshold(ccs811_t *p_ccs, bool b_is_int_mode_enabled);

/**
 * @brief Check CCS811 availability.
 *
 * Tries to read hardware and firmware version registers of CCS811 and checks
 * CCS811 hardware version.
 *
 * @param p_ccs		Pointer to CCS811 data descriptor
 *
 * @return	True on success, false otherwise.
 */
static bool
ccs811_is_available(ccs811_t *p_ccs);

/** @brief Platform printf function.
 *
 * Redirects to platform dependent printf implementation.
 *
 * @param p_ccs		Pointer to CCS811 data descriptor
 * @param p_format  Printed string format
 * @param ...       Printed variables
 *
 * @return	Platform printf function result.
 */
static int
ccs811_printf(ccs811_t *p_ccs, const char *p_format, ...);

/** @brief Platform delay function.
 *
 * Redirects to platform dependent delay implementation.
 *
 * @param p_ccs	        Pointer to CCS811 data descriptor.
 * @param delay_msg     Delay type msg (CCS811_MSG_DELAY_x).
 * @param delay_len     Delay length.
 *
 * @return	Platform delay operation result.
 */
static int
ccs811_delay(ccs811_t* p_ccs, uint8_t delay_msg, size_t delay_len);

/** @brief Platform I2C Write function.
 *
 * Redirects to platform dependent implementation.
 *
 * @param p_ccs		Pointer to CCS811 data descriptor.
 * @param p_buffer  Pointer to data to be written to I2C.
 * @param buf_len   Length of data to be written.
 *
 * @return	Platform I2C Write operation result.
 */
static int
ccs811_i2c_write_bytes(ccs811_t *p_ccs, uint8_t *p_buffer, size_t buf_len);

/** @brief Platform I2C Read function.
 *
 * Redirects to platform dependent implementation.
 *
 * @param p_ccs		Pointer to CCS811 data descriptor
 * @param p_buffer  Pointer to data read from I2C.
 * @param buf_len   Length of data read.
 *
 * @return	Platform I2C Read operation result.
 */
static int
ccs811_i2c_read_bytes(ccs811_t* p_ccs, uint8_t* p_buffer, size_t buf_len);

/*******************************************************************************
* Function definitions
*******************************************************************************/

ccs811_t
*ccs811_init(uint8_t i2c_addr, ccs811_msg_cb platform_cb)
{
	// CCS initialization sequence according to application note AN000369

	ccs811_t *p_ccs;

	if ((p_ccs = malloc(sizeof(ccs811_t))) == NULL) 
	{
		return NULL;
	}

	// Init sensor data structure
	p_ccs->i2c_addr = i2c_addr;
	p_ccs->mode = CCS811_MODE_IDLE;
	p_ccs->error_code = CCS811_OK;
	p_ccs->platform_cb = platform_cb;

	// Check whether sensor is available including the check of the hardware
	// id and the error state
	DEBUG_DEV("--- Checking hardware ID", __FUNCTION__, p_ccs);
	if (!ccs811_is_available(p_ccs)) 
	{
		ERROR_DEV("Sensor is not available.", __FUNCTION__, p_ccs);
		free(p_ccs);
		return NULL;
	}

	const static uint8_t sw_reset[4] = { 0x11, 0xe5, 0x72, 0x8a };

	// Perform a software reset
	DEBUG_DEV("--- Software reset", __FUNCTION__, p_ccs);
	if (!ccs811_reg_write(p_ccs, CCS811_REG_SW_RESET, (uint8_t*)sw_reset, 4))
	{
		ERROR_DEV("Could not reset the sensor.", __FUNCTION__, p_ccs);
		free(p_ccs);
		return NULL;
	}
	
	// Wait tSTART time after the reset
	ccs811_delay(p_ccs, CCS811_MSG_DELAY_US, CCS811_TIME_START_RESET_US);

	uint8_t status;

	// Get the status to check for APP_VALID
	DEBUG_DEV("--- Read Status register", __FUNCTION__, p_ccs);
	if (!ccs811_reg_read(p_ccs, CCS811_REG_STATUS, &status, 1)) 
	{
		ERROR_DEV("Could not read status register 0x%02X.", 
            __FUNCTION__, p_ccs, CCS811_REG_STATUS);
		free(p_ccs);
		return NULL;
	}

	// If sensor is in bootloader mode (FW_MODE == 0), it has to switch
	// to the application mode first
	if (!(status & CCS811_STATUS_FW_MODE)) 
	{
		// Check whether valid application firmware is loaded
		if (!(status & CCS811_STATUS_APP_VALID)) 
		{
			ERROR_DEV("Sensor is in boot mode, but has no valid application.",
				__FUNCTION__, p_ccs);
			free(p_ccs);
			return NULL;
		}

		// Switch to application mode
		DEBUG_DEV("--- Switching to application mode", __FUNCTION__, p_ccs);
		if (!ccs811_reg_write(p_ccs, CCS811_REG_APP_START, 0, 0)) 
		{
			ERROR_DEV("Could not start application", __FUNCTION__, p_ccs);
			free(p_ccs);
			return NULL;
		}

		// Wait for firmware to start
		ccs811_delay(p_ccs, CCS811_MSG_DELAY_US, CCS811_TIME_APP_START_US);

		// Get the status to check whether sensor switched to application mode
		DEBUG_DEV("--- Read Status register", __FUNCTION__, p_ccs);
		if (!ccs811_reg_read(p_ccs, CCS811_REG_STATUS, &status, 1) ||
			!(status & CCS811_STATUS_FW_MODE))
		{
			ERROR_DEV("Could not start application.", __FUNCTION__, p_ccs);
			free(p_ccs);
			return NULL;
		}
	}

	// Try to set the default measurement mode to CCS811_MODE_1S
	DEBUG_DEV("--- Set default measurement mode", __FUNCTION__, p_ccs);
	if (!ccs811_set_mode(p_ccs, CCS811_MODE_1S)) 
	{
		free(p_ccs);
		return NULL;
	}

	return p_ccs;
}

void 
ccs811_shutdown(ccs811_t *p_ccs) 
{
	if (p_ccs)
	{
		ccs811_set_mode(p_ccs, CCS811_MODE_IDLE);
	}
	free(p_ccs);
}

bool 
ccs811_set_mode(ccs811_t* p_ccs, ccs811_mode_t mode) 
{
	ccs811_meas_mode_reg_t reg;

	if (!p_ccs) 
	{
		return false;
	}

	p_ccs->error_code = CCS811_OK;

	// Read current measurement mode register value
	if (!ccs811_reg_read(p_ccs, CCS811_REG_MEAS_MODE, (uint8_t*)&reg, 1)) 
	{
		ERROR_DEV("Error reading MEAS_MODE register.", __FUNCTION__, p_ccs);
		return false;
	}

	// Update regiter value with measurement mode
	reg.drive_mode = mode;
	
	// Write back measurement mode register value
	if (!ccs811_reg_write(p_ccs, CCS811_REG_MEAS_MODE, (uint8_t*)&reg, 1)) 
	{
		ERROR_DEV("Could not write measurement mode.", __FUNCTION__, p_ccs);
		return false;
	}

	// Check whether setting measurement mode were succesfull
	if (!ccs811_reg_read(p_ccs, CCS811_REG_MEAS_MODE, (uint8_t*)&reg, 1)) 
	{
		ERROR_DEV("Error reading MEAS_MODE register.", __FUNCTION__, p_ccs);
		return false;
	}

	if (reg.drive_mode != mode) 
	{
		ERROR_DEV("Measurement mode not set to %d", __FUNCTION__, p_ccs, mode);
		return ccs811_check_error_status(p_ccs);
	}

	p_ccs->mode = mode;

	return true;
}

#define CCS811_ALG_DATA_ECO2_HB   0
#define CCS811_ALG_DATA_ECO2_LB   1
#define CCS811_ALG_DATA_TVOC_HB   2
#define CCS811_ALG_DATA_TVOC_LB   3
#define CCS811_ALG_DATA_STATUS    4
#define CCS811_ALG_DATA_ERROR_ID  5
#define CCS811_ALG_DATA_RAW_HB    6
#define CCS811_ALG_DATA_RAW_LB    7

bool 
ccs811_get_results(ccs811_t *p_ccs, 
    uint16_t *iaq_tvoc, uint16_t *iaq_eco2, uint8_t *raw_i, uint16_t *raw_v)
{
	if (!p_ccs) 
	{
		return false;
	}

	p_ccs->error_code = CCS811_OK;

	if (p_ccs->mode == CCS811_MODE_IDLE)
	{
		ERROR_DEV("Sensor is in idle mode and not performing measurements.",
			__FUNCTION__, p_ccs);
		p_ccs->error_code = CCS811_DRV_WRONG_MODE;
		return false;
	}

	if (p_ccs->mode == CCS811_MODE_250MS && (iaq_tvoc || iaq_eco2))
	{
		ERROR_DEV("Sensor is in constant power mode, only raw data "
			"are available every 250ms", __FUNCTION__, p_ccs);
		p_ccs->error_code = CCS811_DRV_NO_IAQ_DATA;
		return false;
	}

	uint8_t data[8];

	// read IAQ sensor values and RAW sensor data including status and error id
	if (!ccs811_reg_read(p_ccs, CCS811_REG_ALG_RESULT_DATA, data, 8))
	{
		ERROR_DEV("Could not read sensor data.", __FUNCTION__, p_ccs);
		p_ccs->error_code |= CCS811_DRV_RD_DATA_FAILED;
		return false;
	}

	// check for errors
	if (data[CCS811_ALG_DATA_STATUS] & CCS811_STATUS_ERROR)
	{
		return ccs811_check_error_status(p_ccs);
	}

	// Check for new data ready, if not, latest values are read from sensor
	// and error_code is set
	if (!(data[CCS811_ALG_DATA_STATUS] & CCS811_STATUS_DATA_RDY))
	{
		DEBUG_DEV("No new data.", __FUNCTION__, p_ccs);
		p_ccs->error_code = CCS811_DRV_NO_NEW_DATA;
	}

	// If *iaq* is not NULL return IAQ sensor values
	if (iaq_tvoc) 
	{
		*iaq_tvoc = (uint16_t)(data[CCS811_ALG_DATA_TVOC_HB] << 8 | 
                               data[CCS811_ALG_DATA_TVOC_LB]);
	}

	if (iaq_eco2) 
	{
		*iaq_eco2 = (uint16_t)(data[CCS811_ALG_DATA_ECO2_HB] << 8 | 
                               data[CCS811_ALG_DATA_ECO2_LB]);
	}

	// If *raw* is not NULL return RAW sensor data
	if (raw_i) 
	{
		*raw_i = (uint16_t)(data[CCS811_ALG_DATA_RAW_HB] >> 2);
	}

	if (raw_v) 
	{
		*raw_v = (uint16_t)((data[CCS811_ALG_DATA_RAW_HB] & 0x03) << 8 | 
                             data[CCS811_ALG_DATA_RAW_LB]);
	}

	return true;
}

uint32_t 
ccs811_get_ntc_resistance(ccs811_t *p_ccs, uint32_t r_ref)
{
	uint32_t result = 0;
	if (p_ccs)
	{
		uint8_t data[4];
		// Read baseline register
		if (ccs811_reg_read(p_ccs, CCS811_REG_NTC, data, 4))
		{
			// Calculation from application note ams AN000372
			uint16_t v_ref = (uint16_t)((data[0]) << 8 | data[1]);
			uint16_t v_ntc = (uint16_t)((data[2]) << 8 | data[3]);

			result = v_ntc * r_ref / v_ref;
		}
	}
	return result;
}

bool 
ccs811_set_environmental_data(ccs811_t *p_ccs, 
    float temperature, float humidity)
{
	bool result = false;
	if (p_ccs) 
	{
		uint16_t temp = (uint16_t)((temperature + 25) * 512);// -25°C maps to 0
		uint16_t hum = (uint16_t)(humidity * 512);

		// Fill environmental data
		uint8_t env_data[4] = {
			(uint8_t)(temp >> 8), 
			(uint8_t)(temp & 0xff),
			(uint8_t)(hum >> 8), 
			(uint8_t)(hum & 0xff) 
		};

		// Send environmental data to the sensor
		if (ccs811_reg_write(p_ccs, CCS811_REG_ENV_DATA, env_data, 4))
		{
			result = true;
		}
		else
		{
			ERROR_DEV("Could not write environmental data to sensor.", 
				__FUNCTION__, p_ccs);
		}
	}
	return result;
}

bool 
ccs811_set_eco2_thresholds(ccs811_t *p_ccs, 
    uint16_t low, uint16_t high, uint8_t hysteresis)
{
	if (!p_ccs) 
	{
		return false;
	}

	p_ccs->error_code = CCS811_OK;

	// Check whether interrupt has to be disabled
	if (!low && !high && !hysteresis)
		return ccs811_enable_threshold(p_ccs, false);

	// check parameters
	if (low < CCS_ECO2_RANGE_MIN || 
		high > CCS_ECO2_RANGE_MAX || 
		low > high || 
		!hysteresis)
	{
		ERROR_DEV("Wrong threshold parameters", __FUNCTION__, p_ccs);
		p_ccs->error_code = CCS811_DRV_WRONG_PARAMS;
		return ccs811_enable_threshold(p_ccs, false);
	}

	// Fill the threshold data
	uint8_t thr_data[5] = { (uint8_t)(low >> 8), (uint8_t)(low & 0xff),
						(uint8_t)(high >> 8), (uint8_t)(high & 0xff),
						hysteresis };

	// Write threshold data to the sensor
	if (!ccs811_reg_write(p_ccs, CCS811_REG_THRESHOLDS, thr_data, 5))
	{
		ERROR_DEV("Could not write threshold interrupt data to sensor.", 
            __FUNCTION__, p_ccs);
		return ccs811_enable_threshold(p_ccs, false);
	}

	// Enable the threshold interrupt mode    
	return ccs811_enable_threshold(p_ccs, true);
}

bool 
ccs811_enable_interrupt(ccs811_t *p_ccs, bool b_is_int_enabled)
{
	bool result = false;
	if (p_ccs)
	{
		ccs811_meas_mode_reg_t meas_mode_reg;

		// Read measurement mode register value
		if (ccs811_reg_read(p_ccs, CCS811_REG_MEAS_MODE,
			(uint8_t*)&meas_mode_reg, 1))
		{
			meas_mode_reg.int_datardy = b_is_int_enabled;
			meas_mode_reg.int_thresh = false; // Threshold mode must not enabled

			// Write updated measurement mode register value
			if (ccs811_reg_write(p_ccs, CCS811_REG_MEAS_MODE,
				(uint8_t*)&meas_mode_reg, 1))
			{
				result = true;
			}
			else
			{
				ERROR_DEV("Could not set measurement mode register.", 
					__FUNCTION__, p_ccs);
			}
		}
	}
	return result;
}

uint16_t 
ccs811_get_baseline(ccs811_t *p_ccs)
{
	uint16_t result = 0;
	if (p_ccs)
	{
		uint8_t baseline_data[2];
		if (ccs811_reg_read(p_ccs, CCS811_REG_BASELINE, baseline_data, 2))
		{
			result = (uint16_t)((baseline_data[0]) << 8 | baseline_data[1]);
		}
	}
	return result;
}

bool 
ccs811_set_baseline(ccs811_t *p_ccs, uint16_t baseline)
{
	bool result = false;
	if (p_ccs)
	{
		uint8_t data[2] = {(uint8_t)(baseline >> 8),(uint8_t)(baseline & 0xFF)};
		if (ccs811_reg_write(p_ccs, CCS811_REG_BASELINE, data, 2)) 
		{
			result = true;
		}
	}
	return result;
}


/*******************************************************************************
* Private function definitions
*******************************************************************************/

static bool
ccs811_reg_read(ccs811_t *p_ccs, uint8_t reg_addr, uint8_t *p_data,
	uint32_t data_len)
{
	bool result = false;
	if (p_ccs && p_data)
	{
		DEBUG_DEV(" READ [%02X] bytes %d", __FUNCTION__, p_ccs, reg_addr,
			data_len);
		// Select register address
		ccs811_i2c_write_bytes(p_ccs, &reg_addr, 1);
		// Read register value bytes
		int i2c_result = ccs811_i2c_read_bytes(p_ccs, p_data, data_len);
		if (i2c_result == -1) {
			p_ccs->error_code |= (errno == EBUSY) ?
				CCS811_I2C_BUSY : CCS811_I2C_READ_FAILED;
			ERROR_DEV("Error %d on read %d byte from I2C reg addr 0x%02X.",
				__FUNCTION__, p_ccs, errno, data_len, reg_addr);
		}
		else
		{
			result = true;
#			ifdef CCS811_DEBUG_LEVEL_2
			ccs811_printf(p_ccs, "CCS811 %s (0x%02X):  READ ",
				__FUNCTION__, p_ccs->i2c_addr);
			for (int i = 0; i < data_len; i++) {
				ccs811_printf(p_ccs, "%02X ", p_data[i]);
			}
			ccs811_printf(p_ccs, "\n");
#			endif
		}
	}
	return result;
}

static bool
ccs811_reg_write(ccs811_t* p_ccs, uint8_t reg_addr, uint8_t *p_data,
	uint32_t data_len)
{
	bool result = false;
	if (p_ccs)
	{
		uint8_t buffer[data_len + 1];

		buffer[0] = reg_addr;
		for (uint32_t i = 0; i < data_len; i++) {
			buffer[i + 1] = p_data[i];
		}

		DEBUG_DEV("WRITE [%02X] bytes %d", __FUNCTION__, p_ccs, reg_addr,
			data_len);

#		ifdef CCS811_DEBUG_LEVEL_2
		if (p_data && data_len) {
			ccs811_printf(p_ccs, "CCS811 %s (0x%02X): WRITE ",
				__FUNCTION__, p_ccs->i2c_addr);
			for (int i = 0; i <= data_len; i++) {
				ccs811_printf(p_ccs, "%02X ", buffer[i]);
			}
			ccs811_printf(p_ccs, "\n");
		}
#		endif

		int i2c_result = ccs811_i2c_write_bytes(p_ccs, buffer, data_len + 1);
		if (i2c_result == -1) {
			p_ccs->error_code |= (errno == EBUSY) ?
				CCS811_I2C_BUSY : CCS811_I2C_WRITE_FAILED;
			ERROR_DEV("Error %d on write %d byte to I2C slave register 0x%02X.",
				__FUNCTION__, p_ccs, errno, data_len, reg_addr);
		}
		else
		{
			result = true;
		}
	}
	return result;
}

static bool
ccs811_enable_threshold(ccs811_t *p_ccs, bool b_is_int_mode_enabled)
{
	bool result = false;
	if (p_ccs)
	{
		ccs811_meas_mode_reg_t meas_mode_reg;

		// Enable/disable the data ready interrupt
		if (ccs811_enable_interrupt(p_ccs, b_is_int_mode_enabled))
		{
			// Read measurement mode register value
			if (ccs811_reg_read(p_ccs, CCS811_REG_MEAS_MODE, 
				(uint8_t *)&meas_mode_reg, 1))
			{
				// Set the threshold interrupt mode
				meas_mode_reg.int_thresh = b_is_int_mode_enabled;

				// Write back measurement mode register
				if (ccs811_reg_write(p_ccs, CCS811_REG_MEAS_MODE, 
					(uint8_t *)&meas_mode_reg, 1))
				{
					result = true;
				}
				else
				{
					ERROR_DEV("Could not set measurement mode register.", 
						__FUNCTION__, p_ccs);
				}
			}
		}
	}
	return result;
}

static bool 
ccs811_check_error_status(ccs811_t *p_ccs) 
{
	if (!p_ccs) 
	{
		return false;
	}

	p_ccs->error_code = CCS811_OK;

	// Read STATUS register
	uint8_t status;
	DEBUG_DEV("--- Read STATUS register", __FUNCTION__, p_ccs);
	if (!ccs811_reg_read(p_ccs, CCS811_REG_STATUS, &status, 1)) 
	{
		return false;
	}

	if (!(status & CCS811_STATUS_ERROR)) 
	{
		return true;	// No errors
	}

	// Read ERROR_ID register
	uint8_t err_reg;
	DEBUG_DEV("--- Read ERROR_ID register", __FUNCTION__, p_ccs);
	if (!ccs811_reg_read(p_ccs, CCS811_REG_ERROR_ID, &err_reg, 1)) 
	{
		return false;
	}

	// Apply error bit masks to ERROR_ID
	if (err_reg & CCS811_ERR_WRITE_REG_INV)
	{
		ERROR_DEV("Invalid register for write.", __FUNCTION__, p_ccs);
		p_ccs->error_code = CCS811_DRV_WR_REG_INV;
		return false;
	}

	if (err_reg & CCS811_ERR_READ_REG_INV) 
	{
		ERROR_DEV("Invalid register for read.", __FUNCTION__, p_ccs);
		p_ccs->error_code = CCS811_DRV_RD_REG_INV;
		return false;
	}

	if (err_reg & CCS811_ERR_MEASMODE_INV) 
	{
		ERROR_DEV("Invalid measurement mode request.", __FUNCTION__, p_ccs);
		p_ccs->error_code = CCS811_DRV_MM_INV;
		return false;
	}

	if (err_reg & CCS811_ERR_MAX_RESISTANCE) 
	{
		ERROR_DEV("Sensor resistance measurement has reached"
			" or exceeded the maximum range.", __FUNCTION__, p_ccs);
		p_ccs->error_code = CCS811_DRV_MAX_RESIST;
		return false;
	}

	if (err_reg & CCS811_ERR_HEATER_FAULT) 
	{
		ERROR_DEV("Heater current not in range.", __FUNCTION__, p_ccs);
		p_ccs->error_code = CCS811_DRV_HEAT_FAULT;
		return false;
	}

	if (err_reg & CCS811_ERR_HEATER_SUPPLY) 
	{
		ERROR_DEV("Heater voltage is not being applied correctly.", 
			__FUNCTION__, p_ccs);
		p_ccs->error_code = CCS811_DRV_HEAT_SUPPLY;
		return false;
	}

	return true;
}

static bool 
ccs811_is_available(ccs811_t *p_ccs)
{
	if (!p_ccs) 
	{
		return false;
	}

	uint8_t hw_id_data;

	// Check hardware ID (register 0x20)
	if (!ccs811_reg_read(p_ccs, CCS811_REG_HW_ID, &hw_id_data, 1))
	{
		return false;
	}

	if (hw_id_data != CCS811_HW_ID) 
	{
		ERROR_DEV("Wrong hardware ID 0x%02X, should be 0x%02X",
			__FUNCTION__, p_ccs, hw_id_data, CCS811_HW_ID);
		p_ccs->error_code = CCS811_DRV_HW_ID;
		return false;
	}

#if defined(CCS811_DEBUG_LEVEL_2)
	// Read hardware version
	uint8_t hw_ver_data;
	if (!ccs811_reg_read(p_ccs, CCS811_REG_HW_VER, &hw_ver_data, 1))
	{
		return false;
	}

	// Read firmware bootloader version
	uint8_t fw_boot_buffer[2];
	uint16_t fw_boot_data;
	if (!ccs811_reg_read(p_ccs, CCS811_REG_FW_BOOT_VER, fw_boot_buffer, 2))
	{
		return false;
	}
	fw_boot_data = (uint16_t)((fw_boot_buffer[0] << 8) | fw_boot_buffer[1]);

	// Read firmware app version
	uint8_t fw_app_buffer[2];
	uint16_t fw_app_data;
	if (!ccs811_reg_read(p_ccs, CCS811_REG_FW_APP_VER, fw_app_buffer, 2))
	{
		return false;
	}
	fw_app_data = (uint16_t)((fw_app_buffer[0] << 8) | fw_app_buffer[1]);

	DEBUG_DEV("Hardware version:      0x%02X", __func__, p_ccs, hw_ver_data);
	DEBUG_DEV("Firmware boot version: 0x%04X", __func__, p_ccs, fw_boot_data);
	DEBUG_DEV("Firmware app version:  0x%02X", __func__, p_ccs, fw_app_data);
#endif

	return ccs811_check_error_status(p_ccs);
}

static int
ccs811_printf(ccs811_t *p_ccs, const char *p_format, ...)
{
	#define BUF_SIZE 255	// Max printed string length

	char str_buffer[BUF_SIZE];
	va_list args;
	int result = 0;

	va_start(args, p_format);
	int print_result = vsnprintf(str_buffer, BUF_SIZE, p_format, args);
	// vsnprintf creates null terminated string in str_buffer
	va_end(args);

	if (print_result > 0)
	{
		// Call platform printf
		result = (*p_ccs->platform_cb)(p_ccs, CCS811_MSG_PRINTF, 0, str_buffer);
	}
	return result;
}

static int
ccs811_delay(ccs811_t* p_ccs, uint8_t delay_msg, size_t delay_len) 
{
	return (*p_ccs->platform_cb)(p_ccs, delay_msg, delay_len, NULL);
}

static int
ccs811_i2c_write_bytes(ccs811_t *p_ccs, uint8_t *p_buffer, size_t buf_len) 
{
	return (*p_ccs->platform_cb)(p_ccs, CCS811_MSG_I2C_WRITE_BYTES, buf_len,
		p_buffer);
}

static int
ccs811_i2c_read_bytes(ccs811_t* p_ccs, uint8_t* p_buffer, size_t buf_len)
{
	return (*p_ccs->platform_cb)(p_ccs, CCS811_MSG_I2C_READ_BYTES, buf_len,
		p_buffer);
}

/* [] END OF FILE */
