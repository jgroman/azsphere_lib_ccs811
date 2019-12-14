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
 * @file    lib_ccs811.c
 * @version 1.0.0
 *
 * @brief Definitions for CCS811 platform dependent wrapper.
 *
 * @author   Jaroslav Groman
 *
 * @date
 *
 *******************************************************************************/

#include <time.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <applibs/log.h>
#define I2C_STRUCTS_VERSION 1
#include <applibs/i2c.h>
#include <applibs/gpio.h>

#include <lib_ccs811.h>


 /*******************************************************************************
 * Forward declarations of private functions
 *******************************************************************************/

 /** @brief Platform dependent functions callback implementation.
  *
  * This implementation supports Avnet Azure Sphere MT3620 Starter Kit platform.
  *
  * @param p_ccs		    Pointer to CCS811 data descriptor.
  * @param msg           Function type message (see CCS811_MSG_x).
  * @param arg_int       General integer argument.
  * @param p_arg_data    General data pointer argument.
  *
  * @return	Returned value depends on concrete function called.
  */
static int
ccs811_platform_cb(ccs811_t *p_ccs, uint8_t msg, size_t arg_int,
    void *p_arg_data);

/*******************************************************************************
* Global variables
*******************************************************************************/

static     int i2c_fd = -1;		// Global I2C file descriptor
static     int cs_gpio_fd = -1;		// Global CS GPIO file descriptor
static GPIO_Id cs_gpio_id = -1;	    // Global CS GPIO Id

/*******************************************************************************
* Function definitions
*******************************************************************************/

ccs811_t
*ccs811_open(int i2c_fd_arg, I2C_DeviceAddress i2c_addr, GPIO_Id cs_gpio)
{
    bool b_progress_is_ok = true;

    // Activate sensor /WAKE signal
    if (cs_gpio > 0)
    {
        Log_Debug("Open CS GPIO %d\n", cs_gpio);
        b_progress_is_ok = false;

        // Configure CS GPIO as output with Low state, Push-pull mode
        cs_gpio_id = cs_gpio;
        cs_gpio_fd = GPIO_OpenAsOutput(cs_gpio, GPIO_OutputMode_PushPull,
            GPIO_Value_Low);
        if (cs_gpio_fd != -1)
        {
            // /WAKE pin is activated OK
            b_progress_is_ok = true;
        }
        else
        {
            Log_Debug("ERROR: %s: errno=%d (%s)\n", __FUNCTION__, errno,
                strerror(errno));
        }
    }

    ccs811_t *p_result = NULL;

    if (b_progress_is_ok)
    {
        i2c_fd = i2c_fd_arg;

        // Initialize sensor
        p_result = ccs811_init((uint8_t)i2c_addr, ccs811_platform_cb);
    }

    return p_result;
}

void
ccs811_hw_reset(GPIO_Id rst_gpio)
{
    if (rst_gpio > 0)
    {
        Log_Debug("Open RST GPIO %d\n", rst_gpio);
        // Configure RST GPIO as output with Low state, Push-pull mode
        int gpio_rst_fd;
        gpio_rst_fd = GPIO_OpenAsOutput(rst_gpio, GPIO_OutputMode_PushPull,
            GPIO_Value_Low);

        if (gpio_rst_fd == -1)
        {
            Log_Debug("ERROR: GPIO_OpenAsOutput: errno=%d (%s)\n", errno,
                strerror(errno));
        }
        else
        {
            // Wait for tDRESET time
            struct timespec sleepTime;
            sleepTime.tv_sec = 0;
            sleepTime.tv_nsec = CCS811_TIME_DRESET_US * 1000;
            nanosleep(&sleepTime, NULL);

            close(gpio_rst_fd);

            // Configure RST GPIO as input
            gpio_rst_fd = GPIO_OpenAsInput(rst_gpio);
            if (gpio_rst_fd == -1)
            {
                Log_Debug("ERROR: GPIO_OpenAsInput: errno=%d (%s)\n", errno,
                    strerror(errno));
            }
            else
            {
                close(gpio_rst_fd);
                Log_Debug("Close RST GPIO %d\n", rst_gpio);

                // Wait for tSTART time after nRESET
                sleepTime.tv_sec = 0;
                sleepTime.tv_nsec = CCS811_TIME_START_RESET_US * 1000;
                nanosleep(&sleepTime, NULL);

                // Reset pulse activated OK
            }
        }
    }
    return;
}

void
ccs811_close(ccs811_t *p_ccs) {

    // Free resources
    ccs811_shutdown(p_ccs);

    // Deactivate sensor /WAKE signal
    if (cs_gpio_id != -1)
    {
        close(cs_gpio_fd);

        // Configure CS GPIO as input
        cs_gpio_fd = GPIO_OpenAsInput(cs_gpio_id);
        if (cs_gpio_fd == -1) {
            Log_Debug("ERROR: GPIO_OpenAsInput: errno=%d (%s)\n", errno,
                strerror(errno));
            return;
        }
        Log_Debug("Close CS GPIO %d\n", cs_gpio_id);
        close(cs_gpio_fd);
    }
}

static int
ccs811_platform_cb(ccs811_t *p_ccs, uint8_t msg, size_t arg_int,
    void *p_arg_data)
{
    ssize_t result = -1;
    struct timespec sleepTime;

    switch (msg)
    {

    case CCS811_MSG_I2C_READ_BYTES:
        // Read arg_int bytes from I2C address and store it to p_arg_data
        result = I2CMaster_Read(i2c_fd, p_ccs->i2c_addr, p_arg_data,
            arg_int);
        if (result == -1) {
            Log_Debug("ERROR: I2CMaster_Read: errno=%d (%s)\n", errno,
                strerror(errno));
        }
        break;

    case CCS811_MSG_I2C_WRITE_BYTES:
        // Delay required by Azure Sphere OS 19.11
        sleepTime.tv_sec = 0;
        sleepTime.tv_nsec = 800000;
        nanosleep(&sleepTime, NULL);

        // Write arg_int bytes from p_arg_data to I2C address
        result = I2CMaster_Write(i2c_fd, p_ccs->i2c_addr, p_arg_data,
            arg_int);
        if (result == -1)
        {
            Log_Debug("LIB CCS811 ERROR: I2CMaster_Write: errno=%d (%s)\n", errno,
                strerror(errno));
        }
        break;

    case CCS811_MSG_DELAY_MS:
        // Perform delay for arg_int milliseconds
        sleepTime.tv_sec = 0;
        sleepTime.tv_nsec = 1000000 * (long)arg_int;
        result = nanosleep(&sleepTime, NULL);
        break;

    case CCS811_MSG_DELAY_US:
        // Perform delay for arg_int mircoseconds
        sleepTime.tv_sec = 0;
        sleepTime.tv_nsec = 1000 * (long)arg_int;
        result = nanosleep(&sleepTime, NULL);
        break;

    case CCS811_MSG_PRINTF:
        // Output string from p_arg_data
        result = Log_Debug("%s", p_arg_data);
        break;

    default:
        break;
    }

    return result;
}

/* [] END OF FILE */
