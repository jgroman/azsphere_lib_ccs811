#include <stdbool.h>
#include <errno.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/i2c.h>

// Import project hardware abstraction from project property 
// "Target Hardware Definition Directory"
#include <hw/project_hardware.h>

#include "lib_ccs811.h"

// Support functions.
static void TerminationHandler(int signalNumber);
static int InitPeripheralsAndHandlers(void);
static void ClosePeripheralsAndHandlers(void);

// File descriptors - initialized to invalid value
static int i2cFd = -1;

// Termination state
static volatile sig_atomic_t terminationRequired = false;

/// <summary>
///     Signal handler for termination requests. This handler must be 
///     async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

/// <summary>
///     Set up SIGTERM termination handler, initialize peripherals and 
///     set up event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    // Init I2C
    i2cFd = I2CMaster_Open(PROJECT_ISU2_I2C);
    if (i2cFd < 0) {
        Log_Debug("ERROR: I2CMaster_Open: errno=%d (%s)\n", errno,
            strerror(errno));
        return -1;
    }

    int result = I2CMaster_SetBusSpeed(i2cFd, I2C_BUS_SPEED_STANDARD);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetBusSpeed: errno=%d (%s)\n", errno,
            strerror(errno));
        return -1;
    }

    result = I2CMaster_SetTimeout(i2cFd, 100);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetTimeout: errno=%d (%s)\n", errno,
            strerror(errno));
        return -1;
    }

    result = I2CMaster_SetDefaultTargetAddress(i2cFd, 0x3C);
    if (result != 0) {
        Log_Debug("ERROR: I2CMaster_SetDefaultTargetAddress: errno=%d (%s)\n",
            errno, strerror(errno));
        return -1;
    }

    return 0;
}

/// <summary>
///     Close peripherals and handlers.
/// </summary>
static void ClosePeripheralsAndHandlers(void) {
    close(i2cFd);
}

/// <summary>
///     CCS811 demo application
/// </summary>
void ccs811Main(void) {

    ccs811_t *p_ccs;
    Log_Debug("Open CCS\n");
    p_ccs = ccs811_open(i2cFd, CCS811_I2C_ADDRESS_1, SK_SOCKET1_CS_GPIO);

    struct timespec sleepTime;
    sleepTime.tv_sec = 1;
    sleepTime.tv_nsec = 0;

    uint16_t tvoc;
    uint16_t eco2;

    ccs811_set_mode(p_ccs, CCS811_MODE_1S);
    nanosleep(&sleepTime, NULL);

    for (int meas = 0; meas < 300; meas++)
    {
        if (ccs811_get_results(p_ccs, &tvoc, &eco2, 0, 0)) {
            Log_Debug("CCS811 Sensor periodic: TVOC %d ppb, eCO2 %d ppm\n",
                tvoc, eco2);
        }
        else
        {
            Log_Debug("No results\n");
        }

        nanosleep(&sleepTime, NULL);
    }

    Log_Debug("Close CCS\n");
    ccs811_close(p_ccs);
}

/// <summary>
///     Application main entry point
/// </summary>
int main(void)
{
    Log_Debug("\n*** Starting ***\n");

    if (InitPeripheralsAndHandlers() != 0)
    {
        terminationRequired = true;
    }

    if (!terminationRequired) {
        ccs811Main();
    }

    Log_Debug("*** Terminating ***\n");
    ClosePeripheralsAndHandlers();
    return 0;
}
