#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devSSD1331.h"
#include "devDS1307.h"
#include "devMFRC522.h"
#include "main_prog.h"




extern volatile WarpI2CDeviceState	deviceDS1307State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void main_printTime()
{
    configureSensorDS1307();
    for (int i = 0 ; i< 100; i++){
    if(timeChange)
    {
        showTime();
    }
    warpPrint(" 0x%02x 0x%02x,", hours, mins);
    OSA_TimeDelay(2000);
    }
    
}
void updateTime()
{
    mins = outputTimeDS1307(0x01);
    hours = outputTimeDS1307(0x02);
}

bool timeChange()
{
    uint8_t minsOld;

    minsOld = mins;
    updateTime();
    if (minsOld == mins)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void showTime()
{
    writeTime(hours,mins);
}