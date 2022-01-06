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
    uint8_t alarmH[20] = {0,0,1};
    uint8_t alarmM[20] = {2,3,1};
    char*  pillNames[20] = {"Pill X", "Drug Y" , "Tablet Z"};
    uint64_t pillCodes[20] = {0x880404D850,0x8804D5BEE7,0x880495829B};
    //0x88 0x04 0x04 0xD8 0x50
    
    uint8_t alarmNum = 0;
    checkTag(0x880404D850);
    for (int i = 0 ; i< 100; i++){
    //checkTag(0x880404D850);
    if(timeChange())
    {
        alarmNum = checkAlarm(alarmH,alarmM);
        readTag();
        if(!alarmState)
        {
            warpPrint(" 0x%02x 0x%02x,", hours, mins);
            showTime();
        }
        else
        {
            warpPrint(pillNames[alarmNum]);
            showTime();
            writeString(" Take");
            setLine(2);
            writeString(pillNames[alarmNum]);
            for (int j =0; j<20;j++)
            {
            bottomRECT(0x00,0x00,0x00);
            OSA_TimeDelay(500);
            bottomRECT(0xff,0xff,0xff);
            OSA_TimeDelay(500);
            if(false){
                j = 20;
                
            }
            checkTag(pillCodes[0]);
            }

        }

    }
    
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

uint8_t checkAlarm(uint8_t *alarmH, uint8_t *alarmM)
{
    for(int i = 0; i<20;i++)
    {
        if((alarmH[i] == hours)&&(alarmM[i] == mins))
        {
            alarmState = true;
            return i;
        }
    }
    return 0;
}
void
readTag()
{
warpPrint("reading");
uint8_t data[5];
    if(request_tag(0x26, data) == 0)
    { //checks for a tag
	    if(mfrc522_get_card_serial(data) == 0)
        {
		    lastReadTag = data[0];
            lastReadTag <<= 8;
            lastReadTag += data[1];
            lastReadTag <<= 8;
            lastReadTag += data[2];
            lastReadTag <<= 8;
            lastReadTag += data[3];
            lastReadTag <<= 8;
            lastReadTag += data[4];

        }
	    else
        {
		    warpPrint("No card present");
	    }   
    }
}

bool
checkTag(uint64_t savedData)
{
    readTag();
    if (lastReadTag == savedData)
    {
        warpPrint("Success!");
        return true;
    }
    else
    {
        return false;
    }
}
