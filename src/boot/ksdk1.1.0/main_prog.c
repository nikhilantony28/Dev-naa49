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
    setTimeDS1307(0x00,0x38,0x09);
    showTime();
    uint8_t alarmH[10] = {9,10,10,11,12,13,14,15,16,10};
    uint8_t alarmM[10] = {58,0,2,3,4,5,6,7,8,9};
    char*  pillNames[10] = {"pill1","pill2","pill3","pill4","pill5","pill6","pill7","pill8","pill9","pill10"};
    uint64_t pillCodes[10] = {
    0x880404D850,
    0x880495829b,
    0x8804aab395,
    0x8804d5bee7,
    0x880422ba14,
    0x8804408b47,
    0x8804938a95,
    0x880454b66e,
    0x88040440c8,
    0x8804b7162d
    };
    int alarmNum = 0;
    setBrightness(0x01);
    while(1){
        MFRC522PowerDown();
        //MFRC522SoftPowerDown();
    if(timeChange())
    {
        alarmNum = checkAlarm(alarmH,alarmM);
        if(!alarmState)
        {
            warpPrint(" 0x%02x 0x%02x,", hours, mins);
            showTime();
        }
        else
        {
            devSSD1331init();
            setBrightness(0x0F);
            warpPrint(pillNames[alarmNum]);
            showTime();
            writeString(" Take");
            clearLine(2);
            setLine(2);
            writeString(pillNames[alarmNum]);
            for (int j =0; j<100;j++)
            {
            bottomRECT(0x00,0x00,0x00);
            OSA_TimeDelay(400);
            bottomRECT(0x00,0x90,0x00);
            OSA_TimeDelay(200);
            MFRC522PowerUp();
            if(checkTag(pillCodes[alarmNum]))
            {
                j = 100;  
                
            }
            MFRC522PowerDown();

            }
            alarmState = false;
            setBrightness(0x01);
            clearScreen(0x00,0x00,0x5F,0x3F);
            lastReadTag = 0;
            showTime();

        }

    }
    
    OSA_TimeDelay(2000);
    }
    
}
void updateTime()
{
    mins = outputTimeDS1307(0x01);
    hours = outputTimeDS1307(0x02);
    /*
    if(mins > 59)
    {
        setTimeDS1307(0x00,0x00,(hours+1)%24);
    }
    */
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
    updateTime();
    writeTime(hours,mins);
}

int checkAlarm(uint8_t *alarmH, uint8_t *alarmM)
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
