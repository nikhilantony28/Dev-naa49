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
#include "mainProg.h"




extern volatile WarpI2CDeviceState	deviceDS1307State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;


/*
    Main code to run program. Program has infinite loop.
*/


void mainProgram()
{
    //setTimeDS1307(0x50,0x39,0x09); // for demo delete in real code
    showTime();
    uint8_t alarmH[10] = {9,10,10,11,12,13,14,15,16,10}; // medication alarm times : hours
    uint8_t alarmM[10] = {58,0,2,3,4,5,6,7,8,9};// medication alarm times : minutes
    char*  pillNames[10] = {"Pill1","Pill2","Pill3","Pill4","Pill5","Pill6","Pill7","Pill8","Pill9","Pill10"};// medication alarm names
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
    };// codes of the nfc tags
    int alarmNum = 0;
     //low power additions remove comment if low power mode is set
        MFRC522SoftPowerDown;
        devSSD1331SetBrightness(0x01);
    

    while(1){
    if(timeChange()) //below code is excutecuted everytime the minute changes
    {
        alarmNum = checkAlarm(alarmH,alarmM);
        if(!alarmState)
        {
            // updates display to show new time

            warpPrint(" 0x%02x 0x%02x,", hours, mins);
            showTime();
        }
        else
        {
            devSSD1331SetBrightness(0x0F); //low power addition
            //warpPrint(pillNames[alarmNum]); debug print statement

            //updates display to tell user to take the correct medication
            showTime();
            devSSD1331WriteString(" Take");
            devSSD1331ClearLine(2);
            devSSD1331SetLine(2);
            devSSD1331WriteString(pillNames[alarmNum]);


            for (int j =0; j<200;j++)
            {
                //flashing bottom reactangle

                devSSD1331BottomRect(0x00,0xA0,0x00);
                OSA_TimeDelay(200);
                devSSD1331BottomRect(0x00,0x00,0x00);
                OSA_TimeDelay(200);

                if(checkTag(pillCodes[alarmNum]))//checks to see if tag matches alarm's preset tag code
                {
                    j = 200;  //exit loop
                    
                }
            }
            alarmState = false;


            devSSD1331SetBrightness(0x01);      //low power addition

            //reset
            devSSD1331Clear();
            lastReadTag = 0;
            showTime();

        }

    }
    
    OSA_TimeDelay(2000);
    }
    
}

/*
    updateTime checks the DS1307 minutes and hours registers and sets the variables mins and hours to the value in the registers
*/

void updateTime()
{
    mins = outputTimeDS1307(0x01);
    hours = outputTimeDS1307(0x02);
}
/*
    timeChanges returns true if time now is different to what was previously saved
*/

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

/*
    showTime writes the time to the OLED display using the devSSD1331WriteTime function in the driver
*/

void showTime()
{
    updateTime();
    devSSD1331WriteTime(hours,mins);
}

/*
    checkAlarm looks if the time matches any of the preset alarm times
*/

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

/*
    readTag trys to read a tag code if there is one present
*/

void
readTag()
{

//warpPrint("reading"); //debug print statement

//MFRC522SoftPowerUp(); //low power addition

uint8_t data[5];
    if(requestTag(0x26, data) == 0)//checks for a tag
    { 
	    if(getCode(data) == 0)//checks to see if tagRead was okay. Output is saved as data.
        {
            //converts data into a single uint64 number


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
//MFRC522SoftPowerDown(); // low power addition
}

/*
    checkTag compares the last read tag to the parsed one.
*/

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