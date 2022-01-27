# 4B25 Project: Medication tracker using RFID. Nikhil Antony

naa49_coursework4_report.pdf has the main report for this project aims and performance characterisation. This read.me is to explain more about how the embedded systems works using the warp firmware. wiring diagram of this system is shown as Figure 1. The system is composed of: KL03 development baord, SSD1331 OLED Display, DS1307 Real Time Clock and RC522 RFID reader. The INA219 and Arduino Nano are used to monitor power consumption.

## SSD1331 OLED

This is the same OLED display used in the previous coursework items and the original warp firmware has a driver file for the SSD1331. This file was adapted to display text for the project. The OLED display is used to tell the time, like a watch and notify the user if it is time to take medication. 
The files SSD1331.h and SSD1331.c are the driver files for the OLED. An existing driver that used the SSD1331 to display text was found at https://os.mbed.com/users/star297/code/ssd1331//file/4385fd242db0/ssd1331.cpp/ . This code was edited to make sure it could run on the KL03 and any functions that were not useful were removed. The functions writeChar and pixel are from the online driver file and are used to write individual characters using an array containing the pixels needed for each character. As the a metric of evaluation was power, there have been various additions to the driver to reduce power consumptions. In the high power branch every pixel placed is white whilst in the low power branch every pixel is green (see report for difference in consumption). Also there is an additional function called set birghtness which can be used to set the master brightness from 0-16. This is used to have a 'dim mode' when the device is only displaying time.

## MFRC522 RFiD Reader

This is a common low cost RFiD reader chip. It is used in this system to read the code on a RFID tag. The device operates at 13.56MHz. A new driver file was written for this, MFRC522.c and MFRC522.h . Much of the driver is based on code from https://github.com/ljos/MFRC522/blob/master/MFRC522.cpp . This driver can be used to check if there is a tag near and also read the code from the tag if it is near. This device also had modifications used to reduce power. Once the device is switched on to read, the consumption is very high (~24mA), so the first change was to reset the device after every 'getCode' operation. This did end up saving power however the device had to be reinitialised each time after reset. The next idea was to use the low power mode setting on the device itself. This can be done by writting one of the command registers. This ended saving up even more power. 

## DS1307 Real Time Clock

The real time clock was used so that the device would still tell the correct time every if it was powered off for a period of time. This is beacuse the DS1307 has a backup battery. Whilst the KL03 does have it's own RTC it was simpler to use an external RTC (though it does take up more space). The files for the driver are DS1307.c and DS107.h The RTC has registers for each of the time divisions like seconds, minutes, hours etc. Time can be set using the debug menu (an additional option was added to set time). The time can be retreived from the RTC using the command getTime.

## mainProg

This is the main c file to run the device. It continually checks the time and updates the screen if needed. The file has preset medication alarm times and names. If the time is the same as one of the alarm times the device enters 'Alarm State' (as opposed to 'time only state'). This consume more power as the screen becomes brighter and the RFID scanner checks for tags twice every second. The Alarm State is escaped either when there is a timeout or when the correct tag is presented (each medication has a preset tag in the code as well). When Alarm State is exited from the system goes back into time only and runs until the next alarm. This is set in an infinite loop.

## boot.c

boot.c is the main code that is run when the device starts. All the sensors are initialised in boot.c . Boot.c also contains the warp debug menu. As the medication alarm system runs in a infinite loop and the debug runs in an infitinite loop, both cannot run at the same time. To ensure the device performs like a medication alarm normally on startup by still can have a accessible debug menu a work around was made. There is a 'debug tag' code, and a corresponding tag. If there is no debug tag presented when the device turns on, mainProg is run. If there is a tag that matches debug Tag code, then the mainProg code is bypassed and the warp debug menu is presented.

The warp debug menu was edited so if the user typed '?' they could set the RTC time. 
