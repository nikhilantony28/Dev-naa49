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


extern volatile WarpI2CDeviceState	deviceDS1307State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t		gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t		gWarpSupplySettlingDelayMilliseconds;



void
initDS1307(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceDS1307State.i2cAddress			= i2cAddress;
	deviceDS1307State.operatingVoltageMillivolts	= operatingVoltageMillivolts;

	return;
}

WarpStatus
writeSensorRegisterDS1307(uint8_t deviceRegister, uint8_t payload)
{
	uint8_t		payloadByte[1], commandByte[1];
	i2c_status_t	status;
	// Addresses for registers you can write to
	/*
	switch (deviceRegister)
	{
		case 0x00: case 0x05:
		{
			
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}
	*/

	i2c_device_t slave =
	{
		.address = deviceDS1307State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceDS1307State.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	payloadByte[0] = payload; // Right shift by 8 to get top half
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
							0 /* I2C instance */,
							&slave,
							commandByte,
							1,
							payloadByte,
							1,
							gWarpI2cTimeoutMilliseconds);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

uint8_t
convertHexaSexa(uint8_t num)
{
	
	uint8_t upperNibble;
	upperNibble = num/10;
	upperNibble <<= 4;
	uint8_t out = num%10 + upperNibble;
	return out;
}


uint8_t
convertSexaHexa(uint8_t num)
{
	uint8_t tens;
	uint8_t units;
	uint8_t upperNibble;
	uint8_t lowerNibble;
	units = num%16;
	tens = num >> 4; 
	uint8_t out = (units + tens*10);
	return out;
}

WarpStatus
setTimeDS1307(uint8_t seconds, uint8_t minutes, uint8_t hours)
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2, i2cWriteStatus3;
	uint8_t newSeconds;
	uint8_t newMinutes;
	uint8_t newHours;
	newSeconds = convertHexaSexa(seconds);
	newMinutes = convertHexaSexa(minutes);
	newHours = convertHexaSexa(hours);

	warpScaleSupplyVoltage(deviceDS1307State.operatingVoltageMillivolts);
	i2cWriteStatus1 = writeSensorRegisterDS1307(0x00 /*set seconds value  */,		newSeconds);

	i2cWriteStatus2 = writeSensorRegisterDS1307(0x01 /*set minutes value  */,		newMinutes);

	i2cWriteStatus3 = writeSensorRegisterDS1307(0x02 /*set hours value  */,			newHours);

	return (i2cWriteStatus1 | i2cWriteStatus2 | i2cWriteStatus3);
}



WarpStatus
readSensorRegisterDS1307(uint8_t deviceRegister,  int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	// Register addresses you can read
	/*
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03:
		case 0x04: case 0x05:
		{
			
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}
	*/

	i2c_device_t slave =
	{
		.address = deviceDS1307State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceDS1307State.operatingVoltageMillivolts);
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
							0 /* I2C peripheral instance */,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceDS1307State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

uint8_t
outputTimeDS1307(uint8_t reg)
{

	WarpStatus i2cReadStatus;

	i2cReadStatus = readSensorRegisterDS1307(reg /* Voltage*/ , 1 /* numberOfBytes */);
	
	
	
	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
		return 0;
	}
	else{
		warpPrint("%x",deviceDS1307State.i2cBuffer[0]);
		return convertSexaHexa(deviceDS1307State.i2cBuffer[0]);
	}
}

WarpStatus
configureSensorDS1307()
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;


	warpScaleSupplyVoltage(deviceDS1307State.operatingVoltageMillivolts);
	i2cWriteStatus1 = writeSensorRegisterDS1307(0x00 /* register address Config Reg */,							0x00 /* payload default: 0x399F*/);

	i2cWriteStatus2 = writeSensorRegisterDS1307(0x01 /* register address Calibration */,						0x00	 /* payload default: 0x0000 but need to calculate what it should be*/);

	writeSensorRegisterDS1307(0x02 /* register address Calibration */,						0x00	 /* payload default: 0x0000 but need to calculate what it should be*/);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}
