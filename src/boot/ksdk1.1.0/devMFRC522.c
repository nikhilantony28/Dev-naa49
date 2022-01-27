/*
	This driver is based on code from:

	Physical Computation Laboratory, Cambridge (https://github.com/physical-computation/Warp-firmware)
	Miguel Balboa (https://github.com/miguelbalboa/rfid)
	Asif Mahmud Shimon (https://github.com/asif-mahmud/MIFARE-RFID-with-AVR)

 */




#include <stdint.h>

#include "config.h"

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devMFRC522.h"



volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
 enum
 {
 	kMFRC522PinMOSI	= GPIO_MAKE_PIN(HW_GPIOA, 8), //GREEN WIRE
 	kMFRC522PinMISO	= GPIO_MAKE_PIN(HW_GPIOA, 6), //YELLOW WIRE
 	kMFRC522PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9), //BLUE WIRE
 	kMFRC522PinCSn		= GPIO_MAKE_PIN(HW_GPIOA, 5), //WHITE WIRE
 	kMFRC522PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12), // RED WIREss
 	kMFRC522PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 2), //ORANGE WIRE
 };



WarpStatus
writeSensorRegisterMFRC522(uint8_t addr, uint8_t payload)
{
    spi_status_t status;

	payloadBytes[0] = ((addr<<1)&0x7E);
	payloadBytes[1] = payload;


	GPIO_DRV_SetPinOutput(kMFRC522PinCSn);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kMFRC522PinCSn);


	status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)payloadBytes,
					(uint8_t * restrict)inBuffer,
					2 /* transfer size */,
					2000);

	GPIO_DRV_SetPinOutput(kMFRC522PinCSn);

}

uint8_t
readSensorRegisterMFRC522(uint8_t addr)
{
	spi_status_t status;

	payloadBytes[0] = (((addr<<1)&0x7E) | 0x80);
	payloadBytes[1] = 0x00;


	GPIO_DRV_SetPinOutput(kMFRC522PinCSn);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kMFRC522PinCSn);


	status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)payloadBytes,
					(uint8_t * restrict)inBuffer,
					2 /* transfer size */,
					2000);

	GPIO_DRV_SetPinOutput(kMFRC522PinCSn);

	return inBuffer[1];
}

/*
	setBitMask is a function from  https://github.com/miguelbalboa/rfid/blob/master/src/MFRC522.cpp
*/

void
setBitMask(uint8_t addr, uint8_t mask)
{
	uint8_t current = readSensorRegisterMFRC522(addr);
	writeSensorRegisterMFRC522(addr, current | mask);
}

/*
	clearBitMask is a function from  https://github.com/miguelbalboa/rfid/blob/master/src/MFRC522.cpp 
*/

void
clearBitMask(uint8_t addr, uint8_t mask)
{
	uint8_t current;
	current = readSensorRegisterMFRC522(addr);
	writeSensorRegisterMFRC522(addr, current & (~mask));
}

/*
	commandTag is a function from  https://github.com/miguelbalboa/rfid/blob/master/src/MFRC522.cpp
*/

uint8_t commandTag(uint8_t cmd, uint8_t *data, int dlen, uint8_t *result, int *rlen)
{
	int status = MI_ERR;
	uint8_t irqEn = 0x00;
  	uint8_t waitIRq = 0x00;
  	uint8_t lastBits, n;
    int i;

  	switch (cmd) 
	{
		case MFRC522_AUTHENT:
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		case MFRC522_TRANSCEIVE:
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		default:
			break;
  	}

	writeSensorRegisterMFRC522(CommIEnReg, irqEn|0x80);    	// interrupt request
  	clearBitMask(CommIrqReg, 0x80);             			// Clear all interrupt requests bits.
  	setBitMask(FIFOLevelReg, 0x80);            				// FlushBuffer=1, FIFO initialization

	writeSensorRegisterMFRC522(CommandReg, MFRC522_IDLE);   // No action, cancel the current command.

	// Write to FIFO
	for (i=0; i < dlen; i++) 
	{
		writeSensorRegisterMFRC522(FIFODataReg, data[i]);
	}

	//Execute the command.
	writeSensorRegisterMFRC522(CommandReg, cmd);
	if (cmd == MFRC522_TRANSCEIVE) 
	{
		setBitMask(BitFramingReg, 0x80);  // StartSend=1, transmission of data starts
	}

	//Waiting for the command to complete so we can receive data.
	i = 25; // Max wait time is 25ms.
	do 
	{
		OSA_TimeDelay(1);
		n = readSensorRegisterMFRC522(CommIrqReg);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	clearBitMask(BitFramingReg, 0x80);  // StartSend=0 

	if (i != 0) 
	{ // Request did not time out.
		if(!(readSensorRegisterMFRC522(ErrorReg) & 0x1B)) 
		{  // BufferOvfl Collerr CRCErr ProtocolErr
			status = MI_OK;
			if (n & irqEn & 0x01) 
			{
				status = MI_NOTAGERR;
			}

			if (cmd == MFRC522_TRANSCEIVE) 
			{
				n = readSensorRegisterMFRC522(FIFOLevelReg);
				lastBits = readSensorRegisterMFRC522(ControlReg) & 0x07;
				if (lastBits) 
				{
					*rlen = (n-1)*8 + lastBits;
				} else 
				{
					*rlen = n*8;
				}

				if (n == 0) 
				{
					n = 1;
				}

				if (n > MAX_LEN) 
				{
					n = MAX_LEN;
				}

				// Read the recieved data from FIFO
				for (i=0; i<n; i++) 
				{
				result[i] = readSensorRegisterMFRC522(FIFODataReg);
				}
			}
			} else {
			status = MI_ERR;
		}
	}
	return status;
}

/*
	requestTag is a function from https://github.com/miguelbalboa/rfid/blob/master/src/MFRC522.cpp which checks to set if a tag is present
*/

uint8_t requestTag(uint8_t mode, uint8_t *data)
{
	int status, len;
	writeSensorRegisterMFRC522(BitFramingReg, 0x07); // TxLastBists = BitFramingReg[2..0]

	data[0] = mode;
	status = commandTag(MFRC522_TRANSCEIVE, data, 1, data, &len);

	if((status != MI_OK) || (len != 0x10))
	{
		//something has gone wrong
		status = MI_ERR;
	};

	return status;
}

/*
	getCode is based on https://github.com/asif-mahmud/MIFARE-RFID-with-AVR/tree/master/lib/avr-rfid-library/lib to read the code of a tag
	The input array serial_out is loaded with the tag's code
*/

uint8_t getCode(uint8_t *serial_out)
{
	int status, i, len;
	uint8_t check = 0x00;

	writeSensorRegisterMFRC522(BitFramingReg, 0x00);

	serial_out[0] = MF1_ANTICOLL;
	serial_out[1] = 0x20;
	status = commandTag(MFRC522_TRANSCEIVE, serial_out, 2, serial_out, &len);

	len = len/8;
	if(status == MI_OK){

		for(i=0; i<len-1; i++){
			check ^= serial_out[i];
		}
		if (check != serial_out[i]){
			status = MI_ERR;
		}
	}

	return status;

}


void
devMFRC522init()
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI, MISO and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
 	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);
 	PORT_HAL_SetMuxMode(PORTA_BASE, 6u, kPortMuxAlt3);

 	warpEnableSPIpins();

 	/*
 	 *	Override Warp firmware's use of these pins.
 	 *
 	 *	Reconfigure to use as GPIO.
 	 */
 	PORT_HAL_SetMuxMode(PORTA_BASE, 5u, kPortMuxAsGpio);
 	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
 	PORT_HAL_SetMuxMode(PORTB_BASE, 2u, kPortMuxAsGpio);

	// Set CS pin high
	GPIO_DRV_SetPinOutput(kMFRC522PinCSn);
	// Set RST pin high
	GPIO_DRV_SetPinOutput(kMFRC522PinRST);

	/* Initialising sensor*/

	writeSensorRegisterMFRC522(TModeReg, 0x8D);       
	writeSensorRegisterMFRC522(TPrescalerReg, 0x3E);
	writeSensorRegisterMFRC522(TReloadRegL, 30);
	writeSensorRegisterMFRC522(TReloadRegH, 0);

	writeSensorRegisterMFRC522(TxAutoReg, 0x40);      
	writeSensorRegisterMFRC522(ModeReg, 0x3D);

	setBitMask(TxControlReg, 0x03);        // Turn antenna on
	return;
}

/* 
	This functions is used to power off the device via the reset pins. 
	This function is not used in the final design as the soft power down 
	and up are much more effective.
*/

void
MFRC522PowerDown()
{
	GPIO_DRV_ClearPinOutput(kMFRC522PinRST);
}

/* 
	This functions is used to power up the device via the reset pins. 
	This function is not used in the final design as the soft power down 
	and up are much more effective. The device needs reinitialsing as it 
	has been reset.
*/

void
MFRC522PowerUp()
{
	// Set CS pin high
	GPIO_DRV_SetPinOutput(kMFRC522PinCSn);
	// Set RST pin high
	GPIO_DRV_SetPinOutput(kMFRC522PinRST);
	
	/* Initialising sensor*/


	writeSensorRegisterMFRC522(TModeReg, 0x8D);       // These 4 lines input the prescaler to the timer - see datasheet for why these values
	writeSensorRegisterMFRC522(TPrescalerReg, 0x3E);
	writeSensorRegisterMFRC522(TReloadRegL, 30);
	writeSensorRegisterMFRC522(TReloadRegH, 0);

	writeSensorRegisterMFRC522(TxAutoReg, 0x40);      /* 100%ASK */
	writeSensorRegisterMFRC522(ModeReg, 0x3D);

	setBitMask(TxControlReg, 0x03);        /* Turn antenna on */
	return;
}

/* 
	This functions is used to 'soft' power down the device via the command 
	register and power saving mode. 
*/

void
MFRC522SoftPowerDown()
{
	writeSensorRegisterMFRC522(CommandReg,0x10);
}

/* 
	This functions is used to 'soft' power up the device via the command 
	register and power saving mode. There is no need for reinitialsing
*/

void
MFRC522SoftPowerUp()
{
	writeSensorRegisterMFRC522(CommandReg,0x00);
}