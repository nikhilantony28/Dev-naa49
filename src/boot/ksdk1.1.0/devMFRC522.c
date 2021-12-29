
/**************************************************************************/
/*!
Adaptation of MFRC522 RFID sensor to the Warp firmware using https://github.com/ljos/MFRC522/blob/master/MFRC522.cpp and
https://github.com/asif-mahmud/MIFARE-RFID-with-AVR/tree/master/lib/avr-rfid-library/lib for 4B25 Coursework - rlj36
*/
/**************************************************************************/


#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"
#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devMFRC522.h"


volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];
extern volatile WarpSPIDeviceState	deviceMFRC522State;
extern volatile uint32_t		gWarpSpiTimeoutMicroseconds;
extern volatile uint32_t		gWarpSPIBaudRateKbps;
#define _BV(bit) (1<<(bit))


/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
 enum
 {
 	kMFRC522PinMOSI	= GPIO_MAKE_PIN(HW_GPIOA, 8),
 	kMFRC522PinMISO	= GPIO_MAKE_PIN(HW_GPIOA, 6),
 	kMFRC522PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
 	kMFRC522PinCSn		= GPIO_MAKE_PIN(HW_GPIOA, 5),
 	kMFRC522PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
 	kMFRC522PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
 };

WarpStatus
writeSensorRegisterMFRC522(uint8_t deviceRegister, uint8_t writeValue)
{
	deviceMFRC522State.spiSourceBuffer[0] = deviceRegister;
	deviceMFRC522State.spiSourceBuffer[1] = writeValue;

	deviceMFRC522State.spiSinkBuffer[0] = 0x00;
	deviceMFRC522State.spiSinkBuffer[1] = 0x00;

	GPIO_DRV_SetPinOutput(kMFRC522PinCSn);
	OSA_TimeDelay(50);
	GPIO_DRV_ClearPinOutput(kMFRC522PinCSn);


	deviceMFRC522State.ksdk_spi_status = SPI_DRV_MasterTransferBlocking(0 /* master instance */,
					NULL /* spi_master_user_config_t */,
					(const uint8_t * restrict)deviceMFRC522State.spiSourceBuffer,
					(uint8_t * restrict)deviceMFRC522State.spiSinkBuffer,
					2 /* transfer size */,
					gWarpSpiTimeoutMicroseconds);

	GPIO_DRV_SetPinOutput(kMFRC522PinCSn);

	return kWarpStatusOK;
}


void
write_RFID(uint8_t addr, uint8_t payload)
{
    /*
    MFRC uses addresses such that it is 1XXXXXX0 for a read command,
    where XXXXXX is the 6 bit actual address. The MSB is always 1 for a read
    command and 0 for a write
    */
	writeSensorRegisterMFRC522(((addr<<1)&0x7E), payload);
}


WarpStatus
readSensorRegisterMFRC522(uint8_t deviceRegister)
{
	return writeSensorRegisterMFRC522(deviceRegister, 0x00);
}


uint8_t
read_RFID(uint8_t addr)
{
	/*
  MFRC uses addresses such that it is 1XXXXXX0 for a read command,
  where XXXXXX is the 6 bit actual address. The MSB is always 1 for a read
  command and 0 for a write
  */
	readSensorRegisterMFRC522(((addr<<1)&0x7E) | 0x80);

	return deviceMFRC522State.spiSinkBuffer[1];
}

//
void
clearBitMask(uint8_t addr, uint8_t mask)
{
	uint8_t current;
	current = read_RFID(addr);
	write_RFID(addr, current & (~mask));
}


void
setBitMask(uint8_t addr, uint8_t mask)
{
	uint8_t current = read_RFID(addr);
	write_RFID(addr, current | mask);
}



/* 0x91 = software version 1.0, 0x92 = software version 2.0
   my tags return 0x92 i.e. version 2.0 */
uint8_t
getFirmwareVersion(void)
{
	volatile uint8_t response;
	response = read_RFID(VersionReg);
	return response;

}


/*
* Send a command to the tag. CMD = command, data = sent data, dlen = data length, result = received data, rlen = result length (i.e. received data length)
Returns: MI_ERR if an error occurs
         MI_NOTAGERR if no tag to send command to
         MI_ OK if everything worked
*/

uint8_t commandTag(uint8_t cmd, uint8_t *data, int dlen, uint8_t *result, int *rlen)
{
	int status = MI_ERR;
  	//uint8_t irqEn = 0x70;
	uint8_t irqEn = 0x77;
  	uint8_t waitIRq = 0x30;
  	uint8_t lastBits, n;
  	int i;


	write_RFID(CommIEnReg, irqEn|0x80);    /* IRQ sent to the tag */
	clearBitMask(CommIrqReg, 0x80);             /* Clear all interrupt requests bits. */
	setBitMask(FIFOLevelReg, 0x80);             /* FlushBuffer=1, FIFO initialization */

	write_RFID(CommandReg, MFRC522_IDLE);  // Cancel current command - no action

	/* Write data to FIFO */
	for (i=0; i < dlen; i++) {
		write_RFID(FIFODataReg, data[i]);
	}

	/* Execute the command. */
	write_RFID(CommandReg, cmd);
	if (cmd == MFRC522_TRANSCEIVE) {
		setBitMask(BitFramingReg, 0x80);  // StartSend=1, transmission of data starts
	}

	/* Waiting for the command to complete so we can receive data. */
	i = 25;
	do {
		OSA_TimeDelay(1); // added in additional 1ms time delay - need to wait for command to finish
		n = read_RFID(CommIrqReg);
		i--;
	} while ((i!=0) && !(n&0x01) && !(n&waitIRq));

	clearBitMask(BitFramingReg, 0x80);  /* StartSend=0 */

	if (i != 0) { /* Request receieved a reply from tag reader. */
		if(!(read_RFID(ErrorReg) & 0x1B)) {  /* No errors were generated in the command */
		status = MI_OK;
		if (n & irqEn & 0x01) {
			status = MI_NOTAGERR;
		}

		if (cmd == MFRC522_TRANSCEIVE) {
			n = read_RFID(FIFOLevelReg);
			lastBits = read_RFID(ControlReg) & 0x07;
			if (lastBits) {
				*rlen = (n-1)*8 + lastBits;
			} else {
				*rlen = n*8;
			}

			if (n == 0) {
				n = 1;
			}

			if (n > MAX_LEN) {
				n = MAX_LEN;
			}

			/* Read the recieved data from FIFO */
			for (i=0; i<n; i++) {
			result[i] = read_RFID(FIFODataReg);
			}
		}
		} else {
		status = MI_ERR;
		}
	}
	return status;
}



uint8_t request_tag(uint8_t mode, uint8_t *data)
{
	int status, len;
	write_RFID(BitFramingReg, 0x07);
	data[0] = mode;

	status = commandTag(MFRC522_TRANSCEIVE, data, 1, data, &len);

	if((status != MI_OK) || (len != 0x10)){
		status = MI_ERR;
	};

	return status;
}


uint8_t mfrc522_get_card_serial(uint8_t *serial_out)
{
	int status, i, len;
	uint8_t check = 0x00;

	write_RFID(BitFramingReg, 0x00);

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
devMFRC522init(WarpSPIDeviceState volatile* deviceStatePointer)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI, MISO and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
 	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);
 	PORT_HAL_SetMuxMode(PORTA_BASE, 6u, kPortMuxAlt3);

 	enableSPIpins();

 	/*
 	 *	Override Warp firmware's use of these pins.
 	 *
 	 *	Reconfigure to use as GPIO.
 	 */
 	PORT_HAL_SetMuxMode(PORTA_BASE, 5u, kPortMuxAsGpio);
 	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
 	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);

	// Set CS pin high
	GPIO_DRV_SetPinOutput(kMFRC522PinCSn);
	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kMFRC522PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kMFRC522PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kMFRC522PinRST);
	OSA_TimeDelay(100);

	write_RFID(TModeReg, 0x8D);       // These 4 lines input the prescaler to the timer - see datasheet for why these values
	write_RFID(TPrescalerReg, 0x3E);
	write_RFID(TReloadRegL, 30);
	write_RFID(TReloadRegH, 0);

	write_RFID(TxAutoReg, 0x40);      /* 100%ASK */
	write_RFID(ModeReg, 0x3D);

	setBitMask(TxControlReg, 0x03);        /* Turn antenna on */
//	SEGGER_RTT_printf(0, "Firmware Version: %d", getFirmwareVersion());
	return;
}
