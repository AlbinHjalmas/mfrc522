#ifdef __cplusplus
extern "C"
{
#endif

/************************************************************************************
 * Includes
 ***********************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "../inc/Mfrc522.h"
#include "io.h"

/************************************************************************************
 * Defines
 ***********************************************************************************/
// Maximum length of the array
#define MAX_LEN 16

// Arduino hardware control
#define PIN_INPUT_MODE 0
#define PIN_OUTPUT_MODE 1
#define set_pin_mode(base, pin, mode) IOWR_32DIRECT(base, 0, ((pin<<1)|mode))
#define digital_write(base, pin, data) IOWR_32DIRECT(base, 4, ((pin<<1)|data))
#define digital_read(base, pin) IORD_32DIRECT(base, pin*4)

/************************************************************************************
 * Function declarations
 ***********************************************************************************/

/** Brief Setup and initialize a Mfrc522_t device.
 * SPI device and slave shold be initialized prior to this call.
 * Reset pin will be initialized in this function.
 *
 * @param mfrc_dev		The device struct to initialized
 * @param spi_dev		A previously initialized SPI device
 * @param arduino_base	Address to arduino hardware base address
 * @param rst_pin	 	Reset pin on arduino port
 */
void Mfrc522_init(Mfrc522_t* mfrc_dev, SoftSPI_t* spi_dev, SoftSPI_slave_t* spi_slave,
		alt_u32 arduino_base, alt_u32 rst_pin) {
	// Initialize device, assumes spi device and slave
	// is already initialized
	mfrc_dev->spi_dev = spi_dev;
	mfrc_dev->spi_slave = spi_slave;
	mfrc_dev->arduino_base = arduino_base;
	mfrc_dev->rst_pin = rst_pin;

	// Setup reset pin
	set_pin_mode(arduino_base, rst_pin, PIN_OUTPUT_MODE);

	// Set reset pin of MFRC522
	digital_write(arduino_base, rst_pin, 1);

	// Reset MFRC522 card
	Mfrc522_reset(mfrc_dev);
}

/** Brief writes a byte of data to the specified register.
 * Uses the SPI device which the Mfrc522_t device was initialized
 * with to write data into the specified register.
 *
 * @param mfrc_dev 	the initialized MFRC522 device which to send a
 * 					a byte of data to.
 * @param reg		The register in the MFRC522 device which should
 * 					be written to.
 * @param data		The data to written to the specified register.
 * @return none.
 */
void Mfrc522_write(Mfrc522_t* mfrc_dev, alt_u8 reg, alt_u8 data) {
	alt_u8 txFrame[2];
	alt_u8 rxFrame[2];

	// Generate address according to data sheet
	txFrame[0] = (reg * 2) & 0x7E;
	txFrame[1] = data;

	// even though we are calling transfer frame once, we are really sending
	// two 8-bit frames smooshed together-- sending two 8 bit frames back to back
	// results in a spike in the select line which will jack with transactions
	// - top 8 bits are the address. Per the spec, we shift the address left
	//   1 bit, clear the LSb, and clear the MSb to indicate a write
	// - bottom 8 bits are the data bits being sent for that address, we send
	//   them as is
	SoftSPI_txrx(mfrc_dev->spi_dev, mfrc_dev->spi_slave, rxFrame, txFrame, 2);
}

/** Brief Reads a byte of data from the specified register.
 * Reads a byte of data over the SPI interface from the device.
 *
 * @param mfrc_dev	The MFRC522 device to read from.
 * @param reg		The address to read from.
 * @return the read value.
 */
alt_u8 Mfrc522_read(Mfrc522_t* mfrc_dev, alt_u8 reg) {
	// Per the datasheet, we shift the address left
	// 1 bit, clear the LSb, and set the MSb to indicate a read
	// last 8 bits are all 0s on a read per 8.1.2.1 Table 6 in datasheet
	alt_u8 txFrame[] = {((reg * 2) & 0x7E) | 0x80, 0};
	alt_u8 rxFrame[2];

	// Read from device
	SoftSPI_txrx(mfrc_dev->spi_dev, mfrc_dev->spi_slave, rxFrame, txFrame, 2);

	return rxFrame[1];
}

/** Brief Set bits in MFRC522 register.
 * Sets the specified bits of the specified register.
 *
 * @param mfrc_dev	The device to to set bits in.
 * @param reg		The register to alter.
 * @param mask		The bits to set.
 * @return none.
 */
void Mfrc522_setBitMask(Mfrc522_t* mfrc_dev, alt_u8 reg, alt_u8 mask) {
    alt_u8 tmp;
    tmp = Mfrc522_read(mfrc_dev, reg);
    Mfrc522_write(mfrc_dev, reg, tmp | mask);  // set bit mask
}

/** Brief Clears the specified bits of the specified register.
 * Clears bits in MFRC522 register.
 *
 * @param mfrc_dev	The device in which to clear bits.
 * @param reg		The register in which to clear bits.
 * @param mask		The bits to clear.
 */
void Mfrc522_clearBitMask(Mfrc522_t* mfrc_dev, alt_u8 reg, alt_u8 mask) {
    alt_u8 tmp;
    tmp = Mfrc522_read(mfrc_dev, reg);
    Mfrc522_write(mfrc_dev, reg, tmp & (~mask));  // clear bit mask
} 

/** Brief Turn RFID detection antenna on of the specified device.
 * @param mfrc_dev	The device on which to turn the antenna on.
 * @return none.
 */
void Mfrc522_antennaOn(Mfrc522_t* mfrc_dev) {
	Mfrc522_setBitMask(mfrc_dev, TxControlReg, 0x03);
}

/** Brief Shut down RFID antenna of the specified device.
 * Should be at least 1ms between toggling the antenna on/off state,
 * according to the datasheet.
 *
 * @param mfrc_dev	The device on which to turn the antenna off.
 */
void Mfrc522_antennaOff(Mfrc522_t* mfrc_dev) {
	Mfrc522_clearBitMask(mfrc_dev, TxControlReg, 0x03);
}

/** Brief reset MFRC522 device.
 * @param mfrc_dev	The device to reset.
 * @return none.
 */
void Mfrc522_reset(Mfrc522_t* mfrc_dev) {
	Mfrc522_write(mfrc_dev, CommandReg, PCD_RESETPHASE);
	// Timer: TPrescaler * TreloadVal / 6.78MHz = 24ms
	Mfrc522_write(mfrc_dev, TModeReg, 0x8D); // Tauto = 1; f(Timer) = 6.78MHz / TPreScaler
	Mfrc522_write(mfrc_dev, TPrescalerReg, 0x3E); // TModeReg[3..0] + TPrescalerReg
	Mfrc522_write(mfrc_dev, TReloadRegL, 30);
	Mfrc522_write(mfrc_dev, TReloadRegH, 0);
	Mfrc522_write(mfrc_dev, TxAutoReg, 0x40); // force 100% ASK modulation
	Mfrc522_write(mfrc_dev, ModeReg, 0x3D); // CRC Initial value 0x6363

	// turn antenna on
	Mfrc522_antennaOn(mfrc_dev);
}

/** Brief Find cards and read the card type number.
 * If a card was successfully found, The parameter "tagType" will containg
 * The ID of the card type.
 * Different card type ID's are shown below:
 *    0x4400 = Mifare_UltraLight
 *    0x0400 = Mifare_One(S50)
 *    0x0200 = Mifare_One(S70)
 *    0x0800 = Mifare_Pro(X)
 *    0x4403 = Mifare_DESFire
 *
 * @param mfrc_dev	The device to find cards with.
 * @param reqMode	The way to find cards (look in data sheet for info).
 * @param tagType	Will contain the type of card after successful execution.
 * @return MRFC522 error code.
 */
alt_u8 Mfrc522_request(Mfrc522_t* mfrc_dev, alt_u8 reqMode, alt_u8* tagType) {
	alt_u8 status;
	alt_u32 backBits; // The received data bits

	Mfrc522_write(mfrc_dev, BitFramingReg, 0x07);   // TxLastBits = BitFramingReg[2..0]

	tagType[0] = reqMode;

	status = Mfrc522_cardTxRx(mfrc_dev, PCD_TRANSCEIVE, tagType, 1, tagType, &backBits);
	if ((status != MI_OK) || (backBits != 0x10)) {
		status = MI_ERR;
	}

	return status;
}

/** Brief MFRC522 and ISO14443 card communication.
 *
 * @param mfrc_dev		The MFRC522 RFID device.
 * @param command		The command to use, PCD_AUTHENT - certification or PCD_TRANCIEVE
 * 						- transmit FIFO data.
 * @param txData		The data to send to the card.
 * @param txLen			Number of bytes to send (sizeof(txData)).
 * @param rxData		The data received from the card.
 * @param rxLen			Number of bytes to receive from the card.
 * @return MRFC522 error code.
 */
alt_u8 Mfrc522_cardTxRx(Mfrc522_t* mfrc_dev, alt_u8 command, alt_u8* txData,
		alt_u8 txLen, alt_u8* rxData, alt_u32* rxLen) {
	alt_u8 status = MI_ERR;
	alt_u8 irqEn = 0x00;
	alt_u8 waitIRq = 0x00;
	alt_u8 lastBits;
	alt_u8 n;
	alt_u32 i;

	switch (command) {
	case PCD_AUTHENT:     // Certification cards close
		irqEn = 0x12;
		waitIRq = 0x10;
		break;
	case PCD_TRANSCEIVE:  // Transmit FIFO data
		irqEn = 0x77;
		waitIRq = 0x30;
		break;
	default:
		break;
	}

	Mfrc522_write(mfrc_dev, CommIEnReg, irqEn|0x80); // Interrupt request
	Mfrc522_clearBitMask(mfrc_dev, CommIrqReg, 0x80); // Clear all interrupt request bit
	Mfrc522_setBitMask(mfrc_dev, FIFOLevelReg, 0x80); // FlushBuffer=1, FIFO Initialization

	Mfrc522_write(mfrc_dev, CommandReg, PCD_IDLE); // NO action; Cancel the current command

	// Writing data to the FIFO
	for (i=0; i<txLen; i++) {
		Mfrc522_write(mfrc_dev, FIFODataReg, txData[i]);
	}

	// Execute the command
	Mfrc522_write(mfrc_dev, CommandReg, command);
	if (command == PCD_TRANSCEIVE) {
		Mfrc522_setBitMask(mfrc_dev, BitFramingReg, 0x80); // StartSend=1, transmission of data starts
	}

	// Waiting to receive data to complete
	i = 2000;	// i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms
	do {
		// CommIrqReg[7..0]
		// Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
		n = Mfrc522_read(mfrc_dev, CommIrqReg);
    	i--;
	} while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

	Mfrc522_clearBitMask(mfrc_dev, BitFramingReg, 0x80); // StartSend=0

	if (i != 0) {
		// BufferOvfl Collerr CRCErr ProtecolErr
		if(!(Mfrc522_read(mfrc_dev, ErrorReg) & 0x1B)) {
			status = MI_OK;
			if (n & irqEn & 0x01) {
				status = MI_NOTAGERR;
			}

			if (command == PCD_TRANSCEIVE) {
				n = Mfrc522_read(mfrc_dev, FIFOLevelReg);
				lastBits = Mfrc522_read(mfrc_dev, ControlReg) & 0x07;
				if (lastBits) {
					*rxLen = (n-1) * 8 + lastBits;
				} else {
					*rxLen = n*8;
				}

				if (n == 0) {
					n = 1;
				}

				if (n > MAX_LEN) {
					n = MAX_LEN;
				}

				// Reading the received data in FIFO
				for (i=0; i<n; i++) {
					rxData[i] = Mfrc522_read(mfrc_dev, FIFODataReg);
				}
			}
		} else {
			status = MI_ERR;
		}
	} else {
		status = MI_ERR;
	}

	return status;
}

/** Brief Anti-collision detection, reading selected card serial number.
 * @param mfrc_dev	The device to communicate to.
 * @param serNum	Will contain card serial number after execution.
 * 					the first 5 bytes for the checksum byte.
 * @return MRFC522 error code.
 */
alt_u8 Mfrc522_anticoll(Mfrc522_t* mfrc_dev, alt_u8* serNum) {
	alt_u8 status;
	alt_u32 unLen;

	Mfrc522_write(mfrc_dev, BitFramingReg, 0x00); // TxLastBists = BitFramingReg[2..0]

	serNum[0] = PICC_ANTICOLL;
	serNum[1] = 0x20;
	status = Mfrc522_cardTxRx(mfrc_dev, PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

	return status;
} 


/** Brief CRC calculation with MFRC522.
 *
 * @param mfrc_dev	The MFRC522 device.
 * @param inData	A vector containing data indata for CRC calculation.
 * @param len		The length of inData and outData
 * @param outData	The results from CRC calculation.
 * @return none.
 */
/*
void Mfrc522_calculateCrc(Mfrc522_t* mfrc_dev, alt_u8* inData, alt_u8 len, alt_u8* outData) {
	alt_u8 i, n;

	Mfrc522_clearBitMask(mfrc_dev, DivIrqReg, 0x04); // CRCIrq = 0
	Mfrc522_setBitMask(mfrc_dev, FIFOLevelReg, 0x80); // Clear the FIFO pointer

	//Writing data to the FIFO
	for (i=0; i<len; i++) {
		Mfrc522_write(mfrc_dev, FIFODataReg, *(inData + i));
	}
	Mfrc522_write(mfrc_dev, CommandReg, PCD_CALCCRC);

	//Wait until CRC calculation is complete
	i = 0xFF;
	do {
		n = Mfrc522_read(mfrc_dev, DivIrqReg);
		i--;
	} while ((i != 0) && !(n & 0x04));			//CRCIrq = 1

	//Read CRC calculation result
	outData[0] = Mfrc522_read(mfrc_dev, CRCResultRegL);
	outData[1] = Mfrc522_read(mfrc_dev, CRCResultRegM);
}
*/

/** Brief Select a card and return the cards capacity.
 *
 * @param mfrc_dev	The MRFC522 device.
 * @param serNum	The serial number of card to select.
 * @return the selected cards capacity.
 */
/*
alt_u8 Mfrc522_selTag(Mfrc522_t* mfrc_dev, alt_u8* serNum) {
	alt_u8 i;
	alt_u8 status;
	alt_u8 size;
	alt_u32 recvBits;
	alt_u8 buffer[9];

	buffer[0] = PICC_SElECTTAG;
	buffer[1] = 0x70;

	for (i=0; i<5; i++) {
		buffer[i+2] = *(serNum + i);
	}

	Mfrc522_calculateCrc(mfrc_dev, buffer, 7, &buffer[7]);
	status = Mfrc522_cardTxRx(mfrc_dev, PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

	if ((status == MI_OK) && (recvBits == 0x18)) {
		size = buffer[0];
	} else {
		size = 0;
	}

	return size;
}
*/

/** Brief Verify card password.
 *
 * @param mfrc_dev	The MFRC522 device.
 * @param authMode	The password-authentication mode,
 * 					0x60 = Authentication key A
 * 					0x61 = Authentication key B
 * @param blockAddr The block address.
 * @param sectorKey The sector password.
 * @param serNum	The card serial number, 4 bytes.
 * @return MRFC522 error code.
 */
/*
alt_u8 Mfrc522_authenticate(Mfrc522_t* mfrc_dev, alt_u8 authMode, alt_u8 blockAddr,
		alt_u8* sectorKey, alt_u8* serNum) {
	alt_u8 status;
	alt_u32 recvBits;
	alt_u8 i;
	alt_u8 buff[12];

	//Verify the command block address + sector + password + card serial number
	buff[0] = authMode;
	buff[1] = blockAddr;
	for (i=0; i<6; i++) {
		buff[i+2] = *(sectorKey+i);
	}

	for (i=0; i<4; i++) {
		buff[i+8] = *(serNum + i);
	}

	status = Mfrc522_cardTxRx(mfrc_dev, PCD_AUTHENT, buff, 12, buff, &recvBits);

	if ((status != MI_OK) || (!(Mfrc522_read(mfrc_dev, Status2Reg) & 0x08))) {
		status = MI_ERR;
	}

	return status;
}
*/

/** Brief Read blockdata from connected RFID card.
 *
 * @param mfrc_dev	The MFRC522 device.
 * @param blockAddr	The address to block to read.
 * @param rxData	The place to store the read data.
 * @return MFRC522 error code.
 */
/*
alt_u8 Mfrc522_read_blockData(Mfrc522_t* mfrc_dev, alt_u8 blockAddr, alt_u8* rxData) {
	alt_u8 status;
	alt_u32 unLen;

	rxData[0] = PICC_READ;
	rxData[1] = blockAddr;
	Mfrc522_calculateCrc(mfrc_dev, rxData,2, &rxData[2]);
	status = Mfrc522_cardTxRx(mfrc_dev, PCD_TRANSCEIVE, rxData, 4, rxData, &unLen);

	if ((status != MI_OK) || (unLen != 0x90)) {
		status = MI_ERR;
	}

	return status;
}
*/

/** Brief Write block data to connected card.
 *
 * @param mfrc_dev	The MFRC522 device.
 * @param blockAddr The address to the block to write to.
 * @param txData	The data to write to the card.
 * @return MFRC522 error code.
 */
/*
alt_u8 Mfrc522_write_blockData(Mfrc522_t* mfrc_dev, alt_u8 blockAddr, alt_u8* txData) {
	alt_u8 status;
	alt_u32 recvBits;
	alt_u8 i;
	alt_u8 buff[18];

	buff[0] = PICC_WRITE;
	buff[1] = blockAddr;
	Mfrc522_calculateCrc(mfrc_dev, buff, 2, &buff[2]);
	status = Mfrc522_cardTxRx(mfrc_dev, PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

	if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
		status = MI_ERR;
	}

	if (status == MI_OK) {
		//Data to the FIFO write 16Byte
		for (i=0; i<16; i++) {
			buff[i] = *(txData+i);
		}

		Mfrc522_calculateCrc(mfrc_dev, buff, 16, &buff[16]);
		status = Mfrc522_cardTxRx(mfrc_dev, PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A)) {
			status = MI_ERR;
		}
	}

	return status;
}
*/

/** Brief Set MFRC522 device into hibernation.
 *
 * @param mfrc_dev	The device to halt.
 * @return none.
 */
/*
void Mfrc522_halt(Mfrc522_t* mfrc_dev) {
	alt_u32 unLen;
	alt_u8 buff[4];

	buff[0] = PICC_HALT;
	buff[1] = 0;
	Mfrc522_calculateCrc(mfrc_dev, buff, 2, &buff[2]);

	Mfrc522_cardTxRx(mfrc_dev, PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}
*/

#ifdef __cplusplus
}
#endif
