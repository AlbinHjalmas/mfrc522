// Avoid recursive inclusion
#ifndef MFRC522_H_
#define MFRC522_H_
/************************************************************************************
 * Includes
 ***********************************************************************************/
#include "alt_types.h"
#include "../../SoftSPI/inc/SoftSPI.h"

 /***********************************************************************************
 * Defines
 ***********************************************************************************/
// MFRC522 error codes is returned when communication
#define MI_OK                 0
#define MI_NOTAGERR           1
#define MI_ERR                2

//MFRC522 Command words
#define PCD_IDLE              0x00               //NO action; Cancel the current command
#define PCD_AUTHENT           0x0E               //Authentication Key
#define PCD_RECEIVE           0x08               //Receive Data
#define PCD_TRANSMIT          0x04               //Transmit data
#define PCD_TRANSCEIVE        0x0C               //Transmit and receive data,
#define PCD_RESETPHASE        0x0F               //Reset
#define PCD_CALCCRC           0x03               //CRC Calculate

// Mifare_One card command words
# define PICC_REQIDL          0x26               // find the antenna area, does not enter hibernation
# define PICC_REQALL          0x52               // find all the cards within antenna area
# define PICC_ANTICOLL        0x93               // anti-collision
# define PICC_SElECTTAG       0x93               // election card
# define PICC_AUTHENT1A       0x60               // authentication key A
# define PICC_AUTHENT1B       0x61               // authentication key B
# define PICC_READ            0x30               // Read Block
# define PICC_WRITE           0xA0               // write block
# define PICC_DECREMENT       0xC0               // debit
# define PICC_INCREMENT       0xC1               // recharge
# define PICC_RESTORE         0xC2               // transfer block data to the buffer
# define PICC_TRANSFER        0xB0               // save the data in the buffer
# define PICC_HALT            0x50               // Sleep

// MFRC522 Registers
//Page 0:Command and Status
#define     Reserved00            0x00
#define     CommandReg            0x01
#define     CommIEnReg            0x02
#define     DivlEnReg             0x03
#define     CommIrqReg            0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved01            0x0F
//Page 1:Command
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved11            0x1A
#define     Reserved12            0x1B
#define     MifareReg             0x1C
#define     Reserved13            0x1D
#define     Reserved14            0x1E
#define     SerialSpeedReg        0x1F
//Page 2:CFG
#define     Reserved20            0x20
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     Reserved21            0x23
#define     ModWidthReg           0x24
#define     Reserved22            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsPReg              0x28
#define     ModGsPReg             0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F
//Page 3:TestRegister
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     Reserved31            0x3C
#define     Reserved32            0x3D
#define     Reserved33            0x3E
#define     Reserved34            0x3F

 /***********************************************************************************
 * Types
 ***********************************************************************************/
// Mfrc522 object
typedef struct {
	SoftSPI_t* spi_dev;
	SoftSPI_slave_t* spi_slave;
	alt_u32 arduino_base;
	alt_u32 rst_pin;
} Mfrc522_t;

 /***********************************************************************************
 * Function Prototypes
 ***********************************************************************************/

/** Setup and initialize a Mfrc522_t device.
 * SPI device and slave should be initialized prior to this call.
 * Reset pin will be initialized in this function.
 *
 * @param mfrc_dev		The device struct to initialized
 * @param spi_dev		A previously initialized SPI device
 * @param arduino_base	Address to arduino hardware base address
 * @param rst_pin	 	Reset pin on arduino port
 */
void Mfrc522_init(Mfrc522_t* mfrc_dev, SoftSPI_t* spi_dev, SoftSPI_slave_t* spi_slave,
		alt_u32 arduino_base, alt_u32 rst_pin);

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
void Mfrc522_write(Mfrc522_t* mfrc_dev, alt_u8 reg, alt_u8 data);

/** Brief Reads a byte of data from the specified register.
 * Reads a byte of data over the SPI interface from the device.
 *
 * @param mfrc_dev	The MFRC522 device to read from.
 * @param reg		The address to read from.
 * @return the read value.
 */
alt_u8 Mfrc522_read(Mfrc522_t* mfrc_dev, alt_u8 reg);

/** Brief Set bits in MFRC522 register.
 * Sets the specified bits of the specified register.
 *
 * @param mfrc_dev	The device to to set bits in.
 * @param reg		The register to alter.
 * @param mask		The bits to set.
 * @return none.
 */
void Mfrc522_setBitMask(Mfrc522_t* mfrc_dev, alt_u8 reg, alt_u8 mask);

/** Brief Clears the specified bits of the specified register.
 * Clears bits in MFRC522 register.
 *
 * @param mfrc_dev	The device in which to clear bits.
 * @param reg		The register in which to clear bits.
 * @param mask		The bits to clear.
 */
void Mfrc522_clearBitMask(Mfrc522_t* mfrc_dev, alt_u8 reg, alt_u8 mask);

/** Brief Turn RFID detection antenna on of the specified device.
 * @param mfrc_dev	The device on which to turn the antenna on.
 * @return none.
 */
void Mfrc522_antennaOn(Mfrc522_t* mfrc_dev);

/** Brief Shut down RFID antenna of the specified device.
 * Should be at least 1ms between toggling the antenna on/off state,
 * according to the datasheet.
 *
 * @param mfrc_dev	The device on which to turn the antenna off.
 */
void Mfrc522_antennaOff(Mfrc522_t* mfrc_dev);

/** Brief reset MFRC522 device.
 * @param mfrc_dev	The device to reset.
 * @return none.
 */
void Mfrc522_reset(Mfrc522_t* mfrc_dev);

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
 * @param reqMode	The way to find cards (look in datasheet for info).
 * @param tagType	Will contain the type of card after succesfull execution.
 * @return MRFC522 error code.
 */
alt_u8 Mfrc522_request(Mfrc522_t* mfrc_dev, alt_u8 reqMode, alt_u8* tagType);

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
		alt_u8 txLen, alt_u8* rxData, alt_u32* rxLen);

/** Brief Anti-collision detection, reading selected card serial number.
 * @param mfrc_dev	The device to communicate to.
 * @param serNum	Will contain card serial number after execution.
 * 					the first 5 bytes for the checksum byte.
 * @return MRFC522 error code.
 */
alt_u8 Mfrc522_anticoll(Mfrc522_t* mfrc_dev, alt_u8* serNum);

/** Brief CRC calculation with MFRC522.
 *
 * @param mfrc_dev	The MFRC522 device.
 * @param inData	A vector containing data indata for CRC calculation.
 * @param len		The length of inData and outData
 * @param outData	The results from CRC calculation.
 * @return none.
 */
void Mfrc522_calculateCrc(Mfrc522_t* mfrc_dev, alt_u8* inData,
		alt_u8 len, alt_u8* outData);

/** Brief Select a card and return the cards capacity.
 *
 * @param mfrc_dev	The MRFC522 device.
 * @param serNum	The serial number of card to select.
 * @return the selected cards capacity.
 */
alt_u8 Mfrc522_selTag(Mfrc522_t* mfrc_dev, alt_u8* serNum);

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
alt_u8 Mfrc522_authenticate(Mfrc522_t* mfrc_dev, alt_u8 authMode,
		alt_u8 BlockAddr, alt_u8 *Sectorkey, alt_u8 *serNum);

/** Brief Read blockdata from connected RFID card.
 *
 * @param mfrc_dev	The MFRC522 device.
 * @param blockAddr	The address to block to read.
 * @param rxData	The place to store the read data.
 * @return MFRC522 error code.
 */
alt_u8 Mfrc522_read_blockData(Mfrc522_t* mfrc_dev, alt_u8 blockAddr, alt_u8* rxData);

/** Brief Write block data to connected card.
 *
 * @param mfrc_dev	The MFRC522 device.
 * @param blockAddr The address to the block to write to.
 * @param txData	The data to write to the card.
 * @return MFRC522 error code.
 */
alt_u8 Mfrc522_write_blockData(Mfrc522_t* mfrc_dev, alt_u8 blockAddr, alt_u8* txData);

/** Brief Set MFRC522 device into hibernation.
 *
 * @param mfrc_dev	The device to halt.
 * @return none.
 */
void Mfrc522_halt(Mfrc522_t* mfrc_dev);

#endif /* MFRC522_H_ */
