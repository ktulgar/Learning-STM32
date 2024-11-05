#include "rcc522.h"

/*
 * Function Name: RC522_SPI_Transfer
 * Description: A common function used by Write_MFRC522 and Read_MFRC522
 * Input Parameters: data - the value to be written
 * Returns: a byte of data read from the module
 */
uint8_t RC522_SPI_Transfer(uchar data)
{
	uchar rx_data;
	HAL_SPI_TransmitReceive(HSPI_INSTANCE,&data,&rx_data,1,100);

	return rx_data;
}

/*
 * Function Name: Write_MFRC522
 * Function Description: To a certain MFRC522 register to write a byte of data
 * Input Parameters: addr - register address; val - the value to be written
 * Return value: None
 */
void Write_MFRC522(uchar addr, uchar val)
{
	/* CS LOW */
	HAL_GPIO_WritePin(MFRC522_CS_PORT,MFRC522_CS_PIN,GPIO_PIN_RESET);

	  // even though we are calling transfer frame once, we are really sending
	  // two 8-bit frames smooshed together-- sending two 8 bit frames back to back
	  // results in a spike in the select line which will jack with transactions
	  // - top 8 bits are the address. Per the spec, we shift the address left
	  //   1 bit, clear the LSb, and clear the MSb to indicate a write
	  // - bottom 8 bits are the data bits being sent for that address, we send them
	RC522_SPI_Transfer((addr<<1)&0x7E);
	RC522_SPI_Transfer(val);

	/* CS HIGH */
	HAL_GPIO_WritePin(MFRC522_CS_PORT,MFRC522_CS_PIN,GPIO_PIN_SET);
}

/*
 * Function Name: Read_MFRC522
 * Description: From a certain MFRC522 read a byte of data register
 * Input Parameters: addr - register address
 * Returns: a byte of data read from the module
 */
uchar Read_MFRC522(uchar addr)
{
	uchar val;

	/* CS LOW */
	HAL_GPIO_WritePin(MFRC522_CS_PORT,MFRC522_CS_PIN,GPIO_PIN_RESET);

	  // even though we are calling transfer frame once, we are really sending
	  // two 8-bit frames smooshed together-- sending two 8 bit frames back to back
	  // results in a spike in the select line which will jack with transactions
	  // - top 8 bits are the address. Per the spec, we shift the address left
	  //   1 bit, clear the LSb, and set the MSb to indicate a read
	  // - bottom 8 bits are all 0s on a read per 8.1.2.1 Table 6
	RC522_SPI_Transfer(((addr<<1)&0x7E) | 0x80);
	val = RC522_SPI_Transfer(0x00);

	/* CS HIGH */
	HAL_GPIO_WritePin(MFRC522_CS_PORT,MFRC522_CS_PIN,GPIO_PIN_SET);

	return val;

}

/*
 * Function Name: SetBitMask
 * Description: Set RC522 register bit
 * Input parameters: reg - register address; mask - set value
 * Return value: None
 */
void SetBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);  // set bit mask
}

/*
 * Function Name: ClearBitMask
 * Description: clear RC522 register bit
 * Input parameters: reg - register address; mask - clear bit value
 * Return value: None
*/
void ClearBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
}

/*
 * Function Name: AntennaOn
 * Description: Open antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
 * Input: None
 * Return value: None
 */
void AntennaOn(void)
{

	Read_MFRC522(TxControlReg);
	SetBitMask(TxControlReg, 0x03);
}

/*
  * Function Name: AntennaOff
  * Description: Close antennas, each time you start or shut down the natural barrier between the transmitter should be at least 1ms interval
  * Input: None
  * Return value: None
 */
void AntennaOff(void)
{
	ClearBitMask(TxControlReg, 0x03);
}

/*
 * Function Name: MFRC522_Reset
 * Description: Reset RC522
 * Input: None
 * Return value: None
 */
void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}

/*
 * Function Name: MFRC522_Init
 * Description: Initialize RC522
 * Input: None
 * Return value: None
*/
void MFRC522_Init(void)
{
	HAL_GPIO_WritePin(MFRC522_CS_PORT,MFRC522_CS_PIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(MFRC522_RST_PORT,MFRC522_RST_PIN,GPIO_PIN_SET);
	MFRC522_Reset();

	//Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
	Write_MFRC522(TModeReg, 0x8D);		//Tauto=1; f(Timer) = 6.78MHz/TPreScaler
	Write_MFRC522(TPrescalerReg, 0x3E);	//TModeReg[3..0] + TPrescalerReg
	Write_MFRC522(TReloadRegL, 30);
	Write_MFRC522(TReloadRegH, 0);

	Write_MFRC522(TxAutoReg, 0x40);		// force 100% ASK modulation
	Write_MFRC522(ModeReg, 0x3D);		// CRC Initial value 0x6363

	AntennaOn();
}

/*
 * Function Name: MFRC522_ToCard
 * Description: RC522 and ISO14443 card communication
 * Input Parameters: command - MF522 command word,
 *			 sendData--RC522 sent to the card by the data
 *			 sendLen--Length of data sent
 *			 backData--Received the card returns data,
 *			 backLen--Return data bit length
 * Return value: the successful return MI_OK
 */
uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen)
{
    uchar status = MI_ERR;
    uchar irqEn = 0x00;
    uchar waitIRq = 0x00;
    uchar lastBits;
    uchar n;
    uint i;

    switch (command)
    {
        case PCD_AUTHENT:		// Certification cards close
		{
			irqEn = 0x12;
			waitIRq = 0x10;
			break;
		}
		case PCD_TRANSCEIVE:	// Transmit FIFO data
		{
			irqEn = 0x77;
			waitIRq = 0x30;
			break;
		}
		default:
			break;
    }

    Write_MFRC522(CommIEnReg, irqEn|0x80);	// Interrupt request
    ClearBitMask(CommIrqReg, 0x80);			// Clear all interrupt request bit
    SetBitMask(FIFOLevelReg, 0x80);			// FlushBuffer=1, FIFO Initialization

	Write_MFRC522(CommandReg, PCD_IDLE);	// NO action; Cancel the current command

	// Writing data to the FIFO
    for (i=0; i<sendLen; i++)
    {
		Write_MFRC522(FIFODataReg, sendData[i]);
	}

    // Execute the command
	Write_MFRC522(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
		SetBitMask(BitFramingReg, 0x80);		// StartSend=1,transmission of data starts
	}

    // Waiting to receive data to complete
	i = 2000;	// i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms
    do
    {
		//CommIrqReg[7..0]
		//Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
        n = Read_MFRC522(CommIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x01) && !(n&waitIRq));

    ClearBitMask(BitFramingReg, 0x80);			//StartSend=0

    if (i != 0)
    {
        if(!(Read_MFRC522(ErrorReg) & 0x1B))	//BufferOvfl Collerr CRCErr ProtecolErr
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
				status = MI_NOTAGERR;
			}

            if (command == PCD_TRANSCEIVE)
            {
               	n = Read_MFRC522(FIFOLevelReg);
              	lastBits = Read_MFRC522(ControlReg) & 0x07;
                if (lastBits)
                {
					*backLen = (n-1)*8 + lastBits;
				}
                else
                {
					*backLen = n*8;
				}

                if (n == 0)
                {
					n = 1;
				}
                if (n > MAX_LEN)
                {
					n = MAX_LEN;
				}

                // Reading the received data in FIFO
                for (i=0; i<n; i++)
                {
					backData[i] = Read_MFRC522(FIFODataReg);
				}
            }
        }
        else
        {
			status = MI_ERR;
		}

    }

    //SetBitMask(ControlReg,0x80);           //timer stops
    //Write_MFRC522(CommandReg, PCD_IDLE);

    return status;
}

/*
 * Function Name: MFRC522_Request
 * Description: Find cards, read the card type number
 * Input parameters: reqMode - find cards way
 *   TagType - Return Card Type
 *    0x4400 = Mifare_UltraLight
 *    0x0400 = Mifare_One(S50)
 *    0x0200 = Mifare_One(S70)
 *    0x0800 = Mifare_Pro(X)
 *    0x4403 = Mifare_DESFire
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Request(uchar reqMode, uchar *TagType)
{
	uchar status;
	uint backBits;			 // The received data bits

	Write_MFRC522(BitFramingReg, 0x07);		//TxLastBists = BitFramingReg[2..0]

	TagType[0] = reqMode;
	status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

	if ((status != MI_OK) || (backBits != 0x10))
	{
		status = MI_ERR;
	}

	return status;
}

/*
 * Function Name: MFRC522_Anticoll
 * Description: Anti-collision detection, reading selected card serial number card
 * Input parameters: serNum - returns 4 bytes card serial number, the first 5 bytes for the checksum byte
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Anticoll(uchar *serNum)
{
    uchar status;
    uchar i;
	uchar serNumCheck=0;
    uint unLen;

	Write_MFRC522(BitFramingReg, 0x00);		//TxLastBists = BitFramingReg[2..0]

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
	{
    	 //Check card serial number
		for (i=0; i<4; i++)
		{
		 	serNumCheck ^= serNum[i];
		}
		if (serNumCheck != serNum[i])
		{
			status = MI_ERR;
		}
    }

    return status;
}

/*
 * Function Name: CalulateCRC
 * Description: CRC calculation with MF522
 * Input parameters: pIndata - To read the CRC data, len - the data length, pOutData - CRC calculation results
 * Return value: None
 */
void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData)
{
    uchar i, n;

    ClearBitMask(DivIrqReg, 0x04);			//CRCIrq = 0
    SetBitMask(FIFOLevelReg, 0x80);			//Clear the FIFO pointer

    //Writing data to the FIFO
    for (i=0; i<len; i++)
    {
		Write_MFRC522(FIFODataReg, *(pIndata+i));
	}
    Write_MFRC522(CommandReg, PCD_CALCCRC);

    //Wait CRC calculation is complete
    i = 0xFF;
    do
    {
        n = Read_MFRC522(DivIrqReg);
        i--;
    }
    while ((i!=0) && !(n&0x04));			//CRCIrq = 1

    //Read CRC calculation result
    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegH);
}

/*
 * Function Name: MFRC522_SelectTag
 * Description: election card, read the card memory capacity
 * Input parameters: serNum - Incoming card serial number
 * Return value: the successful return of card capacity
 */
uchar MFRC522_SelectTag(uchar *serNum)
{
	uchar i;
	uchar status;
	uchar size;
	uint recvBits;
	uchar buffer[9];

	//ClearBitMask(Status2Reg, 0x08);			//MFCrypto1On=0

    buffer[0] = PICC_SElECTTAG;
    buffer[1] = 0x70;
    for (i=0; i<5; i++)
    {
    	buffer[i+2] = *(serNum+i);
    }
	CalulateCRC(buffer, 7, &buffer[7]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

    if ((status == MI_OK) && (recvBits == 0x18))
    {
		size = buffer[0];
	}
    else
    {
		size = 0;
	}

    return size;
}

/*
 * Function Name: MFRC522_Auth
 * Description: Verify card password
 * Input parameters: authMode - Password Authentication Mode
                 0x60 = A key authentication
                 0x61 = Authentication Key B
             BlockAddr--Block address
             Sectorkey--Sector password
             serNum--Card serial number, 4-byte
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Auth(uchar authMode, uchar BlockAddr, uchar *Sectorkey, uchar *serNum)
{
    uchar status;
    uint recvBits;
    uchar i;
	uchar buff[12];

	//Verify the command block address + sector + password + card serial number
    buff[0] = authMode;
    buff[1] = BlockAddr;
    for (i=0; i<6; i++)
    {
		buff[i+2] = *(Sectorkey+i);
	}
    for (i=0; i<4; i++)
    {
		buff[i+8] = *(serNum+i);
	}
    status = MFRC522_ToCard(PCD_AUTHENT, buff, 12, buff, &recvBits);

    if ((status != MI_OK) || (!(Read_MFRC522(Status2Reg) & 0x08)))
    {
		status = MI_ERR;
	}

    return status;
}

/*
 * Function Name: MFRC522_Read
 * Description: Read block data
 * Input parameters: blockAddr - block address; recvData - read block data
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Read(uchar blockAddr, uchar *recvData)
{
    uchar status;
    uint unLen;

    recvData[0] = PICC_READ;
    recvData[1] = blockAddr;
    CalulateCRC(recvData,2, &recvData[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

    if ((status != MI_OK) || (unLen != 0x90))
    {
        status = MI_ERR;
    }

    return status;
}

/*
 * Function Name: MFRC522_Write
 * Description: Write block data
 * Input parameters: blockAddr - block address; writeData - to 16-byte data block write
 * Return value: the successful return MI_OK
 */
uchar MFRC522_Write(uchar blockAddr, uchar *writeData)
{
    uchar status;
    uint recvBits;
    uchar i;
	uchar buff[18];

    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
		status = MI_ERR;
	}

    if (status == MI_OK)
    {
        for (i=0; i<16; i++)		//Data to the FIFO write 16Byte
        {
        	buff[i] = *(writeData+i);
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

		if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {
			status = MI_ERR;
		}
    }

    return status;
}

/*
 * Function Name: MFRC522_Halt
 * Description: Command card into hibernation
 * Input: None
 * Return value: None
 */
void MFRC522_Halt(void)
{
	uint unLen;
	uchar buff[4];

	buff[0] = PICC_HALT;
	buff[1] = 0;
	CalulateCRC(buff, 2, &buff[2]);

	MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);
}
