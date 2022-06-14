#include "io_nand.h"

//-----------------------------------------------------------------------------------------

#ifdef SET_NAND_CMD
	uint8_t tmpBuf[32] = {0};
	uint8_t tmpLen = 0;
	char tmpChar[256];

	void tmpPrint(const char *func, uint8_t *buf, uint8_t len)
	{
		strcpy(tmpChar, "to_nand:");
		for (uint8_t i = 0; i < len; i++) sprintf(tmpChar+strlen(tmpChar), " %02X", *(uint8_t *)(buf + i));
		Report(0, "\t\t%s\r\n", tmpChar);
	}
#endif


//-------------------------------------------------------------------------------------------
void io_nand_init(NAND_HandleTypeDef *hnand)
{
    //if (HAL_NAND_ECC_Disable(hnand) != HAL_OK) devError |= devNAND;

#if (USE_HAL_NAND_REGISTER_CALLBACKS == 1)
    if (HAL_NAND_RegisterCallback(hnand, HAL_NAND_IT_CB_ID, HAL_NAND_ITCallback) == HAL_ERROR) devError |= devNAND;
#endif

    if (NAND_Read_ID(hnand, &nandID) == HAL_OK) {//read ID information from chip

    	nandState = HAL_NAND_GetState(hnand);

    	memcpy((uint8_t *)&chipConf, (uint8_t *)&hnand->Config, sizeof(s_chipConf));
    	chipConf.PlaneSize *= chipConf.BlockNbr;

    	total_pages = chipConf.BlockSize * chipConf.BlockNbr;
    	total_bytes = total_pages * chipConf.PageSize;//chipConf.PlaneSize;

    }

}
//-------------------------------------------------------------------------------------------
uint32_t io_nand_get_page_size(void)
{
    return chipConf.PageSize;
}
uint32_t io_nand_get_block_number(void)
{
	return chipConf.BlockNbr;
}
uint32_t io_nand_get_block_size(void)
{
	return chipConf.BlockSize;
}
uint32_t io_nand_get_plane_number(void)
{
	return chipConf.PlaneNbr;
}
uint32_t io_nand_get_plane_size(void)
{
	return chipConf.PlaneSize;
}
//-----------------------------------------------------------------------------------------
HAL_StatusTypeDef NAND_Read_ID(NAND_HandleTypeDef *hnand, NAND_IDsTypeDef *pNAND_ID)
{

	if (hnand->State == HAL_NAND_STATE_BUSY) {

		return HAL_BUSY;

	} else if (hnand->State == HAL_NAND_STATE_READY) {

		__HAL_LOCK(hnand);
	    hnand->State = HAL_NAND_STATE_BUSY;

#if defined(FMC_Bank2_3)
	    if (hnand->Init.NandBank == FMC_NAND_BANK2) {
	    	devAdr = NAND_DEVICE1;
	    } else {
	    	devAdr = NAND_DEVICE2;
	    }
#else
	    devAdr = NAND_DEVICE;//MY_NAND_DEVICE;
#endif

	    /* Send Read ID command sequence */
	    *(__IO uint8_t *)((uint32_t)(devAdr | CMD_AREA))  = NAND_CMD_READID;
	    __DSB();
	    *(__IO uint8_t *)((uint32_t)(devAdr | ADDR_AREA)) = 0x00;
	    __DSB();

	    /* Read the electronic signature from NAND flash */
	    if (hnand->Init.MemoryDataWidth == FSMC_NAND_PCC_MEM_BUS_WIDTH_8) {
	    	__IO uint32_t data  = *(__IO uint32_t *)devAdr;
	    	__IO uint32_t data1 = *((__IO uint32_t *)devAdr + 4);

	    	pNAND_ID->Maker_Id   = ADDR_1ST_CYCLE(data);
	    	pNAND_ID->Device_Id  = ADDR_2ND_CYCLE(data);
	    	pNAND_ID->Third_Id   = ADDR_3RD_CYCLE(data);
	    	pNAND_ID->Fourth_Id  = ADDR_4TH_CYCLE(data);
	    	pNAND_ID->Plane_Id   = ADDR_1ST_CYCLE(data1);


	    	hnand->State = HAL_NAND_STATE_READY;
	    }

	    __HAL_UNLOCK(hnand);

	} else {
	    return HAL_ERROR;
	}

	return HAL_OK;
}
//-----------------------------------------------------------------------------------------
uint32_t io_nand_read_8b (uint32_t adr, uint8_t *pBuffer, uint32_t size, uint32_t offset)
{
NAND_AddressTypeDef Address = io_uint32_to_flash_adr(adr);


    if (nandPort->State == HAL_NAND_STATE_BUSY) return HAL_BUSY;

    __HAL_LOCK(nandPort);
    nandPort->State = HAL_NAND_STATE_BUSY;

    uint32_t deviceaddress = devAdr;
    uint32_t nandaddress = ARRAY_ADDRESS(&Address, nandPort);

    if (dbg > logOn)
        	    	Report(1, "[%s] nand_adr:0x%X page:%lu plane:%lu block:%lu offset:%lu\r\n",
        	    		      __func__, nandaddress, Address.Page, Address.Plane, Address.Block, offset);

#ifdef SET_NAND_CMD
    bool tflag = true;
    memset(tmpBuf, 0, sizeof(tmpBuf));
    tmpLen = 0;
#else
    bool tflag = false;
#endif

    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_A;
    __DSB();

    if (tflag) tmpBuf[tmpLen++] = NAND_CMD_AREA_A;

    // Cards with page size <= 512 bytes
    if ((nandPort->Config.PageSize) <= 512U) {
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_1ST_CYCLE(offset);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = COLUMN_1ST_CYCLE(offset);;
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = ADDR_1ST_CYCLE(nandaddress);
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = ADDR_2ND_CYCLE(nandaddress);
        if ((nandPort->Config.BlockSize * nandPort->Config.BlockNbr) > 65535U) {
			*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
			__DSB();
			if (tflag) tmpBuf[tmpLen++] = ADDR_3RD_CYCLE(nandaddress);
        }
    } else {// (hnand->Config.PageSize) > 512
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_1ST_CYCLE(offset);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = COLUMN_1ST_CYCLE(offset);
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_2ND_CYCLE(offset);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = COLUMN_2ND_CYCLE(offset);
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = ADDR_1ST_CYCLE(nandaddress);
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = ADDR_2ND_CYCLE(nandaddress);
    	if ((nandPort->Config.BlockSize * nandPort->Config.BlockNbr) > 65535U) {
    		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
    		__DSB();
    		if (tflag) tmpBuf[tmpLen++] = ADDR_3RD_CYCLE(nandaddress);
    	}
    }

    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA))  = NAND_CMD_AREA_TRUE1;
    __DSB();
    if (tflag) tmpBuf[tmpLen++] = NAND_CMD_AREA_TRUE1;


    uint32_t tickstart = 0U;
    // Check if an extra command is needed for reading pages
    if (nandPort->Config.ExtraCommandEnable == ENABLE) {
        tickstart = HAL_GetTick();
        while (HAL_NAND_Read_Status(nandPort) != NAND_READY) {
            if((HAL_GetTick() - tickstart ) > NAND_WRITE_TIMEOUT) {
            	nandPort->State = HAL_NAND_STATE_ERROR;// Update the NAND controller state
            	__HAL_UNLOCK(nandPort);
            	return HAL_TIMEOUT;
            }
        }

        // Go back to read mode
        *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_A;
        __DSB();
        if (tflag) tmpBuf[tmpLen++] = NAND_CMD_AREA_A;
    }

    // Get Data into Buffer
    uint8_t *buff = pBuffer;
    for (uint32_t index = 0; index < size; index++) *buff++ = *(uint8_t *)deviceaddress;

    nandPort->State = HAL_NAND_STATE_READY;
    __HAL_UNLOCK(nandPort);

    if (tflag & (dbg > logOn)) tmpPrint(__func__, tmpBuf, tmpLen);

    return HAL_OK;
}
//-----------------------------------------------------------------------------
uint32_t io_nand_write_8b(uint32_t adr, uint8_t *pBuffer, uint32_t size, uint32_t offset)
{
NAND_AddressTypeDef Address = io_uint32_to_flash_adr(adr);


    if (nandPort->State == HAL_NAND_STATE_BUSY) return HAL_BUSY;

    __HAL_LOCK(nandPort);
    nandPort->State = HAL_NAND_STATE_BUSY;

    uint32_t deviceaddress = devAdr;
    uint32_t nandaddress = ARRAY_ADDRESS(&Address, nandPort);

    if (dbg > logOn)
        	    	Report(1, "[%s] nand_adr:0x%X page:%lu plane:%lu block:%lu offset:%lu\r\n",
        	    	          __func__, nandaddress, Address.Page, Address.Plane, Address.Block, offset);

#ifdef SET_NAND_CMD
    bool tflag = true;
    memset(tmpBuf, 0, sizeof(tmpBuf));
    tmpLen = 0;
#else
    bool tflag = false;
#endif

    /* Send write page command sequence */
    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_A;
    __DSB();
    if (tflag) tmpBuf[tmpLen++] = NAND_CMD_AREA_A;
    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_WRITE0;
    __DSB();
    if (tflag) tmpBuf[tmpLen++] = NAND_CMD_WRITE0;

    /* Cards with page size <= 512 bytes */
    if (nandPort->Config.PageSize <= 512U) {
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_1ST_CYCLE(offset);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = COLUMN_1ST_CYCLE(offset);
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = ADDR_1ST_CYCLE(nandaddress);
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = ADDR_2ND_CYCLE(nandaddress);
    	if ((nandPort->Config.BlockSize * nandPort->Config.BlockNbr) > 65535U) {
    		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
    		__DSB();
    		if (tflag) tmpBuf[tmpLen++] = ADDR_3RD_CYCLE(nandaddress);
        }
    } else {/* (hnand->Config.PageSize) > 512 */
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_1ST_CYCLE(offset);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = COLUMN_1ST_CYCLE(offset);
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_2ND_CYCLE(offset);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = COLUMN_2ND_CYCLE(offset);
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = ADDR_1ST_CYCLE(nandaddress);
    	*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
    	__DSB();
    	if (tflag) tmpBuf[tmpLen++] = ADDR_2ND_CYCLE(nandaddress);
    	if ((nandPort->Config.BlockSize * nandPort->Config.BlockNbr) > 65535U) {
    		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
    		__DSB();
    		if (tflag) tmpBuf[tmpLen++] = ADDR_3RD_CYCLE(nandaddress);
        }
    }


    /* Write data to memory */
    uint8_t *buff = pBuffer;
    for (uint32_t index = 0; index < size; index++) {
    	*(__IO uint8_t *)deviceaddress = *buff++;
    	__DSB();
    }

    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_WRITE_TRUE1;
    __DSB();
    if (tflag) tmpBuf[tmpLen++] = NAND_CMD_WRITE_TRUE1;

    /* Read status until NAND is ready */
    uint32_t tickstart;
    while(HAL_NAND_Read_Status(nandPort) != NAND_READY) {
        tickstart = HAL_GetTick();
        if ((HAL_GetTick() - tickstart ) > NAND_WRITE_TIMEOUT) {
        	nandPort->State = HAL_NAND_STATE_ERROR;// Update the NAND controller state
        	__HAL_UNLOCK(nandPort);
            return HAL_TIMEOUT;
        }
    }

    nandPort->State = HAL_NAND_STATE_READY;
    __HAL_UNLOCK(nandPort);

    if (tflag & (dbg > logOn)) tmpPrint(__func__, tmpBuf, tmpLen);

    return HAL_OK;
}
//-----------------------------------------------------------------------------
//HAL_StatusTypeDef io_nand_erase_block(NAND_AddressTypeDef *pAddress)
HAL_StatusTypeDef io_nand_erase_block(uint32_t adr)
{
NAND_AddressTypeDef Address = io_uint32_to_flash_adr(adr);

	if (nandPort->State == HAL_NAND_STATE_BUSY) {

		return HAL_BUSY;

	} else if (nandPort->State == HAL_NAND_STATE_READY) {

		__HAL_LOCK(nandPort);
		nandPort->State = HAL_NAND_STATE_BUSY;

		uint32_t deviceaddress = devAdr;
		uint32_t nandaddress = ARRAY_ADDRESS(&Address, nandPort);


		if (dbg > logOn)
					Report(1, "[%s] nand_adr:0x%X page:%lu plane:%lu block:%lu\r\n",
							  __func__, nandaddress, Address.Page, Address.Plane, Address.Block);

#ifdef SET_NAND_CMD
    bool tflag = true;
    memset(tmpBuf, 0, sizeof(tmpBuf));
    tmpLen = 0;
#else
    bool tflag = false;
#endif

		/* Send Erase block command sequence */
		*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_ERASE0;
		__DSB();
		if (tflag) tmpBuf[tmpLen++] = NAND_CMD_ERASE0;
		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
		__DSB();
		if (tflag) tmpBuf[tmpLen++] = ADDR_1ST_CYCLE(nandaddress);
		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
		__DSB();
		if (tflag) tmpBuf[tmpLen++] = ADDR_2ND_CYCLE(nandaddress);
		*(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
		__DSB();
		if (tflag) tmpBuf[tmpLen++] = ADDR_3RD_CYCLE(nandaddress);
		*(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_ERASE1;
		__DSB();
		if (tflag) tmpBuf[tmpLen++] = NAND_CMD_ERASE1;

		nandPort->State = HAL_NAND_STATE_READY;
		__HAL_UNLOCK(nandPort);

		if (tflag & dbg) tmpPrint(__func__, tmpBuf, tmpLen);

	} else {

		return HAL_ERROR;

	}

	return HAL_OK;
}
//-----------------------------------------------------------------------------------------
uint32_t io_flash_adr_to_uint32(NAND_AddressTypeDef *adr)
{
	return  ((adr->Plane * chipConf.PlaneSize) + (adr->Block * chipConf.BlockSize) + adr->Page);
}
//-----------------------------------------------------------------------------------------
NAND_AddressTypeDef io_uint32_to_flash_adr(uint32_t adr)
{
NAND_AddressTypeDef a;

	a.Plane = adr / chipConf.PlaneSize;
	a.Block = (adr - a.Plane * chipConf.PlaneSize) / chipConf.BlockSize;
	a.Page  = adr - (a.Plane * chipConf.PlaneSize) - (a.Block * chipConf.BlockSize);

	return a;
}
//-----------------------------------------------------------------------------------------
uint32_t io_nand_read(uint32_t adr, uint8_t *buffer, uint32_t size, uint32_t offset)
{

	if (io_nand_read_8b(adr, buffer, size, offset) != HAL_OK) devError |= devNAND;

    return 0;
}
//-----------------------------------------------------------------------------------------
uint32_t io_nand_write(uint32_t adr, uint8_t *buffer, uint32_t size, uint32_t offset)
{

	if (io_nand_write_8b(adr, buffer, size, offset) != HAL_OK) devError |= devNAND;

	return 0;
}
//-----------------------------------------------------------------------------------------
void io_nand_block_erase(uint32_t adr)
{
//NAND_AddressTypeDef nans = io_uint32_to_flash_adr(addr);

//	if (io_nand_erase_block(&nans) != HAL_OK) devError |= devNAND;

	if (io_nand_erase_block(adr) != HAL_OK) devError |= devNAND;
}
//-----------------------------------------------------------------------------------------
unsigned char io_nand_get_status()
{
unsigned char ret = 2;//STA_NODISK;


	switch ((unsigned char)HAL_NAND_GetState(nandPort)) {
		case HAL_NAND_STATE_RESET://     = 0x00U,  //NAND not yet initialized or disabled
			ret = 1;
		break;
		case HAL_NAND_STATE_READY://     = 0x01U,  //NAND initialized and ready for use
			ret = 0;
		break;
		//case HAL_NAND_STATE_BUSY://      = 0x02U,  //NAND internal process is ongoing
		//case HAL_NAND_STATE_ERROR://     = 0x03U   //NAND error state
		//break;
	}
//RES_OK = 0,		/* 0: Successful */
//RES_ERROR,		/* 1: R/W Error */
//RES_WRPRT,		/* 2: Write Protected */
//RES_NOTRDY,		/* 3: Not Ready */
//RES_PARERR		/* 4: Invalid Parameter */
//#define STA_NOINIT		0x01	/* Drive not initialized */
//#define STA_NODISK		0x02	/* No medium in the drive */
//#define STA_PROTECT		0x04	/* Write protected */

	return ret;
}
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------
