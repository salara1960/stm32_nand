#ifndef __IO_NAND_H
#define __IO_NAND_H

//---------------------------------------------------------------------------------

#include "main.h"

#define SET_NAND_CMD

//---------------------------------------------------------------------------------
//extern s_chipConf chipConf;
//---------------------------------------------------------------------------------
extern uint8_t Report(const uint8_t addTime, const char *fmt, ...);

//uint32_t nand_PageToBlock(const uint32_t page);
//uint32_t nand_BlockToPage(const uint32_t blk);

void io_nand_init(NAND_HandleTypeDef *hnand);
uint32_t io_nand_get_page_size(void);
uint32_t io_nand_get_block_number(void);
uint32_t io_nand_get_block_size(void);
uint32_t io_nand_get_plane_number(void);
uint32_t io_nand_get_plane_size(void);
HAL_StatusTypeDef NAND_Read_ID(NAND_HandleTypeDef *hnand, NAND_IDsTypeDef *pNAND_ID);
uint32_t io_nand_read_8b (uint32_t adr, uint8_t *pBuffer, uint32_t size, uint32_t offset);
uint32_t io_nand_write_8b(uint32_t adr, uint8_t *pBuffer, uint32_t size, uint32_t offset);
HAL_StatusTypeDef io_nand_erase_block(uint32_t adr);
uint32_t io_flash_adr_to_uint32(NAND_AddressTypeDef *adr);
NAND_AddressTypeDef io_uint32_to_flash_adr(uint32_t adr);
uint32_t io_nand_read(uint32_t addr, uint8_t *buffer, uint32_t size, uint32_t offset);
uint32_t io_nand_write(uint32_t addr, uint8_t *buffer, uint32_t size, uint32_t offset);
void io_nand_block_erase(uint32_t adr);
unsigned char io_nand_get_status();
//---------------------------------------------------------------------------------


#endif
