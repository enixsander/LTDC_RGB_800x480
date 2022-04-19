#ifndef __QUAD_SPI_H
#define __QUAD_SPI_H

#include "main.h"

uint8_t QspiReadData(uint32_t address, uint32_t size, uint8_t* pData);
uint8_t QspiWriteData(uint32_t address, uint32_t size, uint8_t* pData);
uint8_t QspiWaitStatusReg1_BUSY(uint16_t timeout);
uint8_t QspiReadID(void);
uint8_t QSPI_WriteEnable(void);
uint8_t BSP_QSPI_Erase_Sector(uint32_t BlockAddress);
uint8_t BSP_QSPI_Erase_Block_64KB(uint32_t BlockAddress, uint16_t countBlock);

#endif // __QUAD_SPI_H 
