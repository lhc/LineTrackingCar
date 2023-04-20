/**
 * @file    w25qxx.h
 * @brief   Library to read and write on memorys w25qxx
 *          In Page,Sector and block read/write functions, can put 0 to read maximum bytes
 */

//==============================================================================
// Define to prevent recursive inclusion
//==============================================================================

#ifndef _W25QXX_H
#define _W25QXX_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// Includes
//==============================================================================

#include "setup_hw.h"
#include <stdbool.h>

//==============================================================================
//Exported constants
//==============================================================================

//==============================================================================
// Exported macro
//==============================================================================

//==============================================================================
// Exported types
//==============================================================================

typedef enum
{
	eW25Q10 = 0x4011,
	eW25Q20 = 0x4012,
	eW25Q40 = 0x4013,
	eW25Q80 = 0x4014,
	eW25Q16 = 0x4015,
	eW25Q32 = 0x4016,
	eW25Q64 = 0x4017,
	eW25Q128 = 0x4018,
	eW25Q256 = 0x4019,
	eW25Q512 = 0x401A,
} W25QXX_ID_e;

typedef struct
{
	W25QXX_ID_e ID;
	uint8_t UniqID[8];
	uint16_t PageSize;
	uint32_t PageCount;
	uint32_t SectorSize;
	uint32_t SectorCount;
	uint32_t BlockSize;
	uint32_t BlockCount;
	uint32_t CapacityInKiloByte;
	uint8_t StatusRegister1;
	uint8_t StatusRegister2;
	uint8_t StatusRegister3;
	uint8_t Lock;
} W25QXX_Get_t;

typedef struct
{
  SPI_HandleTypeDef *spi;
  GPIO_TypeDef * Gpio;
  uint32_t Pin;
} W25QXX_Set_t;

typedef struct
{
  W25QXX_Set_t Set;
  W25QXX_Get_t Get;
} W25QXX_Handler_t;

//==============================================================================
// Exported variables
//==============================================================================

//==============================================================================
// Exported functions prototypes
//==============================================================================

void W25QXX_Attach( W25QXX_Handler_t *pHandler, GPIO_TypeDef * GpioCS, uint32_t PinCS, SPI_HandleTypeDef *hspi );

int W25QXX_Init( W25QXX_Handler_t *pHandler);

void W25QXX_EraseChip( W25QXX_Handler_t *pHandler );
void W25QXX_EraseSector( W25QXX_Handler_t *pHandler, uint32_t SectorAddr );
void W25QXX_EraseBlock( W25QXX_Handler_t *pHandler, uint32_t BlockAddr );

uint32_t W25QXX_PageToSector( W25QXX_Handler_t *pHandler, uint32_t PageAddress );
uint32_t W25QXX_PageToBlock( W25QXX_Handler_t *pHandler, uint32_t PageAddress );
uint32_t W25QXX_SectorToBlock( W25QXX_Handler_t *pHandler, uint32_t SectorAddress );
uint32_t W25QXX_SectorToPage( W25QXX_Handler_t *pHandler, uint32_t SectorAddress );
uint32_t W25QXX_BlockToPage( W25QXX_Handler_t *pHandler, uint32_t BlockAddress );

bool W25QXX_IsEmptyPage( W25QXX_Handler_t *pHandler, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize );
bool W25QXX_IsEmptySector( W25QXX_Handler_t *pHandler, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize );
bool W25QXX_IsEmptyBlock( W25QXX_Handler_t *pHandler, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize );

void W25QXX_WriteByte( W25QXX_Handler_t *pHandler, uint8_t pBuffer, uint32_t Bytes_Address );
void W25QXX_WritePage( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize );
void W25QXX_WriteSector( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize );
void W25QXX_WriteBlock( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_BlockSize );

void W25QXX_ReadByte( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Bytes_Address );
void W25QXX_ReadBytes( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead );
void W25QXX_ReadPage( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize );
void W25QXX_ReadSector( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize );
void W25QXX_ReadBlock( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize );

//==============================================================================
// Exported functions
//==============================================================================

#ifdef __cplusplus
}
#endif

#endif

