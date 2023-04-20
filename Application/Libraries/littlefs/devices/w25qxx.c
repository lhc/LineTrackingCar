/**
 * @file    w25qxx.c
 * @brief   Library to read and write on memorys w25qxx
 */

//==============================================================================
// Includes
//==============================================================================

#include "w25qxx.h"
#include "Task/serial.h"
#include <string.h>

//==============================================================================
// Private definitions
//==============================================================================

#if !defined( HAL_SPI_MODULE_ENABLED )
#error "SPI HAL not enable"
#endif

#define W25QXX_USE_FREERTOS         ( 1 )
#define W25QXX_DUMMY_BYTE           ( 0xA5 )
#define W25QXX_DEBUG_ENABLE         ( 0 )
//==============================================================================
// Private macro
//==============================================================================

#if ( W25QXX_DEBUG_ENABLE > 0 )

#define W25QXX_PRINT(fmt, ...)   Serial_Message(fmt, ##__VA_ARGS__)
#define W25QXX_INFO(fmt, ...)    Serial_Message(" %d [W25QX INFO] "fmt, xTaskGetTickCount(), ##__VA_ARGS__)
#define W25QXX_WARN(fmt, ...)    Serial_Message(" %d [W25QX WARN] "fmt, xTaskGetTickCount(), ##__VA_ARGS__)
#define W25QXX_ERRO(fmt, ...)    Serial_Message(" %d [W25QX ERRO] "fmt, xTaskGetTickCount(), ##__VA_ARGS__)

#else

#define W25QXX_PRINT(fmt, ...)
#define W25QXX_INFO(fmt, ...)
#define W25QXX_WARN(fmt, ...)
#define W25QXX_ERRO(fmt, ...)

#endif

#if ( W25QXX_USE_FREERTOS == 1 )
#define W25QXX_Delay(delay)  // vTaskDelay(delay)
#else
#define W25QXX_Delay(delay)   HAL_Delay(delay)
#endif

//==============================================================================
// Private typedef
//==============================================================================

//==============================================================================
// Extern variables
//==============================================================================

//==============================================================================
// Private variables
//==============================================================================

//==============================================================================
// Private function prototypes
//==============================================================================

static int W25QXX_Error( int Error );
static void W25QXX_WaitForWriteEnd(  W25QXX_Handler_t *pHandler );
static uint8_t W25QXX_Spi( W25QXX_Handler_t *pHandler, uint8_t Data );
static uint32_t W25QXX_ReadID(  W25QXX_Handler_t *pHandler );
static void W25QXX_ReadUniqID(  W25QXX_Handler_t *pHandler );
static void W25QXX_WriteEnable(  W25QXX_Handler_t *pHandler );
static void W25QXX_WriteDisable(  W25QXX_Handler_t *pHandler );
static uint8_t W25QXX_ReadStatusRegister(  W25QXX_Handler_t *pHandler, uint8_t SelectStatusRegister_1_2_3 );
static void W25QXX_WriteStatusRegister(  W25QXX_Handler_t *pHandler, uint8_t SelectStatusRegister_1_2_3, uint8_t Data );

//==============================================================================
// Private functions
//==============================================================================

/**
 * @brief Function to monitor the error occurred
 * @param Error
 * @return
 */
static int W25QXX_Error( int Error )
{
  if( Error != 0 )
  {
    for(;;);
  }

  return Error;
}

/**
 * @brief
 * @param pHandler
 * @param Data
 * @return
 */
static uint8_t W25QXX_Spi( W25QXX_Handler_t *pHandler, uint8_t Data )
{
  uint8_t ret;
  HAL_SPI_TransmitReceive( pHandler->Set.spi, &Data, &ret, 1, 10 );
  return ret;
}

/**
 * @brief
 * @param pHandler
 * @return
 */
static uint32_t W25QXX_ReadID(  W25QXX_Handler_t *pHandler )
{
  uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );

  W25QXX_Spi( pHandler, 0x9F );
  Temp0 = W25QXX_Spi( pHandler, W25QXX_DUMMY_BYTE );
  Temp1 = W25QXX_Spi( pHandler, W25QXX_DUMMY_BYTE );
  Temp2 = W25QXX_Spi( pHandler, W25QXX_DUMMY_BYTE );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

  Temp = ( Temp0 << 16 ) | ( Temp1 << 8 ) | Temp2;
  return Temp;
}

/**
 * @brief
 * @param pHandler
 */
static void W25QXX_ReadUniqID(  W25QXX_Handler_t *pHandler )
{
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0x4B );

  for( uint8_t i = 0; i < 4; i++ )
  {
    W25QXX_Spi( pHandler, W25QXX_DUMMY_BYTE );
  }

  for( uint8_t i = 0; i < 8; i++ )
  {
    pHandler->Get.UniqID[ i ] =W25QXX_Spi( pHandler, W25QXX_DUMMY_BYTE );
  }

  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );
}

/**
 * @brief
 * @param pHandler
 */
static void W25QXX_WriteEnable(  W25QXX_Handler_t *pHandler )
{
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0x06 );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

  W25QXX_Delay( 1 );
}

/**
 * @brief
 * @param pHandler
 */
static void W25QXX_WriteDisable( W25QXX_Handler_t *pHandler )
{
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0x04 );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

  W25QXX_Delay( 1 );
}

/**
 * @brief
 * @param pHandler
 * @param SelectStatusRegister_1_2_3
 * @return
 */
static uint8_t W25QXX_ReadStatusRegister( W25QXX_Handler_t *pHandler, uint8_t SelectStatusRegister_1_2_3 )
{
  uint8_t status = 0;
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );

  if( SelectStatusRegister_1_2_3 == 1 )
  {
    W25QXX_Spi( pHandler, 0x05 );
    status = W25QXX_Spi( pHandler, W25QXX_DUMMY_BYTE );
    pHandler->Get.StatusRegister1 = status;
  }
  else if( SelectStatusRegister_1_2_3 == 2 )
  {
    W25QXX_Spi( pHandler, 0x35 );
    status = W25QXX_Spi( pHandler, W25QXX_DUMMY_BYTE );
    pHandler->Get.StatusRegister2 = status;
  }
  else
  {
    W25QXX_Spi( pHandler, 0x15 );
    status = W25QXX_Spi( pHandler, W25QXX_DUMMY_BYTE );
    pHandler->Get.StatusRegister3 = status;
  }

  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );
  return status;
}

/**
 * @brief
 * @param pHandler
 * @param SelectStatusRegister_1_2_3
 * @param Data
 */
static void W25QXX_WriteStatusRegister(  W25QXX_Handler_t *pHandler, uint8_t SelectStatusRegister_1_2_3, uint8_t Data )
{
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );

  if( SelectStatusRegister_1_2_3 == 1 )
  {
    W25QXX_Spi( pHandler, 0x01 );
    pHandler->Get.StatusRegister1 = Data;
  }
  else if( SelectStatusRegister_1_2_3 == 2 )
  {
    W25QXX_Spi( pHandler, 0x31 );
    pHandler->Get.StatusRegister2 = Data;
  }
  else
  {
    W25QXX_Spi( pHandler, 0x11 );
    pHandler->Get.StatusRegister3 = Data;
  }

  W25QXX_Spi( pHandler, Data );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );
}

/**
 * @brief
 * @param pHandler
 */
static void W25QXX_WaitForWriteEnd( W25QXX_Handler_t *pHandler )
{
  W25QXX_Delay( 1 );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0x05 );

  do
  {
    pHandler->Get.StatusRegister1 =W25QXX_Spi( pHandler, W25QXX_DUMMY_BYTE );
    W25QXX_Delay( 1 );
  } while( ( pHandler->Get.StatusRegister1 & 0x01 ) == 0x01 );

  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );
}

//==============================================================================
// Exported functions
//==============================================================================

/**
 * @brief
 * @param pHandler
 * @param GPIOx
 * @param GPIO_Pin
 * @param hspi
 */
void W25QXX_Attach( W25QXX_Handler_t *pHandler, GPIO_TypeDef * GpioCS, uint32_t PinCS, SPI_HandleTypeDef *hspi )
{
  if( hspi != NULL && GpioCS != NULL )
  {
    pHandler->Set.spi = hspi;
    pHandler->Set.Gpio = GpioCS;
    pHandler->Set.Pin = PinCS;

    HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );
  }
}

/**
 * @brief
 * @param pHandler
 * @return
 */
int W25QXX_Init( W25QXX_Handler_t *pHandler )
{
  uint32_t id;
  int err;

  err = 0;
  pHandler->Get.Lock = 1;

  W25QXX_Delay( 100 );

  W25QXX_INFO( "w25qxx Init Begin... \r\n" );

  id = W25QXX_ReadID( pHandler );

  W25QXX_INFO( "w25qxx ID:0x%X \r\n", id );

  switch( id & 0x0000FFFF )
  {
    case eW25Q512:
    {
      pHandler->Get.ID = eW25Q512;
      pHandler->Get.BlockCount = 1024;
      W25QXX_INFO( "w25qxx Chip: w25q512 \r\n" );
      break;
    }
    case eW25Q256:
    {
      pHandler->Get.ID = eW25Q256;
      pHandler->Get.BlockCount = 512;
      W25QXX_INFO( "w25qxx Chip: w25q256 \r\n" );
      break;
    }
    case eW25Q128:
    {
      pHandler->Get.ID = eW25Q128;
      pHandler->Get.BlockCount = 256;
      W25QXX_INFO( "w25qxx Chip: w25q128 \r\n" );
      break;
    }
    case eW25Q64:
    {
      pHandler->Get.ID = eW25Q64;
      pHandler->Get.BlockCount = 128;
      W25QXX_INFO( "w25qxx Chip: w25q64 \r\n" );
      break;
    }
    case eW25Q32:
    {
      pHandler->Get.ID = eW25Q32;
      pHandler->Get.BlockCount = 64;
      W25QXX_INFO( "w25qxx Chip: w25q32 \r\n" );
      break;
    }
    case eW25Q16:
    {
      pHandler->Get.ID = eW25Q16;
      pHandler->Get.BlockCount = 32;
      W25QXX_INFO( "w25qxx Chip: w25q16 \r\n" );
      break;
    }
    case eW25Q80:
    {
      pHandler->Get.ID = eW25Q80;
      pHandler->Get.BlockCount = 16;
      W25QXX_INFO( "w25qxx Chip: w25q80 \r\n" );
      break;
    }
    case eW25Q40:
    {
      pHandler->Get.ID = eW25Q40;
      pHandler->Get.BlockCount = 8;
      W25QXX_INFO( "w25qxx Chip: w25q40" );
      break;
    }
    case eW25Q20:
    {
      pHandler->Get.ID = eW25Q20;
      pHandler->Get.BlockCount = 4;
      W25QXX_INFO( "w25qxx Chip: w25q20 \r\n" );
      break;
    }
    case eW25Q10:
    {
      pHandler->Get.ID = eW25Q10;
      pHandler->Get.BlockCount = 2;
      W25QXX_INFO( "w25qxx Chip: w25q10 \r\n" );
      break;
    }
    default:
    {
      W25QXX_INFO( "w25qxx Unknown ID \r\n" );
      pHandler->Get.Lock = 0;
      err = 1;
    }
  }

  if( err == 0 )
  {
    pHandler->Get.PageSize = 256;
    pHandler->Get.SectorSize = 0x1000;
    pHandler->Get.SectorCount = pHandler->Get.BlockCount * 16;
    pHandler->Get.PageCount = ( pHandler->Get.SectorCount * pHandler->Get.SectorSize ) / pHandler->Get.PageSize;
    pHandler->Get.BlockSize = pHandler->Get.SectorSize * 16;
    pHandler->Get.CapacityInKiloByte = ( pHandler->Get.SectorCount * pHandler->Get.SectorSize ) / 1024;

    W25QXX_ReadUniqID( pHandler );
    W25QXX_ReadStatusRegister( pHandler, 1 );
    W25QXX_ReadStatusRegister( pHandler, 2 );
    W25QXX_ReadStatusRegister( pHandler, 3 );

    W25QXX_INFO( "w25qxx Page Size: %d Bytes \r\n", pHandler->Get.PageSize );
    W25QXX_INFO( "w25qxx Page Count: %d \r\n", pHandler->Get.PageCount );
    W25QXX_INFO( "w25qxx Sector Size: %d Bytes \r\n", pHandler->Get.SectorSize );
    W25QXX_INFO( "w25qxx Sector Count: %d \r\n", pHandler->Get.SectorCount );
    W25QXX_INFO( "w25qxx Block Size: %d Bytes \r\n", pHandler->Get.BlockSize );
    W25QXX_INFO( "w25qxx Block Count: %d \r\n", pHandler->Get.BlockCount );
    W25QXX_INFO( "w25qxx Capacity: %d KiloBytes \r\n", pHandler->Get.CapacityInKiloByte );
    W25QXX_INFO( "w25qxx Init Done \r\n" );
  }

  pHandler->Get.Lock = 0;

  return W25QXX_Error( err);
}

/**
 * @brief
 * @param pHandler
 */
void W25QXX_EraseChip(  W25QXX_Handler_t *pHandler )
{
  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

#if ( W25QXX_DEBUG_ENABLED > 0 )
  uint32_t StartTime = HAL_GetTick();
  W25QXX_INFO( "w25qxx EraseChip Begin... \r\n" );
#endif

  W25QXX_WriteEnable( pHandler );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0xC7 );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );
  W25QXX_WaitForWriteEnd( pHandler );

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx EraseBlock done after %d ms! \r\n", HAL_GetTick() - StartTime );
#endif

  W25QXX_Delay( 10 );
  pHandler->Get.Lock = 0;
}

/**
 * @brief
 * @param pHandler
 * @param SectorAddr
 */
void W25QXX_EraseSector( W25QXX_Handler_t *pHandler, uint32_t SectorAddr )
{
  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

#if ( W25QXX_DEBUG_ENABLED > 0 )
  uint32_t StartTime = HAL_GetTick();
  W25QXX_INFO( "w25qxx EraseSector %d Begin... \r\n", SectorAddr );
#endif

  W25QXX_WaitForWriteEnd( pHandler );
  SectorAddr = SectorAddr * pHandler->Get.SectorSize;
  W25QXX_WriteEnable( pHandler );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0x20 );

  if( pHandler->Get.ID >= eW25Q256 )
  {
    W25QXX_Spi( pHandler, ( SectorAddr & 0xFF000000 ) >> 24 );
  }

  W25QXX_Spi( pHandler, ( SectorAddr & 0xFF0000 ) >> 16 );
  W25QXX_Spi( pHandler, ( SectorAddr & 0xFF00 ) >> 8 );
  W25QXX_Spi( pHandler, SectorAddr & 0xFF );

  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

  W25QXX_WaitForWriteEnd( pHandler );

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx EraseSector done after %d ms \r\n", HAL_GetTick() - StartTime );
#endif

  W25QXX_Delay( 1 );
  pHandler->Get.Lock = 0;
}

/**
 * @brief
 * @param pHandler
 * @param BlockAddr
 */
void W25QXX_EraseBlock( W25QXX_Handler_t *pHandler, uint32_t BlockAddr )
{
  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx EraseBlock %d Begin... \r\n", BlockAddr );
  uint32_t StartTime = HAL_GetTick();
#endif

  W25QXX_WaitForWriteEnd( pHandler );
  BlockAddr = BlockAddr * pHandler->Get.SectorSize * 16;
  W25QXX_WriteEnable( pHandler );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0xD8 );

  if( pHandler->Get.ID >= eW25Q256 )
  {
    W25QXX_Spi( pHandler, ( BlockAddr & 0xFF000000 ) >> 24 );
  }

  W25QXX_Spi( pHandler, ( BlockAddr & 0xFF0000 ) >> 16 );
  W25QXX_Spi( pHandler, ( BlockAddr & 0xFF00 ) >> 8 );
  W25QXX_Spi( pHandler, BlockAddr & 0xFF );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );
  W25QXX_WaitForWriteEnd( pHandler );

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx EraseBlock done after %d ms \r\n", HAL_GetTick() - StartTime );
#endif

  W25QXX_Delay( 1 );
  pHandler->Get.Lock = 0;
}

/**
 * @brief
 * @param pHandler
 * @param PageAddress
 * @return
 */
uint32_t W25QXX_PageToSector( W25QXX_Handler_t *pHandler, uint32_t PageAddress )
{
  return ( ( PageAddress * pHandler->Get.PageSize ) / pHandler->Get.SectorSize );
}

/**
 * @brief
 * @param pHandler
 * @param PageAddress
 * @return
 */
uint32_t W25QXX_PageToBlock( W25QXX_Handler_t *pHandler, uint32_t PageAddress )
{
  return ( ( PageAddress * pHandler->Get.PageSize ) / pHandler->Get.BlockSize );
}

/**
 * @brief
 * @param pHandler
 * @param SectorAddress
 * @return
 */
uint32_t W25QXX_SectorToBlock( W25QXX_Handler_t *pHandler, uint32_t SectorAddress )
{
  return ( ( SectorAddress * pHandler->Get.SectorSize ) / pHandler->Get.BlockSize );
}

/**
 * @brief
 * @param pHandler
 * @param SectorAddress
 * @return
 */
uint32_t W25QXX_SectorToPage( W25QXX_Handler_t *pHandler, uint32_t SectorAddress )
{
  return ( SectorAddress * pHandler->Get.SectorSize ) / pHandler->Get.PageSize;
}

/**
 * @brief
 * @param pHandler
 * @param BlockAddress
 * @return
 */
uint32_t W25QXX_BlockToPage( W25QXX_Handler_t *pHandler, uint32_t BlockAddress )
{
  return ( BlockAddress * pHandler->Get.BlockSize ) / pHandler->Get.PageSize;
}

/**
 * @brief
 * @param pHandler
 * @param Page_Address
 * @param OffsetInByte
 * @param NumByteToCheck_up_to_PageSize
 * @return
 */
bool W25QXX_IsEmptyPage( W25QXX_Handler_t *pHandler, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_PageSize )
{
  uint8_t pBuffer[ 32 ];
  uint32_t WorkAddress;
  uint32_t i;

  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

  if( ( ( NumByteToCheck_up_to_PageSize + OffsetInByte ) > pHandler->Get.PageSize ) || ( NumByteToCheck_up_to_PageSize == 0 ) )
  {
    NumByteToCheck_up_to_PageSize = pHandler->Get.PageSize - OffsetInByte;
  }

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx CheckPage:%d, Offset:%d, Bytes:%d begin... \r\n", Page_Address, OffsetInByte, NumByteToCheck_up_to_PageSize );
  uint32_t StartTime = HAL_GetTick();
#endif

  for( i = OffsetInByte; i < pHandler->Get.PageSize; i += sizeof( pBuffer ) )
  {
    HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
    WorkAddress = ( i + Page_Address * pHandler->Get.PageSize );
    W25QXX_Spi( pHandler, 0x0B );

    if( pHandler->Get.ID >= eW25Q256 )
    {
      W25QXX_Spi( pHandler, ( WorkAddress & 0xFF000000 ) >> 24 );
    }

    W25QXX_Spi( pHandler, ( WorkAddress & 0xFF0000 ) >> 16 );
    W25QXX_Spi( pHandler, ( WorkAddress & 0xFF00 ) >> 8 );
    W25QXX_Spi( pHandler, WorkAddress & 0xFF );
    W25QXX_Spi( pHandler, 0 );

    HAL_SPI_Receive( pHandler->Set.spi, pBuffer, sizeof( pBuffer ), 100 );
    HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

    for( uint8_t x = 0; x < sizeof( pBuffer ); x++ )
    {
      if( pBuffer[ x ] != 0xFF )
      {
        goto NOT_EMPTY;
      }
    }
  }

  if( ( pHandler->Get.PageSize + OffsetInByte ) % sizeof( pBuffer ) != 0 )
  {
    i -= sizeof( pBuffer );

    for( ; i < pHandler->Get.PageSize; i++ )
    {
      HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
      WorkAddress = ( i + Page_Address * pHandler->Get.PageSize );
      W25QXX_Spi( pHandler, 0x0B );

      if( pHandler->Get.ID >= eW25Q256 )
      {
        W25QXX_Spi( pHandler, ( WorkAddress & 0xFF000000 ) >> 24 );
      }

      W25QXX_Spi( pHandler, ( WorkAddress & 0xFF0000 ) >> 16 );
      W25QXX_Spi( pHandler, ( WorkAddress & 0xFF00 ) >> 8 );
      W25QXX_Spi( pHandler, WorkAddress & 0xFF );
      W25QXX_Spi( pHandler, 0 );

      HAL_SPI_Receive( pHandler->Set.spi, pBuffer, 1, 100 );
      HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

      if( pBuffer[ 0 ] != 0xFF )
      {
        goto NOT_EMPTY;
      }
    }
  }

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx CheckPage is Empty in %d ms \r\n", HAL_GetTick() - StartTime );
#endif

  pHandler->Get.Lock = 0;
  return true;

  NOT_EMPTY:

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx CheckPage is Not Empty in %d ms \r\n", HAL_GetTick() - StartTime );
#endif

  pHandler->Get.Lock = 0;
  return false;
}

/**
 * @brief
 * @param pHandler
 * @param Sector_Address
 * @param OffsetInByte
 * @param NumByteToCheck_up_to_SectorSize
 * @return
 */
bool W25QXX_IsEmptySector( W25QXX_Handler_t *pHandler, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_SectorSize )
{
  uint8_t pBuffer[ 32 ];
  uint32_t WorkAddress;
  uint32_t i;

  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

  if( ( NumByteToCheck_up_to_SectorSize > pHandler->Get.SectorSize ) || ( NumByteToCheck_up_to_SectorSize == 0 ) )
  {
    NumByteToCheck_up_to_SectorSize = pHandler->Get.SectorSize;
  }

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx CheckSector:%d, Offset:%d, Bytes:%d begin... \r\n", Sector_Address, OffsetInByte, NumByteToCheck_up_to_SectorSize );
  uint32_t StartTime = HAL_GetTick();
#endif

  for( i = OffsetInByte; i < pHandler->Get.SectorSize; i += sizeof( pBuffer ) )
  {
    HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
    WorkAddress = ( i + Sector_Address * pHandler->Get.SectorSize );
    W25QXX_Spi( pHandler, 0x0B );

    if( pHandler->Get.ID >= eW25Q256 )
    {
      W25QXX_Spi( pHandler, ( WorkAddress & 0xFF000000 ) >> 24 );
    }

    W25QXX_Spi( pHandler, ( WorkAddress & 0xFF0000 ) >> 16 );
    W25QXX_Spi( pHandler, ( WorkAddress & 0xFF00 ) >> 8 );
    W25QXX_Spi( pHandler, WorkAddress & 0xFF );
    W25QXX_Spi( pHandler, 0 );

    HAL_SPI_Receive( pHandler->Set.spi, pBuffer, sizeof( pBuffer ), 100 );
    HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

    for( uint8_t x = 0; x < sizeof( pBuffer ); x++ )
    {
      if( pBuffer[ x ] != 0xFF )
      {
        goto NOT_EMPTY;
      }
    }
  }

  if( ( pHandler->Get.SectorSize + OffsetInByte ) % sizeof( pBuffer ) != 0 )
  {
    i -= sizeof( pBuffer );

    for( ; i < pHandler->Get.SectorSize; i++ )
    {
      HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
      WorkAddress = ( i + Sector_Address * pHandler->Get.SectorSize );
      W25QXX_Spi( pHandler, 0x0B );

      if( pHandler->Get.ID >= eW25Q256 )
      {
        W25QXX_Spi( pHandler, ( WorkAddress & 0xFF000000 ) >> 24 );
      }

      W25QXX_Spi( pHandler, ( WorkAddress & 0xFF0000 ) >> 16 );
      W25QXX_Spi( pHandler, ( WorkAddress & 0xFF00 ) >> 8 );
      W25QXX_Spi( pHandler, WorkAddress & 0xFF );
      W25QXX_Spi( pHandler, 0 );

      HAL_SPI_Receive( pHandler->Set.spi, pBuffer, 1, 100 );
      HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

      if( pBuffer[ 0 ] != 0xFF )
      {
        goto NOT_EMPTY;
      }
    }
  }
#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx CheckSector is Empty in %d ms \r\n", HAL_GetTick() - StartTime );
#endif

  pHandler->Get.Lock = 0;
  return true;

  NOT_EMPTY:

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx CheckSector is Not Empty in %d ms \r\n", HAL_GetTick() - StartTime );
#endif

  pHandler->Get.Lock = 0;
  return false;
}

/**
 * @brief
 * @param pHandler
 * @param Block_Address
 * @param OffsetInByte
 * @param NumByteToCheck_up_to_BlockSize
 * @return
 */
bool W25QXX_IsEmptyBlock( W25QXX_Handler_t *pHandler, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToCheck_up_to_BlockSize )
{
  uint8_t pBuffer[ 32 ];
  uint32_t WorkAddress;
  uint32_t i;

  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

  if( ( NumByteToCheck_up_to_BlockSize > pHandler->Get.BlockSize ) || ( NumByteToCheck_up_to_BlockSize == 0 ) )
  {
    NumByteToCheck_up_to_BlockSize = pHandler->Get.BlockSize;
  }

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx CheckBlock:%d, Offset:%d, Bytes:%d begin... \r\n", Block_Address, OffsetInByte, NumByteToCheck_up_to_BlockSize );
  uint32_t StartTime = HAL_GetTick();
#endif

  for( i = OffsetInByte; i < pHandler->Get.BlockSize; i += sizeof( pBuffer ) )
  {
    HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
    WorkAddress = ( i + Block_Address * pHandler->Get.BlockSize );
    W25QXX_Spi( pHandler, 0x0B );

    if( pHandler->Get.ID >= eW25Q256 )
    {
      W25QXX_Spi( pHandler, ( WorkAddress & 0xFF000000 ) >> 24 );
    }

    W25QXX_Spi( pHandler, ( WorkAddress & 0xFF0000 ) >> 16 );
    W25QXX_Spi( pHandler, ( WorkAddress & 0xFF00 ) >> 8 );
    W25QXX_Spi( pHandler, WorkAddress & 0xFF );
    W25QXX_Spi( pHandler, 0 );

    HAL_SPI_Receive( pHandler->Set.spi, pBuffer, sizeof( pBuffer ), 100 );
    HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

    for( uint8_t x = 0; x < sizeof( pBuffer ); x++ )
    {
      if( pBuffer[ x ] != 0xFF )
      {
        goto NOT_EMPTY;
      }
    }
  }

  if( ( pHandler->Get.BlockSize + OffsetInByte ) % sizeof( pBuffer ) != 0 )
  {
    i -= sizeof( pBuffer );

    for( ; i < pHandler->Get.BlockSize; i++ )
    {
      HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
      WorkAddress = ( i + Block_Address * pHandler->Get.BlockSize );
      W25QXX_Spi( pHandler, 0x0B );

      if( pHandler->Get.ID >= eW25Q256 )
      {
        W25QXX_Spi( pHandler, ( WorkAddress & 0xFF000000 ) >> 24 );
      }

      W25QXX_Spi( pHandler, ( WorkAddress & 0xFF0000 ) >> 16 );
      W25QXX_Spi( pHandler, ( WorkAddress & 0xFF00 ) >> 8 );
      W25QXX_Spi( pHandler, WorkAddress & 0xFF );
      W25QXX_Spi( pHandler, 0 );

      HAL_SPI_Receive( pHandler->Set.spi, pBuffer, 1, 100 );
      HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

      if( pBuffer[ 0 ] != 0xFF )
      {
        goto NOT_EMPTY;
      }
    }
  }
#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx CheckBlock is Empty in %d ms \r\n", HAL_GetTick() - StartTime );
#endif

  pHandler->Get.Lock = 0;
  return true;
  NOT_EMPTY:

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx CheckBlock is Not Empty in %d ms \r\n", HAL_GetTick() - StartTime );
#endif

  pHandler->Get.Lock = 0;

  return false;
}

/**
 * @brief
 * @param pHandler
 * @param pBuffer
 * @param WriteAddr_inBytes
 */
void W25QXX_WriteByte( W25QXX_Handler_t *pHandler, uint8_t pBuffer, uint32_t WriteAddr_inBytes )
{
  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

#if ( W25QXX_DEBUG_ENABLED > 0 )
  uint32_t StartTime = HAL_GetTick();
  W25QXX_INFO( "w25qxx WriteByte 0x%02X at address %d begin... \r\n", pBuffer, WriteAddr_inBytes );
#endif

  W25QXX_WaitForWriteEnd( pHandler );
  W25QXX_WriteEnable( pHandler );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0x02 );

  if( pHandler->Get.ID >= eW25Q256 )
  {
    W25QXX_Spi( pHandler, ( WriteAddr_inBytes & 0xFF000000 ) >> 24 );
  }

  W25QXX_Spi( pHandler, ( WriteAddr_inBytes & 0xFF0000 ) >> 16 );
  W25QXX_Spi( pHandler, ( WriteAddr_inBytes & 0xFF00 ) >> 8 );
  W25QXX_Spi( pHandler, WriteAddr_inBytes & 0xFF );
  W25QXX_Spi( pHandler, pBuffer );

  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );
  W25QXX_WaitForWriteEnd( pHandler );

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx WriteByte done after %d ms \r\n", HAL_GetTick() - StartTime );
#endif

  pHandler->Get.Lock = 0;
}

/**
 * @brief
 * @param pHandler
 * @param pBuffer
 * @param Page_Address
 * @param OffsetInByte
 * @param NumByteToWrite_up_to_PageSize
 */
void W25QXX_WritePage( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_PageSize )
{
  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

  if( ( ( NumByteToWrite_up_to_PageSize + OffsetInByte ) > pHandler->Get.PageSize ) || ( NumByteToWrite_up_to_PageSize
      == 0 ) )
  {
    NumByteToWrite_up_to_PageSize = pHandler->Get.PageSize - OffsetInByte;
  }

  if( ( OffsetInByte + NumByteToWrite_up_to_PageSize ) > pHandler->Get.PageSize )
  {
    NumByteToWrite_up_to_PageSize = pHandler->Get.PageSize - OffsetInByte;
  }

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx WritePage:%d, Offset:%d ,Writes %d Bytes, begin... \r\n", Page_Address, OffsetInByte, NumByteToWrite_up_to_PageSize );
  uint32_t StartTime = HAL_GetTick();
#endif

  W25QXX_WaitForWriteEnd( pHandler );
  W25QXX_WriteEnable( pHandler );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0x02 );
  Page_Address = ( Page_Address * pHandler->Get.PageSize ) + OffsetInByte;

  if( pHandler->Get.ID >= eW25Q256 )
  {
    W25QXX_Spi( pHandler, ( Page_Address & 0xFF000000 ) >> 24 );
  }

  W25QXX_Spi( pHandler, ( Page_Address & 0xFF0000 ) >> 16 );
  W25QXX_Spi( pHandler, ( Page_Address & 0xFF00 ) >> 8 );
  W25QXX_Spi( pHandler, Page_Address & 0xFF );
  HAL_SPI_Transmit( pHandler->Set.spi, pBuffer, NumByteToWrite_up_to_PageSize, 100 );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );
  W25QXX_WaitForWriteEnd( pHandler );

#if ( W25QXX_DEBUG_ENABLED > 0 )

  StartTime = HAL_GetTick() - StartTime;

  for( uint32_t i = 0; i < NumByteToWrite_up_to_PageSize; i++ )
  {
    if( ( i % 8 == 0 ) && ( i > 2 ) )
    {
      W25QXX_INFO( "\r\n" );
      W25QXX_Delay( 10 );
    }

    W25QXX_INFO( "0x%02X,", pBuffer[ i ] );
  }

  W25QXX_INFO( "\r\n" );
  W25QXX_INFO( "w25qxx WritePage done after %d ms \r\n", StartTime );

#endif

  W25QXX_Delay( 1 );
  pHandler->Get.Lock = 0;
}

/**
 * @brief
 * @param pHandler
 * @param pBuffer
 * @param Sector_Address
 * @param OffsetInByte
 * @param NumByteToWrite_up_to_SectorSize
 */
void W25QXX_WriteSector( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToWrite_up_to_SectorSize )
{
  uint32_t StartPage;
  int32_t BytesToWrite;
  uint32_t LocalOffset;

  if( ( NumByteToWrite_up_to_SectorSize > pHandler->Get.SectorSize ) || ( NumByteToWrite_up_to_SectorSize == 0 ) )
  {
    NumByteToWrite_up_to_SectorSize = pHandler->Get.SectorSize;
  }

  W25QXX_INFO( "(OK) w25qxx WriteSector:%d, Offset:%d ,Write %d Bytes, begin... \r\n", Sector_Address, OffsetInByte, NumByteToWrite_up_to_SectorSize );

  if( OffsetInByte >= pHandler->Get.SectorSize )
  {
    W25QXX_ERRO( "w25qxx WriteSector Faild! \r\n" );
    return;
  }

  if( ( OffsetInByte + NumByteToWrite_up_to_SectorSize ) > pHandler->Get.SectorSize )
  {
    BytesToWrite = pHandler->Get.SectorSize - OffsetInByte;
  }
  else
  {
    BytesToWrite = NumByteToWrite_up_to_SectorSize;
  }

  StartPage = W25QXX_SectorToPage( pHandler, Sector_Address ) + ( OffsetInByte / pHandler->Get.PageSize );
  LocalOffset = OffsetInByte % pHandler->Get.PageSize;

  do
  {
    W25QXX_WritePage( pHandler, pBuffer, StartPage, LocalOffset, BytesToWrite );
    StartPage++;
    BytesToWrite -= pHandler->Get.PageSize - LocalOffset;
    pBuffer += pHandler->Get.PageSize;
    LocalOffset = 0;
  } while( BytesToWrite > 0 );

  W25QXX_INFO( "w25qxx WriteSector Done \r\n" );
}

/**
 * @brief
 * @param pHandler
 * @param pBuffer
 * @param Block_Address
 * @param OffsetInByte
 * @param NumByteToWrite_up_to_BlockSize
 */
void W25QXX_WriteBlock( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte,
                        uint32_t NumByteToWrite_up_to_BlockSize )
{
  uint32_t StartPage;
  int32_t BytesToWrite;
  uint32_t LocalOffset;

  StartPage = 0;
  BytesToWrite = 0;
  LocalOffset = 0;

  if( ( NumByteToWrite_up_to_BlockSize > pHandler->Get.BlockSize ) || ( NumByteToWrite_up_to_BlockSize == 0 ) )
  {
    NumByteToWrite_up_to_BlockSize = pHandler->Get.BlockSize;
  }

  W25QXX_INFO( "(OK) w25qxx WriteBlock:%d, Offset:%d ,Write %d Bytes, begin... \r\n", Block_Address, OffsetInByte, NumByteToWrite_up_to_BlockSize );

  if( OffsetInByte >= pHandler->Get.BlockSize )
  {
    W25QXX_ERRO( "w25qxx WriteBlock Faild! \r\n" );
    return;
  }

  if( ( OffsetInByte + NumByteToWrite_up_to_BlockSize ) > pHandler->Get.BlockSize )
  {
    BytesToWrite = pHandler->Get.BlockSize - OffsetInByte;
  }
  else
  {
    BytesToWrite = NumByteToWrite_up_to_BlockSize;
  }

  StartPage = W25QXX_BlockToPage( pHandler, Block_Address ) + ( OffsetInByte / pHandler->Get.PageSize );
  LocalOffset = OffsetInByte % pHandler->Get.PageSize;

  do
  {
    W25QXX_WritePage( pHandler, pBuffer, StartPage, LocalOffset, BytesToWrite );
    StartPage++;
    BytesToWrite -= pHandler->Get.PageSize - LocalOffset;
    pBuffer += pHandler->Get.PageSize;
    LocalOffset = 0;
  } while( BytesToWrite > 0 );

  W25QXX_INFO( "(X) w25qxx WriteBlock Done \r\n" );
}

/**
 * @brief
 * @param pHandler
 * @param pBuffer
 * @param Bytes_Address
 */
void W25QXX_ReadByte( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Bytes_Address )
{
  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

#if ( W25QXX_DEBUG_ENABLED > 0 )
  uint32_t StartTime = HAL_GetTick();
  W25QXX_INFO( "w25qxx ReadByte at address %d begin... \r\n", Bytes_Address );
#endif

  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0x0B );

  if( pHandler->Get.ID >= eW25Q256 )
  {
    W25QXX_Spi( pHandler, ( Bytes_Address & 0xFF000000 ) >> 24 );
  }

  W25QXX_Spi( pHandler, ( Bytes_Address & 0xFF0000 ) >> 16 );
  W25QXX_Spi( pHandler, ( Bytes_Address & 0xFF00 ) >> 8 );
  W25QXX_Spi( pHandler, Bytes_Address & 0xFF );
  W25QXX_Spi( pHandler, 0 );

  *pBuffer =W25QXX_Spi( pHandler, W25QXX_DUMMY_BYTE );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx ReadByte 0x%02X done after %d ms \r\n", *pBuffer, HAL_GetTick() - StartTime );
#endif

  pHandler->Get.Lock = 0;
}

/**
 * @brief
 * @param pHandler
 * @param pBuffer
 * @param ReadAddr
 * @param NumByteToRead
 */
void W25QXX_ReadBytes( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead )
{
  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

#if ( W25QXX_DEBUG_ENABLED > 0 )
  uint32_t StartTime = HAL_GetTick();
  W25QXX_INFO( "w25qxx ReadBytes at Address:%d, %d Bytes  begin... \r\n", ReadAddr, NumByteToRead );
#endif

  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0x0B );

  if( pHandler->Get.ID >= eW25Q256 )
  {
    W25QXX_Spi( pHandler, ( ReadAddr & 0xFF000000 ) >> 24 );
  }

  W25QXX_Spi( pHandler, ( ReadAddr & 0xFF0000 ) >> 16 );
  W25QXX_Spi( pHandler, ( ReadAddr & 0xFF00 ) >> 8 );
  W25QXX_Spi( pHandler, ReadAddr & 0xFF );
  W25QXX_Spi( pHandler, 0 );
  HAL_SPI_Receive( pHandler->Set.spi, pBuffer, NumByteToRead, 2000 );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

#if ( W25QXX_DEBUG_ENABLED > 0 )

  StartTime = HAL_GetTick() - StartTime;

  for( uint32_t i = 0; i < NumByteToRead; i++ )
  {
    if( ( i % 8 == 0 ) && ( i > 2 ) )
    {
      W25QXX_INFO( "\r\n" );
      W25QXX_Delay( 10 );
    }
    W25QXX_INFO( "0x%02X,", pBuffer[ i ] );
  }
  W25QXX_INFO( "\r\n" );
  W25QXX_INFO( "w25qxx ReadBytes done after %d ms \r\n", StartTime );
  W25QXX_Delay( 100 );

#endif

  W25QXX_Delay( 1 );
  pHandler->Get.Lock = 0;
}

/**
 * @brief
 * @param pHandler
 * @param pBuffer
 * @param Page_Address
 * @param OffsetInByte
 * @param NumByteToRead_up_to_PageSize
 */
void W25QXX_ReadPage( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Page_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_PageSize )
{
  while( pHandler->Get.Lock == 1 )
  {
    W25QXX_Delay( 1 );
  }

  pHandler->Get.Lock = 1;

  if( ( NumByteToRead_up_to_PageSize > pHandler->Get.PageSize ) || ( NumByteToRead_up_to_PageSize == 0 ) )
  {
    NumByteToRead_up_to_PageSize = pHandler->Get.PageSize;
  }
  if( ( OffsetInByte + NumByteToRead_up_to_PageSize ) > pHandler->Get.PageSize )
  {
    NumByteToRead_up_to_PageSize = pHandler->Get.PageSize - OffsetInByte;
  }

#if ( W25QXX_DEBUG_ENABLED > 0 )
  W25QXX_INFO( "w25qxx ReadPage:%d, Offset:%d ,Read %d Bytes, begin... \r\n", Page_Address, OffsetInByte, NumByteToRead_up_to_PageSize );
  uint32_t StartTime = HAL_GetTick();
#endif

  Page_Address = Page_Address * pHandler->Get.PageSize + OffsetInByte;
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_RESET );
  W25QXX_Spi( pHandler, 0x0B );

  if( pHandler->Get.ID >= eW25Q256 )
  {
    W25QXX_Spi( pHandler, ( Page_Address & 0xFF000000 ) >> 24 );
  }

  W25QXX_Spi( pHandler, ( Page_Address & 0xFF0000 ) >> 16 );
  W25QXX_Spi( pHandler, ( Page_Address & 0xFF00 ) >> 8 );
  W25QXX_Spi( pHandler, Page_Address & 0xFF );
  W25QXX_Spi( pHandler, 0 );
  HAL_SPI_Receive( pHandler->Set.spi, pBuffer, NumByteToRead_up_to_PageSize, 100 );
  HAL_GPIO_WritePin( pHandler->Set.Gpio, pHandler->Set.Pin, GPIO_PIN_SET );

#if ( W25QXX_DEBUG_ENABLED > 0 )
  StartTime = HAL_GetTick() - StartTime;

  for( uint32_t i = 0; i < NumByteToRead_up_to_PageSize; i++ )
  {
    if( ( i % 8 == 0 ) && ( i > 2 ) )
    {
      W25QXX_INFO( "\r\n" );
      W25QXX_Delay( 10 );
    }
    W25QXX_INFO( "0x%02X,", pBuffer[ i ] );
  }

  W25QXX_INFO( "\r\n" );
  W25QXX_INFO( "w25qxx ReadPage done after %d ms \r\n", StartTime );
#endif

  W25QXX_Delay( 1 );
  pHandler->Get.Lock = 0;
}

/**
 * @brief
 * @param pHandler
 * @param pBuffer
 * @param Sector_Address
 * @param OffsetInByte
 * @param NumByteToRead_up_to_SectorSize
 */
void W25QXX_ReadSector( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Sector_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_SectorSize )
{
  uint32_t StartPage;
  int32_t BytesToRead;
  uint32_t LocalOffset;

  StartPage = 0;
  BytesToRead = 0;
  LocalOffset = 0;

  if( ( NumByteToRead_up_to_SectorSize > pHandler->Get.SectorSize ) || ( NumByteToRead_up_to_SectorSize == 0 ) )
  {
    NumByteToRead_up_to_SectorSize = pHandler->Get.SectorSize;
  }

  W25QXX_INFO( "(OK) w25qxx ReadSector:%d, Offset:%d ,Read %d Bytes, begin... \r\n", Sector_Address, OffsetInByte, NumByteToRead_up_to_SectorSize );

  if( OffsetInByte >= pHandler->Get.SectorSize )
  {
    W25QXX_ERRO( "w25qxx ReadSector Faild! \r\n" );
    return;
  }

  if( ( OffsetInByte + NumByteToRead_up_to_SectorSize ) > pHandler->Get.SectorSize )
  {
    BytesToRead = pHandler->Get.SectorSize - OffsetInByte;
  }
  else
  {
    BytesToRead = NumByteToRead_up_to_SectorSize;
  }

  StartPage = W25QXX_SectorToPage( pHandler, Sector_Address ) + ( OffsetInByte / pHandler->Get.PageSize );
  LocalOffset = OffsetInByte % pHandler->Get.PageSize;

  do
  {
    W25QXX_ReadPage( pHandler, pBuffer, StartPage, LocalOffset, BytesToRead );
    StartPage++;
    BytesToRead -= pHandler->Get.PageSize - LocalOffset;
    pBuffer += pHandler->Get.PageSize;
    LocalOffset = 0;
  } while( BytesToRead > 0 );

  W25QXX_INFO( "w25qxx ReadSector Done \r\n" );
}

/**
 * @brief
 * @param pHandler
 * @param pBuffer
 * @param Block_Address
 * @param OffsetInByte
 * @param NumByteToRead_up_to_BlockSize
 */
void W25QXX_ReadBlock( W25QXX_Handler_t *pHandler, uint8_t *pBuffer, uint32_t Block_Address, uint32_t OffsetInByte, uint32_t NumByteToRead_up_to_BlockSize )
{
  uint32_t StartPage;
  int32_t BytesToRead;
  uint32_t LocalOffset;

  StartPage = 0;
  BytesToRead = 0;
  LocalOffset = 0;

  if( ( NumByteToRead_up_to_BlockSize > pHandler->Get.BlockSize ) || ( NumByteToRead_up_to_BlockSize == 0 ) )
  {
    NumByteToRead_up_to_BlockSize = pHandler->Get.BlockSize;
  }

  W25QXX_INFO( "(OK) w25qxx ReadBlock:%d, Offset:%d ,Read %d Bytes, begin... \r\n", Block_Address, OffsetInByte, NumByteToRead_up_to_BlockSize );

  if( OffsetInByte >= pHandler->Get.BlockSize )
  {
    W25QXX_ERRO( "w25qxx ReadBlock Faild! \r\n" );
    return;
  }

  if( ( OffsetInByte + NumByteToRead_up_to_BlockSize ) > pHandler->Get.BlockSize )
  {
    BytesToRead = pHandler->Get.BlockSize - OffsetInByte;
  }
  else
  {
    BytesToRead = NumByteToRead_up_to_BlockSize;
  }

  StartPage = W25QXX_BlockToPage( pHandler, Block_Address ) + ( OffsetInByte / pHandler->Get.PageSize );
  LocalOffset = OffsetInByte % pHandler->Get.PageSize;

  do
  {
    W25QXX_ReadPage( pHandler, pBuffer, StartPage, LocalOffset, BytesToRead );
    StartPage++;
    BytesToRead -= pHandler->Get.PageSize - LocalOffset;
    pBuffer += pHandler->Get.PageSize;
    LocalOffset = 0;
  } while( BytesToRead > 0 );

  W25QXX_INFO( "w25qxx ReadBlock Done \r\n" );
}

