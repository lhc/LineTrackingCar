/**
 * @file    setup_database.c
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================

#include "Setup/setup_hw.h"
#include "setup_database.h"
#include "Task/serial.h"
#include "littlefs/lfs.h"
#include "littlefs/lfs_util.h"
#include "littlefs/devices/lfs_file.h"
#include "littlefs/devices/w25qxx.h"
#include "littlefs/devices/w25qxx_littlefs.h"

//==============================================================================
// Private definitions
//==============================================================================

//==============================================================================
// Private macro
//==============================================================================

//==============================================================================
// Private typedef
//==============================================================================

//==============================================================================
// Extern variables
//==============================================================================

extern SPI_HandleTypeDef hspi1;

//==============================================================================
// Private variables
//==============================================================================

static LfsHandler_t gLfsHandler = {0};
static W25QXX_Handler_t gW25qxxHandler = {0};

//==============================================================================
// Private function prototypes
//==============================================================================

void Setup_DatabaseError(void);
lfs_ssize_t LittleFS_Write( LfsHandler_t *pHandler, const char *FileName, const char *Text, uint16_t Len );
lfs_ssize_t LittleFS_Read( LfsHandler_t *pHandler, const char *FileName, char *Data, uint16_t Len );

//==============================================================================
// Private functions
//==============================================================================

void Setup_DatabaseError(void)
{
  for(;;)
  {

  }
}

lfs_ssize_t LittleFS_Write( LfsHandler_t *pHandler, const char *FileName, const char *Text, uint16_t Len )
{
//  ErrResult_e result;
  lfs_ssize_t err = LFS_ERR_INVAL;
  struct lfs_file_config file_cfg;

//  err = file_lock( pHandler );
//  if( err != eSuccess )
//  {
//    LITFS_ERRO( "write, error take mutex.\r\n" );
//    return err;
//  }

  memset( &file_cfg, 0, sizeof(struct lfs_file_config) );
  file_cfg.buffer = pHandler->buffers.StaticBuffer;
  file_cfg.attr_count = 0;

  err = lfs_file_opencfg( &pHandler->FlSystem.Lfs, &pHandler->FlSystem.File, (const char *)FileName, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC, &file_cfg );
  if( err == LFS_ERR_OK )
  {
    err = lfs_file_write( &pHandler->FlSystem.Lfs, &pHandler->FlSystem.File, ( const void* ) Text, Len );
    if( err > 0 )
    {
      err = LFS_ERR_OK;
      lfs_file_close( &pHandler->FlSystem.Lfs, &pHandler->FlSystem.File );
      Serial_Message( "write file %s ok\r\n", FileName );
    }
    else
    {
      Serial_Message( "Error writing file %s \r\n", FileName );
    }
  }
  else
  {
    Serial_Message( "WRITE: Error opening file %s\r\n", FileName );
  }

//  file_unlock( pHandler );

  return err;
}

/**
 * @brief Read file
 * @param ctr
 * @param filename
 * @param data
 * @param len
 * @param offset
 * @return
 */
lfs_ssize_t LittleFS_Read( LfsHandler_t *pHandler, const char *FileName, char *Data, uint16_t Len )
{
  lfs_ssize_t err;
  struct lfs_file_config file_cfg;

//  err = file_lock( pHandler );
//  if( err != eSuccess )
//  {
//    LITFS_ERRO( "read, error take mutex\r\n" );
//    return err;
//  }

  memset( &file_cfg, 0, sizeof(struct lfs_file_config) );
  file_cfg.buffer = pHandler->buffers.StaticBuffer;
  file_cfg.attr_count = 0;

  err = lfs_file_opencfg( &pHandler->FlSystem.Lfs, &pHandler->FlSystem.File, (const char *)FileName, LFS_O_RDONLY, &file_cfg );
  if( err == LFS_ERR_OK )
  {
    err = lfs_file_read( &pHandler->FlSystem.Lfs, &pHandler->FlSystem.File, Data, Len );
    if( err > 0 )
    {
      err = LFS_ERR_OK;
      lfs_file_close( &pHandler->FlSystem.Lfs, &pHandler->FlSystem.File );
    }
    else
    {
      Serial_Message( "Error reading file %s  \r\n", FileName );
    }
  }
  else
  {
    Serial_Message( "READ: Error opening file %s \r\n", FileName );
  }

//  snprintf_( ( char* ) pHandler->Get.File.Name, LFS_MAX_FILENAME_SIZE, "%s", FileName );
//  file_unlock( pHandler );

  return err;
}

//==============================================================================
// Exported functions
//==============================================================================

void Setup_Database_Init( void )
{
  int err;

  W25QXX_Attach( &gW25qxxHandler, GPIOA, GPIO_PIN_4, &hspi1 );

  err = W25QXX_LittleFs_Init( &gLfsHandler, &gW25qxxHandler );
  if( err != 0 )
  {
    Setup_DatabaseError();
  }

  err = lfs_mount( &gLfsHandler.FlSystem.Lfs, &gLfsHandler.FlSystem.Cfg );
  if( err != 0 )
  {
    err = lfs_format( &gLfsHandler.FlSystem.Lfs, &gLfsHandler.FlSystem.Cfg );
    if(err == 0)
    {
      err = lfs_mount( &gLfsHandler.FlSystem.Lfs, &gLfsHandler.FlSystem.Cfg );
    }
  }
}

int32_t Database_Write( const char *FileName, void *Data, uint16_t Len )
{
  return LittleFS_Write( &gLfsHandler, FileName, Data, Len );
}

int32_t Database_Read( const char *FileName, void *Data, uint16_t Len )
{
  return LittleFS_Read( &gLfsHandler, FileName, Data, Len );
}
