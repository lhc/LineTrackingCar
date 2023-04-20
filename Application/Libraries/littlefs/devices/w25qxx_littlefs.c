/**
 * @file    w25qxx_littlefs.c
 * @brief   Library to use littlefs with w25qxx memory
 */

//==============================================================================
// Includes
//==============================================================================

#include "w25qxx_littlefs.h"
#include "w25qxx.h"

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
// Private function prototypes
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

static int W25QXX_Fs_BlockFlashRead(const struct lfs_config *LfsHandler, lfs_block_t Block, lfs_off_t Off, void *pBuffer, lfs_size_t Size);
static int w25qxx_Fs_BlockFlashProg(const struct lfs_config *LfsHandler, lfs_block_t Block, lfs_off_t Off, const void *pBuffer, lfs_size_t Size);
static int W25QXX_Fs_BlockFlashErase(const struct lfs_config *LfsHandler, lfs_block_t Block);
static int W25QXX_Fs_BlockFlashSync(const struct lfs_config *c);

//==============================================================================
// Private functions
//==============================================================================

static int W25QXX_Fs_BlockFlashRead(const struct lfs_config *LfsHandler, lfs_block_t Block, lfs_off_t Off, void *pBuffer, lfs_size_t Size)
{
  W25QXX_Handler_t *pW25qxxHandler;

  pW25qxxHandler = LfsHandler->context;

  W25QXX_ReadBlock(pW25qxxHandler, pBuffer, Block, Off, Size);

  return 0;
}

static int w25qxx_Fs_BlockFlashProg(const struct lfs_config *LfsHandler, lfs_block_t Block, lfs_off_t Off, const void *pBuffer, lfs_size_t Size)
{
  W25QXX_Handler_t *pW25qxxHandler;

  pW25qxxHandler = LfsHandler->context;

  W25QXX_WriteBlock( pW25qxxHandler, (uint8_t *)pBuffer, Block, Off, Size);

  return 0;
}

static int W25QXX_Fs_BlockFlashErase(const struct lfs_config *LfsHandler, lfs_block_t Block)
{
  uint32_t addr =  Block;
  W25QXX_Handler_t *pW25qxxHandler;

  pW25qxxHandler = LfsHandler->context;

  W25QXX_EraseBlock( pW25qxxHandler, addr);

  return 0;
}

static int W25QXX_Fs_BlockFlashSync(const struct lfs_config *c)
{
  return 0;
}

//==============================================================================
// Exported functions
//==============================================================================

int W25QXX_LittleFs_Init( LfsHandler_t *pLfsHandler, W25QXX_Handler_t *pDevice )
{
  int err;

  if( ( err = W25QXX_Init( pDevice ) )== 0)
  {
    pLfsHandler->FlSystem.Cfg.read  = W25QXX_Fs_BlockFlashRead;
    pLfsHandler->FlSystem.Cfg.prog  = w25qxx_Fs_BlockFlashProg;
    pLfsHandler->FlSystem.Cfg.erase = W25QXX_Fs_BlockFlashErase;
    pLfsHandler->FlSystem.Cfg.sync  = W25QXX_Fs_BlockFlashSync;

    /* Block device configuration */
    pLfsHandler->FlSystem.Cfg.read_buffer = pLfsHandler->buffers.Read;
    pLfsHandler->FlSystem.Cfg.prog_buffer = pLfsHandler->buffers.Prog;
    pLfsHandler->FlSystem.Cfg.lookahead_buffer = pLfsHandler->buffers.Lookahead;

    pLfsHandler->FlSystem.Cfg.read_size = sizeof(pLfsHandler->buffers.Read);
    pLfsHandler->FlSystem.Cfg.prog_size = sizeof(pLfsHandler->buffers.Prog);
    pLfsHandler->FlSystem.Cfg.lookahead_size = sizeof(pLfsHandler->buffers.Lookahead);

    pLfsHandler->FlSystem.Cfg.block_size =  pDevice->Get.BlockSize;
    pLfsHandler->FlSystem.Cfg.block_count = pDevice->Get.BlockCount;

    pLfsHandler->FlSystem.Cfg.cache_size = sizeof(pLfsHandler->buffers.Read);
    pLfsHandler->FlSystem.Cfg.block_cycles = 500;
    pLfsHandler->FlSystem.Cfg.context = pDevice;
  }

  return err;
}
