/**
* @file    lfs_file.h
* @brief
*/

//==============================================================================
// Define to prevent recursive inclusion
//==============================================================================

#ifndef _LFS_FILE_H
#define _LFS_FILE_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// Includes
//==============================================================================
#include "../lfs.h"

//==============================================================================
//Exported constants
//==============================================================================

#define LFS_FILE_SIZE_BUFFER      (64)
#define LFS_MAX_FILENAME_SIZE     (25)

//==============================================================================
// Exported macro
//==============================================================================

//==============================================================================
// Exported types
//==============================================================================

typedef struct
{
  uint8_t Read[ LFS_FILE_SIZE_BUFFER ];
  uint8_t Prog[ LFS_FILE_SIZE_BUFFER ];
  uint8_t Lookahead[ LFS_FILE_SIZE_BUFFER ];
  uint8_t StaticBuffer[ LFS_FILE_SIZE_BUFFER ];
} LfsBuffers_t;

typedef struct
{
  lfs_t Lfs;
  lfs_file_t File;
  lfs_dir_t Dir;
  struct lfs_info Info;
  struct lfs_config Cfg;
  bool IsMounted;
  SemaphoreHandle_t MutexFatfs;
} LfsFilepHandlerl_t;

typedef struct
{
  LfsFilepHandlerl_t FlSystem;
  LfsBuffers_t buffers;
} LfsHandler_t;

//==============================================================================
// Exported variables
//==============================================================================

//==============================================================================
// Exported functions prototypes
//==============================================================================

//==============================================================================
// Exported functions
//==============================================================================

#ifdef __cplusplus
}
#endif

#endif /* _LFS_FILE_H */
