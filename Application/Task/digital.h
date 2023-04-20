/**
 * @file    digital.h
 * @brief
 */

//==============================================================================
// Define to prevent recursive inclusion
//==============================================================================
#ifndef _TASK_DIGITAL_H
#define _TASK_DIGITAL_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// Includes
//==============================================================================

#include <stdint.h>
#include <stdbool.h>

//==============================================================================
//Exported constants
//==============================================================================

#define ANALOG_DMA_SIZE                  (8)

//==============================================================================
// Exported macro
//==============================================================================

//==============================================================================
// Exported types
//==============================================================================

typedef struct
{
  uint8_t Values;
} DigitalValues_t;

//==============================================================================
// Exported variables
//==============================================================================

//==============================================================================
// Exported functions prototypes
//==============================================================================

bool Digital_Read( DigitalValues_t *Analog );
void Digital_Task(void *Parameters);
void Digital_AttachBtn_Callback(void (*function_cb)(bool status));
//==============================================================================
// Exported functions
//==============================================================================

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif  /* _TASK_DIGITAL_H */
