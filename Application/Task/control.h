/**
 * @file    control.h
 * @brief
 */

//==============================================================================
// Define to prevent recursive inclusion
//==============================================================================
#ifndef _TASK_CONTROL_H
#define _TASK_CONTROL_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// Includes
//==============================================================================

#include "pid/pid.h"
#include <stdint.h>
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
  eCAR_DIR_STOP = 0,
  eCAR_DIR_FRONT,
  eCAR_DIR_BACK,
  eCAR_DIR_RIGHT,
  eCAR_DIR_LEFT,
  eCAR_DIR_MAX,
} CarDirection_e;

//==============================================================================
// Exported variables
//==============================================================================

//==============================================================================
// Exported functions prototypes
//==============================================================================

void Control_EnableTrace( bool Status );
void Control_SetDirection( CarDirection_e Direction );
CarDirection_e Control_GetDirection( void );
const char* Control_GetDirectionString( CarDirection_e Direction );
void Control_SetParamPID( PidTypeParam_e Type, float Value );
float Control_GetParamPID( PidTypeParam_e Type );
void Control_Task(void *Parameters);

//==============================================================================
// Exported functions
//==============================================================================

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif  /* _TASK_CONTROL_H */
