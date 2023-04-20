/**
 * @file    setup_hw.c
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================

#include "setup_hw.h"
#include "setup_database.h"

#include "Task/commands.h"
#include "Task/digital.h"
#include "Task/control.h"
#include "Task/serial.h"
#include <stddef.h>

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

extern UART_HandleTypeDef huart1;

//==============================================================================
// Private variables
//==============================================================================

//==============================================================================
// Private function prototypes
//==============================================================================

//==============================================================================
// Private functions
//==============================================================================

#define ANALOG_PRIORITY    (tskIDLE_PRIORITY + 4)
#define SERIAL_PRIORITY    (tskIDLE_PRIORITY + 4)
#define DIGITAL_PRIORITY   (tskIDLE_PRIORITY + 4)

//==============================================================================
// Exported functions
//==============================================================================

void Setup_Error(void)
{
  for(;;)
  {

  }
}
/**
 * @brief
 */
void Setup_Init( void )
{
  BaseType_t result;

  // Cria tarefas
#if (SERIAL_ENABLE > 0)

  result = xTaskCreate( Serial_Task, "serial", (configMINIMAL_STACK_SIZE * 4), NULL, SERIAL_PRIORITY, NULL );
  if( !result )
  {
    Setup_Error();
  }

  result = xTaskCreate( Serial_CLI_Task, "cli", (configMINIMAL_STACK_SIZE * 6), NULL, SERIAL_PRIORITY, NULL );
  if( !result )
  {
    Setup_Error();
  }

  CLI_RegisterCommands();

//  vTaskDelay(2000);

#endif

  Setup_Database_Init();

  result = xTaskCreate( Digital_Task, "digital", (configMINIMAL_STACK_SIZE * 5), NULL, ANALOG_PRIORITY, NULL );
  if( !result )
  {
    Setup_Error();
  }

  result = xTaskCreate( Control_Task, "control", (configMINIMAL_STACK_SIZE * 5), NULL, DIGITAL_PRIORITY, NULL );
  if( !result )
  {
    Setup_Error();
  }
}
