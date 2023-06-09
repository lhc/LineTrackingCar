/**
 * @file    led.c
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================

#include "led.h"

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

//==============================================================================
// Private function prototypes
//==============================================================================

//==============================================================================
// Private variables
//==============================================================================

//==============================================================================
// Private functions
//==============================================================================

//==============================================================================
// Exported functions
//==============================================================================

void Led_Blink( Led_t *Led, LedBlinkType_e Type, uint8_t NumBlink )
{
  uint32_t timeBetweenBlinkMs;

  if( Type == eLED_BLINK_1 )
  {
    timeBetweenBlinkMs = 500;
  }
  else
  {
    timeBetweenBlinkMs = 200;
  }

  HAL_GPIO_WritePin( Led->Gpio, Led->GpioPin, GPIO_PIN_RESET );
  for(uint16_t index = 0; index < NumBlink*2; index++)
  {
    HAL_GPIO_TogglePin( Led->Gpio, Led->GpioPin );
    vTaskDelay( timeBetweenBlinkMs );
  }
}
