/**
 * @file    printer.cpp
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================

#include "digital.h"
#include "Setup/setup_hw.h"
#include "bitwise/bitwise.h"

//==============================================================================
// Private definitions
//==============================================================================

#define DIGITAL_QUEUE_NUM_ITEMS           (2)
#define DIGITAL_QUEUE_READ_TIMEOUT_MS     (5000)
#define DIGITAL_QUEUE_WRITE_TIMEOUT_MS    (5000)
#define DIGITAL_NUM_GPIO                  (8)

//==============================================================================
// Private macro
//==============================================================================

//==============================================================================
// Private typedef
//==============================================================================

typedef struct
{
  GPIO_TypeDef * Gpio;
  uint32_t Pin;
}DigitaDrv_t;

//==============================================================================
// Extern variables
//==============================================================================

//==============================================================================
// Private function prototypes
//==============================================================================

static void Digital_Error( HAL_StatusTypeDef Err );
static void Digital_Init( void );
static void Digital_GPIO_Read( DigitalValues_t *Digital );

//==============================================================================
// Private variables
//==============================================================================

static xQueueHandle gQueueDigital;
static DigitaDrv_t Sensor[DIGITAL_NUM_GPIO] = {0};
static DigitaDrv_t SensorIR = {0};

//==============================================================================
// Private functions
//==============================================================================

static void Digital_Error( HAL_StatusTypeDef Err )
{
  // Algum erro ocorreu, verificar
  for( ;; )
  {

  }
}

static void Digital_Init(void)
{
  SensorIR.Gpio = GPIOB;
  SensorIR.Pin = GPIO_PIN_2;

  Sensor[0].Gpio = GPIOA;
  Sensor[0].Pin = GPIO_PIN_0;

  Sensor[1].Gpio = GPIOA;
  Sensor[1].Pin = GPIO_PIN_1;

  Sensor[2].Gpio = GPIOA;
  Sensor[2].Pin = GPIO_PIN_2;

  Sensor[3].Gpio = GPIOA;
  Sensor[3].Pin = GPIO_PIN_3;

  Sensor[4].Gpio = GPIOA;
  Sensor[4].Pin = GPIO_PIN_6;

  Sensor[5].Gpio = GPIOA;
  Sensor[5].Pin = GPIO_PIN_7;

  Sensor[6].Gpio = GPIOB;
  Sensor[6].Pin = GPIO_PIN_0;

  Sensor[7].Gpio = GPIOB;
  Sensor[7].Pin = GPIO_PIN_1;

  gQueueDigital = xQueueCreate( DIGITAL_QUEUE_NUM_ITEMS, sizeof(uint8_t) );
  if( gQueueDigital == NULL )
  {
    Digital_Error( HAL_ERROR );
  }

  HAL_GPIO_WritePin( SensorIR.Gpio, SensorIR.Pin, GPIO_PIN_SET );
}

static void Digital_GPIO_Read( DigitalValues_t *Digital )
{
  uint8_t value;
  GPIO_PinState gpio_state;

  for( uint8_t index = 0; index < DIGITAL_NUM_GPIO; index++ )
  {
    gpio_state = HAL_GPIO_ReadPin( Sensor[ index ].Gpio, Sensor[ index ].Pin );
    _BIT_WR_BOL( value, index, gpio_state );
  }

  Digital->Values = value;
}

//==============================================================================
// Exported functions
//==============================================================================

bool Digital_Read( DigitalValues_t *Digital )
{
  if( xQueueReceive( gQueueDigital, &Digital->Values, DIGITAL_QUEUE_READ_TIMEOUT_MS ) == pdTRUE )
  {
    return true;
  }

  return false;
}

void Digital_Task( void *Parameters )
{
  DigitalValues_t digital;

  Digital_Init();

  /* Infinite loop */
  for( ;; )
  {
    Digital_GPIO_Read(&digital);
    vTaskDelay(10);
    // Adiciona valor calculado a queue, q agora pode ser consumida pela task control
    xQueueSend( gQueueDigital, &digital, DIGITAL_QUEUE_WRITE_TIMEOUT_MS );
  }
}
