/**
 * @file    digital.c
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================

#include "digital.h"
#include "Setup/setup_hw.h"
#include "bitwise/bitwise.h"
#include "button/button.h"

//==============================================================================
// Private definitions
//==============================================================================

#define DIGITAL_QUEUE_NUM_ITEMS           (2)
#define DIGITAL_QUEUE_READ_TIMEOUT_MS     (5000)
#define DIGITAL_QUEUE_WRITE_TIMEOUT_MS    (300)
#define DIGITAL_NUM_GPIO                  (8)

//==============================================================================
// Private macro
//==============================================================================

//==============================================================================
// Private typedef
//==============================================================================

typedef struct
{
  GPIO_TypeDef *Gpio;
  uint32_t Pin;
} DigitaDrv_t;

typedef struct
{
  TIM_HandleTypeDef *TimerDrv;
  uint32_t TimerChannel;
} PwmDriver_t;

//==============================================================================
// Extern variables
//==============================================================================

extern TIM_HandleTypeDef htim4; // Pino DIR do sensor de distancia 250 Hz

//==============================================================================
// Private function prototypes
//==============================================================================

static void Digital_Error( HAL_StatusTypeDef Err );
static void Digital_Init( void );
static void Digital_ReadLineSensor( DigitalValues_t *Digital );

//==============================================================================
// Private variables
//==============================================================================

static xQueueHandle gQueueDigital;
static DigitaDrv_t gLineSensor[DIGITAL_NUM_GPIO] = {0};
static PwmDriver_t gLineSensorIR = {0};
static Button_t gBtn = {0};

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

/**
 * @brief percent:  range: 0 to 100
 */
static void Digital_Motor_SetPwm( PwmDriver_t *Motor, uint8_t Percent )
{
  uint32_t compareValue;

  if( Percent <= 100 )
  {
    // Converte a porcentagem do duty cycle em um valor de contagem para o PWM.
    compareValue = ( ( uint32_t ) Percent * __HAL_TIM_GET_AUTORELOAD( Motor->TimerDrv ) ) / 100;
    __HAL_TIM_SET_COMPARE( Motor->TimerDrv, Motor->TimerChannel, compareValue );
  }
}

static void Digital_Init(void)
{
  gBtn.Drv.Gpio = GPIOB;
  gBtn.Drv.Pin = GPIO_PIN_9;
  gBtn.OldStatus = true;
  gBtn.CurrStatus = true;
  gBtn.function_cb = NULL;

  gLineSensorIR.TimerDrv = &htim4;
  gLineSensorIR.TimerChannel = TIM_CHANNEL_2;

  gLineSensor[0].Gpio = GPIOB;
  gLineSensor[0].Pin = GPIO_PIN_6;

  gLineSensor[1].Gpio = GPIOA;
  gLineSensor[1].Pin = GPIO_PIN_1;

  gLineSensor[2].Gpio = GPIOA;
  gLineSensor[2].Pin = GPIO_PIN_2;

  gLineSensor[3].Gpio = GPIOA;
  gLineSensor[3].Pin = GPIO_PIN_3;

  gLineSensor[4].Gpio = GPIOB;
  gLineSensor[4].Pin = GPIO_PIN_4;

  gLineSensor[5].Gpio = GPIOB;
  gLineSensor[5].Pin = GPIO_PIN_5;

  gLineSensor[6].Gpio = GPIOB;
  gLineSensor[6].Pin = GPIO_PIN_0;

  gLineSensor[7].Gpio = GPIOB;
  gLineSensor[7].Pin = GPIO_PIN_1;

  gQueueDigital = xQueueCreate( DIGITAL_QUEUE_NUM_ITEMS, sizeof(uint8_t) );
  if( gQueueDigital == NULL )
  {
    Digital_Error( HAL_ERROR );
  }

  HAL_TIM_PWM_Start( gLineSensorIR.TimerDrv, gLineSensorIR.TimerChannel );
  Digital_Motor_SetPwm( &gLineSensorIR, 50);
}

static void Digital_ReadLineSensor( DigitalValues_t *Digital )
{
  uint8_t value;
  GPIO_PinState gpio_state;

  for( uint8_t index = 0; index < DIGITAL_NUM_GPIO; index++ )
  {
    gpio_state = HAL_GPIO_ReadPin( gLineSensor[ index ].Gpio, gLineSensor[ index ].Pin );
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

void Digital_AttachBtn_Callback(void (*function_cb)(bool status))
{
  gBtn.function_cb = function_cb;
}

void Digital_Task( void *Parameters )
{
  DigitalValues_t digital;

  Digital_Init();

  /* Infinite loop */
  for( ;; )
  {
    Digital_ReadLineSensor( &digital );
    Button_Read( &gBtn );
    vTaskDelay( 10 );

    // Adiciona valor calculado a queue, q agora pode ser consumida pela task control
    xQueueSend( gQueueDigital, &digital, DIGITAL_QUEUE_WRITE_TIMEOUT_MS );
  }
}
