/**
 * @file    printer.cpp
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================

#include "control.h"
#include "digital.h"
#include "serial.h"
#include "Setup/setup_hw.h"
#include "bitwise/bitwise.h"

//==============================================================================
// Private definitions
//==============================================================================

//==============================================================================
// Private macro
//==============================================================================

//==============================================================================
// Private typedef
//==============================================================================

typedef enum
{
  eMOTOR_HALT = 0,
  eMOTOR_FORWARD,
  eMOTOR_BACKWARD,
} MotorDirection_e;

typedef enum
{
  eSTOP = 0,
  eFRONT,
  eBACK,
  eRIGHT,
  eLETH,
} CarDirection_e;

typedef struct
{
  /* Gpio Driver control */
  GPIO_TypeDef *GpioP;
  uint16_t GpioPinP;
  GPIO_TypeDef *GpioN;
  uint16_t GpioPinN;

  /** Pwm driver control */
  TIM_HandleTypeDef *TimerDrv;
  uint32_t TimerChannel;
  uint32_t CompareValue;
} MotorDriver_t;

typedef struct
{
  MotorDriver_t MotorA;
  MotorDriver_t MotorB;
} MotorCtrl_t;

typedef struct
{
  CarDirection_e Dir;
  uint8_t DutyCyclePercentA; /* range 0 to 100%  */
  uint8_t DutyCyclePercentB; /* range 0 to 100%  */
} CarCtrl_t;

//==============================================================================
// Extern variables
//==============================================================================

extern TIM_HandleTypeDef htim1; // Pwm configurado para 1Khz
extern TIM_HandleTypeDef htim2; // Pwm configurado para 1Khz

//==============================================================================
// Private function prototypes
//==============================================================================

static void Control_Init( MotorCtrl_t *Ctrl );
static void Control_Motor_SetDir( MotorDriver_t *Motor,  MotorDirection_e Dir );
static void Control_Motor_SetPwm( MotorDriver_t *Motor, uint8_t Percent );

static void Control_Car_SetDir( MotorCtrl_t *MotorCtrl, CarCtrl_t *CarCtrl );
static void Control_Car_CalcDirection( CarCtrl_t *Car, DigitalValues_t *Digital );
static void Control_Trace( CarCtrl_t *Car, DigitalValues_t *Digital );

//==============================================================================
// Private variables
//==============================================================================

static bool gIsLogEnable = false;
static PidCtrl_t gPidCtrl = {0};

//==============================================================================
// Private functions
//==============================================================================

#if 0

// Funcao usada para o teste dos motores
static void Control_Test_Raw(void)
{
  uint32_t dutyCycle1  = 0;
  uint32_t dutyCycle2  = 0;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15 , GPIO_PIN_SET);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 , GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13 , GPIO_PIN_SET);

  for(;;)
  {
    if( dutyCycle2 < __HAL_TIM_GET_AUTORELOAD( &htim2 ) )
    {
      __HAL_TIM_SET_COMPARE( &htim2, TIM_CHANNEL_1, ++dutyCycle2 );
    }
    else
    {
      dutyCycle2 = 0;
    }

    if( dutyCycle1 < __HAL_TIM_GET_AUTORELOAD( &htim1 ) )
    {
      __HAL_TIM_SET_COMPARE( &htim1, TIM_CHANNEL_1, ++dutyCycle1 );
    }
    else
    {
      dutyCycle1 = 0;
    }

    vTaskDelay(10);
  }
}


// Funcao usada para a direcao dos motores
static void Control_Test_Direction(void)
{
  CarCtrl_t carCtrl;
  MotorCtrl_t MotorCtrl;

  DigitalValues_t adcValues;

  // Configura GPIO e PWM dos motores
  Control_Init( &MotorCtrl );

  carCtrl.DutyCyclePercentA = 50;
  carCtrl.DutyCyclePercentB = 50;

  for(;;)
  {
    carCtrl.Dir = eFRONT;
    Control_Car_SetDir( &MotorCtrl, &carCtrl );
    vTaskDelay(1000);

    carCtrl.Dir = eBACK;
    Control_Car_SetDir( &MotorCtrl, &carCtrl );
    vTaskDelay(1000);

    carCtrl.Dir = eRIGHT;
    Control_Car_SetDir( &MotorCtrl, &carCtrl );
    vTaskDelay(1000);

    carCtrl.Dir = eLETH;
    Control_Car_SetDir( &MotorCtrl, &carCtrl );
    vTaskDelay(1000);
  }
}

#endif

static void Control_Init( MotorCtrl_t *Ctrl )
{
  Ctrl->MotorA.GpioP = GPIOB;
  Ctrl->MotorA.GpioPinP = GPIO_PIN_14;
  Ctrl->MotorA.GpioN = GPIOB;
  Ctrl->MotorA.GpioPinN = GPIO_PIN_15;
  Ctrl->MotorA.TimerDrv = &htim1;
  Ctrl->MotorA.TimerChannel = TIM_CHANNEL_1;

  Ctrl->MotorB.GpioP = GPIOB;
  Ctrl->MotorB.GpioPinP = GPIO_PIN_12;
  Ctrl->MotorB.GpioN = GPIOB;
  Ctrl->MotorB.GpioPinN = GPIO_PIN_13;
  Ctrl->MotorB.TimerDrv = &htim2;
  Ctrl->MotorB.TimerChannel = TIM_CHANNEL_1;

  __HAL_TIM_SET_COMPARE( Ctrl->MotorA.TimerDrv, Ctrl->MotorA.TimerChannel, 0 );
  HAL_TIM_PWM_Start( Ctrl->MotorA.TimerDrv, Ctrl->MotorA.TimerChannel );

  __HAL_TIM_SET_COMPARE( Ctrl->MotorB.TimerDrv, Ctrl->MotorB.TimerChannel, 0 );
  HAL_TIM_PWM_Start( Ctrl->MotorB.TimerDrv, Ctrl->MotorB.TimerChannel );
}

static void Control_Motor_SetDir( MotorDriver_t *Motor,  MotorDirection_e Dir )
{
  switch( Dir )
  {
    case eMOTOR_HALT:
    {
      HAL_GPIO_WritePin( Motor->GpioP, Motor->GpioPinP, GPIO_PIN_RESET );
      HAL_GPIO_WritePin( Motor->GpioN, Motor->GpioPinN, GPIO_PIN_RESET );
      break;
    }
    case eMOTOR_FORWARD:
    {
      HAL_GPIO_WritePin(Motor->GpioP, Motor->GpioPinP, GPIO_PIN_SET );
      HAL_GPIO_WritePin(Motor->GpioN, Motor->GpioPinN, GPIO_PIN_RESET );
      break;
    }
    case eMOTOR_BACKWARD:
    {
      HAL_GPIO_WritePin( Motor->GpioP, Motor->GpioPinP, GPIO_PIN_RESET );
      HAL_GPIO_WritePin( Motor->GpioN, Motor->GpioPinN, GPIO_PIN_SET );
      break;
    }
    default:
      return;
  }
}

/**
 * @brief percent:  range: 0 to 100
 */
static void Control_Motor_SetPwm( MotorDriver_t *Motor, uint8_t Percent )
{
  if( Percent <= 100 )
  {
    // Converte a porcentagem do duty cycle em um valor de contagem para o PWM.
    Motor->CompareValue = ( ( uint32_t ) Percent * __HAL_TIM_GET_AUTORELOAD( Motor->TimerDrv ) ) / 100;
    __HAL_TIM_SET_COMPARE( Motor->TimerDrv, Motor->TimerChannel, Motor->CompareValue );
  }
}

static void Control_Car_SetDir( MotorCtrl_t *MotorCtrl, CarCtrl_t *CarCtrl )
{
  MotorDirection_e dirMotorA;
  MotorDirection_e dirMotorB;

  switch( CarCtrl->Dir )
  {
    case eSTOP:
    {
      dirMotorA = eMOTOR_HALT;
      dirMotorB = eMOTOR_HALT;
      break;
    }
    case eFRONT:
    {
      dirMotorA = eMOTOR_FORWARD;
      dirMotorB = eMOTOR_FORWARD;
      break;
    }
    case eBACK:
    {
      dirMotorA = eMOTOR_BACKWARD;
      dirMotorB = eMOTOR_BACKWARD;
      break;
    }
    case eRIGHT:
    {
      dirMotorA = eMOTOR_HALT;
      dirMotorB = eMOTOR_FORWARD;
      break;
    }
    case eLETH:
    {
      dirMotorA = eMOTOR_FORWARD;
      dirMotorB = eMOTOR_HALT;
      break;
    }
    default:
      return;
  }

  // Motor A
  Control_Motor_SetDir( &MotorCtrl->MotorA, dirMotorA );
//  Control_Motor_SetPwm( &MotorCtrl->MotorA, CarCtrl->DutyCyclePercentA );

  // Motor B
  Control_Motor_SetDir( &MotorCtrl->MotorB, dirMotorB );
//  Control_Motor_SetPwm( &MotorCtrl->MotorB, CarCtrl->DutyCyclePercentB );
}

static void Control_Car_CalcDirection( CarCtrl_t *Car, DigitalValues_t *Digital )
{
  static uint16_t percent; // Variavel global que n perder o valor(static)

  if( ++percent > 100 )
  {
    percent = 0;
  }

  Car->Dir = eFRONT;
  Car->DutyCyclePercentA = percent;
  Car->DutyCyclePercentB = percent;
}

__inline static void Control_Trace(CarCtrl_t *Car, DigitalValues_t *Digital)
{
  if( gIsLogEnable )
  {
    Serial_Message( "S: 0b"_BYTE_TO_BINARY_PATTERN", Dir: %d, M1: %d %%, M2: %d %%\r\n", _BYTE_TO_BINARY(Digital->Values), Car->Dir, Car->DutyCyclePercentA, Car->DutyCyclePercentB );
  }
}

//==============================================================================
// Exported functions
//==============================================================================

void Control_EnableTrace( bool Status )
{
  gIsLogEnable = Status;
}

void Control_SetParamPID(PidTypeParam_e Type, float Value)
{
  switch( Type )
  {
    case ePID_KP:
    {
      gPidCtrl.Kp = Value;
      break;
    }
    case ePID_KI:
    {
      gPidCtrl.Ki = Value;
      break;
    }
    case ePID_KD:
    {
      gPidCtrl.Kd = Value;
      break;
    }
    case ePID_SetPoint:
    {
      gPidCtrl.SetPoint = Value;
      break;
    }
    default:
    break;
  }
}

float Control_GetParamPID( PidTypeParam_e Type )
{
  switch( Type )
  {
    case ePID_KP:
    {
      return gPidCtrl.Kp;
    }
    case ePID_KI:
    {
      return gPidCtrl.Ki;
    }
    case ePID_KD:
    {
      return gPidCtrl.Kd;
    }
    case ePID_SetPoint:
    {
      return gPidCtrl.SetPoint;
    }
    default:
      return 0;
  }
}

void Control_Task(void *Parameters)
{
  bool adcUpdated;

  CarCtrl_t carCtrl;
  MotorCtrl_t MotorCtrl;

  DigitalValues_t digitalValues;

  // Configura GPIO e PWM dos motores
  Control_Init( &MotorCtrl );

  /* Infinite loop */
  for(;;)
  {
    // procura novos valores do canal AD
    adcUpdated = Digital_Read( &digitalValues );
    if( adcUpdated )
    {
      Control_Car_CalcDirection( &carCtrl, &digitalValues );
      Control_Car_SetDir( &MotorCtrl, &carCtrl );
      Control_Trace( &carCtrl, &digitalValues );
    }
  }
}
