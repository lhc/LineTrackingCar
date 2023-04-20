/**
 * @file    serial.c
 * @brief
 */

//==============================================================================
// Includes
//==============================================================================

#include "serial.h"
#include "Setup/setup_hw.h"
#include "bitwise/bitwise.h"
#include "xprintf/xprintf.h"
#include "cli/FreeRTOS_CLI.h"
#include <stdio.h>
#include <string.h>

//==============================================================================
// Private definitions
//==============================================================================

#define SERIAL_BUFFER_SIZE                     (1024)
#define SERIAL_TRIGGER_LEVEL                   (10)
#define SERIAL_MUTEX_TIMEOUT_MS                (1000)
#define SERIAL_TX_WAIT_TIMEOUT_MS              (5000)

#define SERIAL_RX_CONSOLE_CIRCULAR_BUFFER_SIZE (512)
#define SERIAL_TX_CONSOLE_CIRCULAR_BUFFER_SIZE (512)

#define SERIAL_CHAR_ASCII_RETURN               ( 0x0D )
#define SERIAL_CHAR_ASCII_DEL                  ( '\b' )

//==============================================================================
// Private macro
//==============================================================================

//==============================================================================
// Private typedef
//==============================================================================

static SemaphoreHandle_t gSerialMutex = NULL;
static SemaphoreHandle_t gSemaphoreTx = NULL;
static StreamBufferHandle_t xStreamBuffer = NULL;
static uint8_t gSerialBuffer[ SERIAL_BUFFER_SIZE ];

static QueueHandle_t gQueueRxCli = NULL;
static uint8_t gRxCliData;
static uint8_t gCliBuffer[SERIAL_RX_CONSOLE_CIRCULAR_BUFFER_SIZE];
static uint8_t gCliOutputBuffer[SERIAL_TX_CONSOLE_CIRCULAR_BUFFER_SIZE];

//==============================================================================
// Extern variables
//==============================================================================

extern UART_HandleTypeDef huart1;

//==============================================================================
// Private function prototypes
//==============================================================================

static void Serial_Init( void );
static void Serial_Error( HAL_StatusTypeDef Err );
static bool Serial_Lock( void );
static void Serial_Unlock( void );
static void Serial_Transmit( void );
static void Serial_ISR_Error( void );
static void Serial_ISR_Tx( void );

static void Serial_CLI_Init( void );
static bool Serial_CLI_MountCommand( uint8_t *Buffer, uint16_t SizeBuffer, uint16_t *CmdIndex, uint8_t Caracter );
static void Serial_CLI_ParseCommand( uint8_t *inputBuffer, uint16_t inputBufferSize, uint8_t *outputBuffer, uint16_t outputBufferSize );

//==============================================================================
// Private variables
//==============================================================================

//==============================================================================
// Private functions
//==============================================================================

static void Serial_Error( HAL_StatusTypeDef Err )
{
  // Algum erro ocorreu, verificar
  for( ;; )
  {

  }
}

static void Serial_Init( void )
{
  xStreamBuffer = xStreamBufferCreate( 2048, SERIAL_TRIGGER_LEVEL );
  if( xStreamBuffer == NULL )
  {
    Serial_Error( HAL_ERROR );
  }

  gSerialMutex = xSemaphoreCreateBinary( );
  if( gSerialMutex == NULL )
  {
    Serial_Error( HAL_ERROR );
  }
  else
  {
    xSemaphoreGive( gSerialMutex );
  }

  gSemaphoreTx = xSemaphoreCreateBinary( );
  if( gSemaphoreTx == NULL )
  {
    Serial_Error( HAL_ERROR );
  }
}

static bool Serial_Lock(void)
{
  if( gSerialMutex == NULL )
  {
    return false;
  }

  if( xSemaphoreTake( gSerialMutex, SERIAL_MUTEX_TIMEOUT_MS ) == pdTRUE )
  {
    return true;
  }

  return false;
}

static void Serial_Unlock(void)
{
  xSemaphoreGive( gSerialMutex );
}

static void Serial_Transmit( void )
{
  uint8_t buffer[20];
  size_t xReceivedBytes;
  const TickType_t xBlockTime = pdMS_TO_TICKS( 20 );

  if(xStreamBuffer == NULL)
  {
    return;
  }

  xReceivedBytes = xStreamBufferReceive( xStreamBuffer, ( void* ) buffer, sizeof( buffer ), xBlockTime );
  if( xReceivedBytes > 0 )
  {
    HAL_UART_Transmit_DMA( &huart1, buffer, xReceivedBytes);
    xSemaphoreTake(gSemaphoreTx, SERIAL_TX_WAIT_TIMEOUT_MS);
  }
}

static void Serial_ISR_Error( void )
{
  BaseType_t taskWoken = pdFALSE;

  if( gSemaphoreTx != NULL )
  {
    xSemaphoreGiveFromISR( gSemaphoreTx, &taskWoken );
  }
}

static void Serial_ISR_Tx( void )
{
  BaseType_t taskWoken = pdFALSE;

  if( gSemaphoreTx != NULL )
  {
    xSemaphoreGiveFromISR( gSemaphoreTx, &taskWoken );
  }
}

static void Serial_ISR_Rx( void )
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if( gQueueRxCli != NULL )
  {
    xQueueSendFromISR( gQueueRxCli, &gRxCliData, &xHigherPriorityTaskWoken );

    HAL_UART_Receive_IT( &huart1, ( uint8_t* ) &gRxCliData, 1 );

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
  }
}

static void Serial_CLI_Init( void )
{
  gQueueRxCli = xQueueCreate( 30, sizeof(uint8_t) );
  if( gQueueRxCli == NULL )
  {
    Serial_Error( HAL_ERROR );
  }
}

bool Serial_CLI_MountCommand( uint8_t *Buffer, uint16_t SizeBuffer, uint16_t *CmdIndex, uint8_t Caracter )
{
  bool isReady = false;

  switch (Caracter)
  {
    case SERIAL_CHAR_ASCII_DEL:
    {
      if( *CmdIndex > 0 )
      {
        Buffer[ (*CmdIndex)-- ] = 0x00;
      }
      break;
    }
    case '\n':
    case '\r':
    {
      if( *CmdIndex > 0 )
      {
        isReady = true;
      }
      break;
    }
    default:
      Buffer[ ( *CmdIndex )++ ] = Caracter;
      break;
  }

  if( *CmdIndex >= SizeBuffer )
  {
    *CmdIndex = 0;
    memset ( (char*)Buffer, 0x00, SizeBuffer );
  }

  return isReady;
}

static void Serial_CLI_ParseCommand( uint8_t *inputBuffer, uint16_t inputBufferSize, uint8_t *outputBuffer, uint16_t outputBufferSize )
{
  BaseType_t moreDataToFollow = 0;

  do
  {
    // Get the next output string from the command interpreter.
    moreDataToFollow = FreeRTOS_CLIProcessCommand( (const char *)inputBuffer, (char *)outputBuffer, outputBufferSize );

    // Write the generated string to the UART.
    Serial_Message( "%s\r\n", outputBuffer );

  } while( moreDataToFollow != pdFALSE );

}

//==============================================================================
// Exported functions
//==============================================================================

void Serial_Task( void *Parameters )
{
  Serial_Init();

  /* Infinite loop */
  for( ;; )
  {
    Serial_Transmit();
  }
}

void Serial_CLI_Task (void * argument)
{
  uint8_t data;
  uint16_t size_cmd = 0;
  bool isReady = false;

  // Initialize buffers
  memset ( (char*)gCliBuffer, 0x00, sizeof ( gCliBuffer ) );
  memset ( (char*)gCliOutputBuffer, 0x00, sizeof ( gCliOutputBuffer ) );

  Serial_CLI_Init();

  // Start RX Interrupt on usart7, for first reception
  HAL_UART_Receive_IT ( &huart1, (uint8_t *) &gRxCliData, 1 );

  Serial_Message ( "\r\n==== LINE TRACKING CAR %s ====\r\n", SETUP_FIRMWARE_VERSION );

  for ( ;; )
  {
    if ( xQueueReceive( gQueueRxCli, &data, portMAX_DELAY ) )
    {
      isReady = Serial_CLI_MountCommand(gCliBuffer, sizeof(gCliBuffer), &size_cmd, data);
      if( isReady )
      {
        Serial_CLI_ParseCommand(gCliBuffer, size_cmd, gCliOutputBuffer, sizeof(gCliOutputBuffer));

        /*  Clear the input string ready to receive the next command. */
        memset( gCliBuffer, 0x00, size_cmd );
        size_cmd = 0;
      }
     }
  }
}

#if (SERIAL_ENABLE == 0)

__weak void Serial_Message(const char *Format, ... )
{

}

#else

void Serial_Message(const char *Format, ... )
{
  size_t xBytesSent;
  va_list args;
  uint16_t length ;
  const TickType_t x100ms = pdMS_TO_TICKS( 100 );

  if( xStreamBuffer == NULL )
  {
    return;
  }

  if( Serial_Lock() )
  {
    va_start( args, Format );
    length = vsnprintf_( ( char* ) gSerialBuffer, sizeof( gSerialBuffer ), Format, args );
    va_end( args );

    /* Send an array to the stream buffer, blocking for a maximum of 100ms to
     wait for enough space to be available in the stream buffer. */
    xBytesSent = xStreamBufferSend( xStreamBuffer, gSerialBuffer, length, x100ms );

    Serial_Unlock();
  }
}

/* CALLBACKS DA HAL DA ST */

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
  if( huart->Instance == USART1 )
  {
    Serial_ISR_Rx();
  }
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *huart )
{
  if( huart->Instance == USART1 )
  {
    Serial_ISR_Tx();
  }
}

void HAL_UART_AbortCpltCallback( UART_HandleTypeDef *huart )
{
}

void HAL_UART_ErrorCallback( UART_HandleTypeDef *huart )
{
  if( huart->Instance == USART1 )
  {
    Serial_ISR_Error();
  }
}

void HAL_UART_AbortTransmitCpltCallback( UART_HandleTypeDef *huart )
{
  if( huart->Instance == USART1 )
  {
    Serial_ISR_Error();
  }
}

#endif
