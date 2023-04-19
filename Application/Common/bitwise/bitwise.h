/**
 * @file    bitiwise.h
 * @brief
 */

//==============================================================================
// Define to prevent recursive inclusion
//==============================================================================
#ifndef _BITWISE_
#define	_BITWISE_

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
// Exported macro
//==============================================================================

//==============================================================================
//Exported constants
//==============================================================================

/** @brief coloca em 1 o bit x da variavel. */
#define _BIT_SET( value, bit_x )           ( value |= ( 1 << bit_x ) )

/** @brief coloca em 0 no bit_x da variavel. */
#define _BIT_CLR( value, bit_x )           ( value &= ~( 1 << bit_x ) )

/** @brief troca o estado logico do bit x da variavel. */
#define _BIT_TOOGLE( value, bit_x )      ( value ^= (1 << bit_x ) )

/** @brief retorna o estado do bit x na forma de mascara de bits. */
#define _BIT_TST( value, bit_x )           ( value & ( 1 << bit_x ) )

/** @brief retorna 0 ou 1 conforme leitura do bit x da variavel. */
#define _BIT_TST_BOOL( value, bit_x )      ( ( value & ( 1 << bit_x ) ) >> bit_x )

/** @brief Escreve 0 ou 1 no bit x da variavel. */
#define _BIT_WR_BOL( value, bit_x, _bool ) ( _bool == 1 ? _BIT_SET( value, bit_x ) : _BIT_CLR( value, bit_x ) )

/** @brief check if just the mask of bits are setting */
#define _BIT_CHK_MASK( value, mask )        ( ( value & mask ) == mask ? 1 : 0 )


#define _BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define _BYTE_TO_BINARY(byte)  \
  ((byte) & 0x80 ? '1' : '0'), \
  ((byte) & 0x40 ? '1' : '0'), \
  ((byte) & 0x20 ? '1' : '0'), \
  ((byte) & 0x10 ? '1' : '0'), \
  ((byte) & 0x08 ? '1' : '0'), \
  ((byte) & 0x04 ? '1' : '0'), \
  ((byte) & 0x02 ? '1' : '0'), \
  ((byte) & 0x01 ? '1' : '0')

//==============================================================================
// Exported macro
//==============================================================================

//==============================================================================
// Exported types
//==============================================================================

//==============================================================================
// Exported variables
//==============================================================================

//==============================================================================
// Exported functions prototypes
//==============================================================================

bool isDigitString( const char *buffer, uint16_t len );

//==============================================================================
// Exported functions
//==============================================================================

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif /* _BITWISE_ */
