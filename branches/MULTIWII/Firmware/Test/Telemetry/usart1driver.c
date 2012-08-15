//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief usart 1 driver stub
///
/// \file
///
///
//
//============================================================================*/

// ---- Include Files -------------------------------------------------------

 #include <stdint.h>

#include "config.h"
#include "stm32f10x.h"
#include "usart1driver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#   undef VAR_STATIC
#endif
#define   VAR_STATIC static

#ifdef    VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

/*---------------------------------- Functions -------------------------------*/


//----------------------------------------------------------------------------
//
/// \brief   Get character from USART 1 buffer
/// \param   c = pointer to destination character
/// \returns TRUE if receive buffer wasn't empty
/// \remarks -
///
//----------------------------------------------------------------------------
bool USART1_Getch(uint8_t * c) {

    return TRUE;
}

//----------------------------------------------------------------------------
//
/// \brief   Put a character into USART 1 buffer
/// \param   c = character
/// \returns
/// \remarks -
///
//----------------------------------------------------------------------------
void USART1_Putch(uint8_t c) {

}

//----------------------------------------------------------------------------
//
/// \brief   Put a word into USART 1 buffer
/// \param   w = word
/// \returns
/// \remarks -
///
//----------------------------------------------------------------------------
void USART1_Putw(uint16_t w) {

    uint8_t ucTemp;

    ucTemp = (uint8_t)((w >> 8) & 0x00FF);  // get MSB
    USART1_Putch(ucTemp);                   // store MSB
    ucTemp = (uint8_t)(w & 0x00FF);         // get LSB
    USART1_Putch(ucTemp);                   // store LSB
}

//----------------------------------------------------------------------------
//
/// \brief   Put a float number into USART 1 buffer
/// \param   f = float number
/// \returns
/// \remarks -
///
//----------------------------------------------------------------------------
void USART1_Putf(float f) {

    uint8_t * pucTemp = (uint8_t *)&f;

    USART1_Putch(*pucTemp++);   // store byte 1
    USART1_Putch(*pucTemp++);   // store byte 2
    USART1_Putch(*pucTemp++);   // store byte 3
    USART1_Putch(*pucTemp);     // store byte 4
}

//----------------------------------------------------------------------------
//
/// \brief
/// \param
/// \returns
/// \remarks -
///
//----------------------------------------------------------------------------
void USART1_Transmit( void ) {

}
