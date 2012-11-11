//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief usart 1 stub
///
/// \file
///
//  Change added test messages, USART1_Init function cycles through test
//         messages at each call, USART1_Transmit signals end of test
//
//============================================================================*/

/*------------------------------- Include Files ------------------------------*/

#include <string.h>

#include "stm32f10x.h"
#include "stm32f10x_usart.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#   undef VAR_STATIC
#endif
#define   VAR_STATIC static

#ifdef    VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

#define RX_BUFFER_LENGTH    128  //!< length of receive buffer
#define TX_BUFFER_LENGTH    128  //!< length of transmit buffer

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

const uint8_t PID_MENU[] = { 0xFE, 0x02, 0x08, 0xFF, 0x00, 0x15, 0x00, 0x00, 0xAD, 0x95 };
const uint8_t HUD_MENU[] = { 0xFE, 0x06, 0x0C, 0xFF, 0x00, 0x42, 0x14, 0x00, 0x00, 0x00, 0x0A, 0x01, 0x8B, 0x56 };
const uint8_t GPS_MENU[] = { 0xFE, 0x06, 0x04, 0xFF, 0x00, 0x42, 0x01, 0x00, 0x00, 0x00, 0x02, 0x01, 0x89, 0x69 };

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC uint8_t test_step = 0;                   //!< clear test counter
VAR_STATIC bool bTest_Running;
VAR_STATIC uint8_t ucRxWindex;                      //!< uplink write index
VAR_STATIC uint8_t ucRxRindex;                      //!< uplink read index
VAR_STATIC uint8_t ucTxWindex;                      //!< downlink write index
VAR_STATIC uint8_t ucTxRindex;                      //!< downlink read index
VAR_STATIC uint8_t ucRxBuffer[RX_BUFFER_LENGTH];    //!< uplink data buffer
VAR_STATIC uint8_t ucTxBuffer[TX_BUFFER_LENGTH];    //!< downlink data buffer

/*--------------------------------- Prototypes -------------------------------*/

/*---------------------------------- Functions -------------------------------*/

//----------------------------------------------------------------------------
//
/// \brief   Initialize USART1
/// \return  -
/// \remarks -
///
///
//----------------------------------------------------------------------------
void USART1_Init( void ) {

    ucRxRindex = 0;         // clear uplink read index
    ucTxWindex = 0;         // clear downlink write index
    ucTxRindex = 0;         // clear downlink read index
    bTest_Running = TRUE;
}


//----------------------------------------------------------------------------
//
/// \brief   Get character from USART 1 buffer
/// \param   c = pointer to destination character
/// \returns TRUE if receive buffer wasn't empty
/// \remarks -
///
//----------------------------------------------------------------------------
bool USART1_Getch(uint8_t * c) {

    if (ucRxRindex != ucRxWindex) {           // received another character
        *c = ucRxBuffer[ucRxRindex++];        // read character
        if (ucRxRindex >= RX_BUFFER_LENGTH) { // update read index
            ucRxRindex = 0;
        }
        return TRUE;
    } else {
        return FALSE;
    }
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

    ucTxBuffer[ucTxWindex++] = c;           // store character
    if (ucTxWindex >= TX_BUFFER_LENGTH) {   // update write index
        ucTxWindex = 0;
    }
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
    bTest_Running = FALSE;
}

//----------------------------------------------------------------------------
//
/// \brief
/// \param
/// \returns
/// \remarks -
///
//----------------------------------------------------------------------------
bool Test_Running( void ) {

    return bTest_Running;
}
