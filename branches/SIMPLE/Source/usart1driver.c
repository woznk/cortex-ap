//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief usart 1 driver
///
/// \file
///
/// Change: completed initialization of USART1 registers
//
//============================================================================*/

// ---- Include Files -------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
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

#define RX_BUFFER_LENGTH    48  //!< length of receive buffer
#define TX_BUFFER_LENGTH    48  //!< length of transmit buffer

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

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
/// \remarks for direct register initialization of USART 1 see :
/// http://www.micromouseonline.com/2009/12/31/stm32-usart-basics/#ixzz1eG1EE8bT
///
///
//----------------------------------------------------------------------------
void USART1_Init( void ) {

#define UART_CLOCK (12000000)
#define BAUD_RATE  (57600)
#define DIV        ((float)UART_CLOCK / (16.0f * (float)BAUD_RATE))
#define DIV_INT    ((uint32_t)DIV)
#define DIV_FR     ((uint32_t)((DIV - DIV_INT) * 16.0f))

    NVIC_InitTypeDef NVIC_InitStructure;

    USART1->BRR = DIV_INT << 4 + DIV_FR;    // BAUD rate register

    USART1->CR1  = 0x0000002C;              // control register 1
                  //      ||
                  //      |+- enable tx (TE=1), enable rx (RE=1)
                  //      +-- enable RXNE interrupt

    // Configure NVIC for USART1 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    USART1->CR1 |= 0x00002000;              // enable USART
                  //     |
                  //     +--- enable USART (UE=1)

    ucRxWindex = 0;                         // clear uplink write index
    ucRxRindex = 0;                         // clear uplink read index
    ucTxWindex = 0;                         // clear downlink write index
    ucTxRindex = 0;                         // clear downlink read index
}

//----------------------------------------------------------------------------
//
/// \brief   USART 1 interrupt handler
/// \param   -
/// \returns -
/// \remarks -
///
//----------------------------------------------------------------------------
void USART1_IRQHandler( void ) {
//  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//  portCHAR cChar;

#define RXNE ((uint32_t)0x00000020)

    if ((USART1->SR & RXNE) != 0) {
//		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
		ucRxBuffer[ucRxWindex++] = (uint8_t)(USART1->DR);
        if (ucRxWindex >= RX_BUFFER_LENGTH) {
            ucRxWindex = 0;
        }
    }
/*
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
//		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
		ucRxBuffer[ucRxWindex++] = USART_ReceiveData( USART1 );
        if (ucRxWindex >= RX_BUFFER_LENGTH) {
            ucRxWindex = 0;
        }
    }
*/
//	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
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

    ucTemp = (uint8_t)(w & 0x00FF);         // get LSB
    USART1_Putch(ucTemp);                   // store LSB
    ucTemp = (uint8_t)((w >> 8) & 0x00FF);  // get MSB
    USART1_Putch(ucTemp);                   // store MSB
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

#define TXE ((uint32_t)0x00000080)

    while (ucTxRindex != ucTxWindex) {
        while ((USART1->SR & TXE) == 0) {
        }
        USART1->DR = ucTxBuffer[ucTxRindex++];
        if (ucTxRindex >= RX_BUFFER_LENGTH) {
            ucTxRindex = 0;
        }
    }
/*
    while (ucTxRindex != ucTxWindex) {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
        }
        USART_SendData(USART1, ucTxBuffer[ucTxRindex++]);
        if (ucTxRindex >= RX_BUFFER_LENGTH) {
            ucTxRindex = 0;
        }
    }
*/
}
