//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief usart 1 driver header file
///
/// \file
///
///
//
//
//============================================================================*/

// ---- Include Files -------------------------------------------------------


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

/*---------------------------------- Interface -------------------------------*/

void USART1_Init(void);            // Initialize USART1
bool USART1_Getch(uint8_t * c);    // Get character from USART 1 buffer
void USART1_Putch(uint8_t c);      // Put a character into USART 1 buffer
void USART1_Putw(uint16_t w);      // Put a word into USART 1 buffer
void USART1_Putf(float f);         // Put a float number into USART 1 buffer
void USART1_Transmit( void );      // 
