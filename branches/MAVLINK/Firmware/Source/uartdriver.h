//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \file
///  UART driver header file
//  CHANGES UART 0 and UART 1 are now managed in the same way
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*---------------------------------- Interface -------------------------------*/

void UARTInit(void);
void UART0Send(const unsigned char *pucBuffer, unsigned long ulCount);
void UART1Send(const unsigned char *pucBuffer, unsigned long ulCount);
tBoolean UART1GetChar ( char *ch );
tBoolean UART0GetChar ( char *ch );

