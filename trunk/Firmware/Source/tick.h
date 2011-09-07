//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief  Tick manager
//  CHANGES Modified for STM32F100RB
//
//============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

#define FLAG_CLOCK_TICK_10      0           //
#define FLAG_CLOCK_TICK_20      1           //
#define FLAG_CLOCK_COUNT_LOW    2           // The low bit of the clock count
#define FLAG_CLOCK_COUNT_HIGH   3           // The high bit of the clock count
#define FLAG_BUTTON             4           // Debounced state of the button
#define FLAG_DEBOUNCE_LOW       5           // Low bit of the debounce clock
#define FLAG_DEBOUNCE_HIGH      6           // High bit of the debounce clock
#define FLAG_BUTTON_PRESS       7           // The button was just pressed

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*------------------------------------ Types ---------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

//
// A set of flags used to track the state of the application.
//
VAR_GLOBAL unsigned long g_ulFlags;

/*---------------------------------- Interface -------------------------------*/

void Logic( void );
