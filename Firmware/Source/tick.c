//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief  Tick manager
//  CHANGES disktimerproc() called inside SysTick_Handler()
//
//============================================================================*/

#include "stm32f10x.h"
#include "diskio.h"
#include "tick.h"

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

//
// A set of flags used to track the state of the application.
//
VAR_GLOBAL unsigned long g_ulFlags;

/*----------------------------------- Locals ---------------------------------*/

//
// The debounced state of the five push buttons.
//
//     0 - Up
//     1 - Down
//     2 - Left
//     3 - Right
//     4 - Select
//
VAR_STATIC unsigned char g_ucSwitches = 0x1f;

//
// The vertical counter used to debounce the push buttons.
// The bit positions are the same as g_ucSwitches.
//
VAR_STATIC unsigned char g_ucSwitchClockA = 0;
VAR_STATIC unsigned char g_ucSwitchClockB = 0;


/*--------------------------------- Prototypes -------------------------------*/


//----------------------------------------------------------------------------
//
/// \brief   Handles the SysTick timeout interrupt.
/// \return
/// \remarks
///
//----------------------------------------------------------------------------
void TickInit(void) {

}

//----------------------------------------------------------------------------
//
/// \brief   Handles the SysTick timeout interrupt.
/// \return
/// \remarks
///
//----------------------------------------------------------------------------
RAMFUNC void SysTick_Handler(void)
{
  static uint32_t ticks;

    //
    // Indicate that a 10 ms tick has occurred.
    //
    g_ulFlags |= FLAG_CLOCK_TICK_10;

    //
    // Increment the tick count.
    //
    if ((ticks++ % 2) == 0) {
        //
        // Indicate that a 20 ms tick has occurred.
        //
        g_ulFlags |= FLAG_CLOCK_TICK_20;
    }
    //
    // disk timer to be called every 10ms
    //
    disk_timerproc();
}

//----------------------------------------------------------------------------
//
/// \brief   Manages Logic and I/O
/// \return
/// \remarks
///
//----------------------------------------------------------------------------
void Logic(void) {

    unsigned long ulData, ulDelta;

    //
    // Determine the switches that are at a different state than the debounced
    // state.
    //
    ulDelta = ulData ^ g_ucSwitches;

    //
    // Increment the clocks by one.
    //
    g_ucSwitchClockA ^= g_ucSwitchClockB;
    g_ucSwitchClockB = ~g_ucSwitchClockB;

    //
    // Reset the clocks corresponding to switches that have not changed state.
    //
    g_ucSwitchClockA &= ulDelta;
    g_ucSwitchClockB &= ulDelta;

    //
    // Get the new debounced switch state.
    //
    g_ucSwitches &= g_ucSwitchClockA | g_ucSwitchClockB;
    g_ucSwitches |= (~(g_ucSwitchClockA | g_ucSwitchClockB)) & ulData;

    //
    // Determine the switches that just changed debounced state.
    //
    ulDelta ^= (g_ucSwitchClockA | g_ucSwitchClockB);

    //
    // See if the select button was just pressed.
    //
    if ((ulDelta & 0x10) && !(g_ucSwitches & 0x10))
    {
        //
        // Set a flag to indicate that the select button was just pressed.
        //
        g_ulFlags |= FLAG_BUTTON_PRESS;
    }
}

