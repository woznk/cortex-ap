//============================================================================+
//
// $RCSfile: tick.c,v $ (SOURCE FILE)
// $Revision: 1.2 $
// $Date: 2009/10/21 20:22:15 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             Tick manager
//
//  CHANGES     added TickInit function
//              added management of 20 ms tick flag
//
//============================================================================*/

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"

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

VAR_STATIC unsigned long g_ulTickCount = 0;

/*--------------------------------- Prototypes -------------------------------*/


//----------------------------------------------------------------------------
//
/// \brief   Handles the SysTick timeout interrupt.
///
/// \remarks 
///
//----------------------------------------------------------------------------
void
TickInit(void)
{
    //
    // Configure SysTick for a 100Hz interrupt
    //
    SysTickPeriodSet(SysCtlClockGet() / 100);
    SysTickEnable();
    SysTickIntEnable();

    //
    // Enable I/O port G.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG); 

    //
    // Configure the GPIOs used to read the push buttons.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTG_BASE,
                         GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
                         GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTG_BASE,
                     GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
                     GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

//----------------------------------------------------------------------------
//
/// \brief   Handles the SysTick timeout interrupt.
///
/// \remarks 
///
//----------------------------------------------------------------------------
void
SysTickIntHandler(void)
{
    unsigned long ulData, ulDelta;

    //
    // Call the FatFs tick timer.
    //
    disk_timerproc();

    //
    // Increment the tick count.
    //
    if ((g_ulTickCount++ % 2) == 0)
    {
        //
        // Indicate that a 20 ms tick has occurred.
        //
        HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK) = 1;
    }

    //
    // Read the state of the push buttons.
    //
    ulData = ((GPIOPinRead(GPIO_PORTG_BASE, (GPIO_PIN_3 | GPIO_PIN_4 |
                                            GPIO_PIN_5 | GPIO_PIN_6)) >> 3) |
              (GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_7) >> 3));

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
    if((ulDelta & 0x10) && !(g_ucSwitches & 0x10))
    {
        //
        // Set a flag to indicate that the select button was just pressed.
        //
        HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS) = 1;
    }
}
