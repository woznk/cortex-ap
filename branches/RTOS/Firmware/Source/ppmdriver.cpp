//============================================================================+
//
// $RCSfile: ppmdriver.cpp,v $ (SOURCE FILE)
// $Revision: 1.2 $
// $Date: 2011/01/19 18:33:18 $
// $Author: Lorenz $
//
/// \brief   PPM input driver
///
/// \file
///
//  CHANGES Eliminato lampeggio LED nell'interrupt PPM per debug
//
//============================================================================*/

#include "stdafx.h"

#include "inc/lm3s9b90.h"

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "ppmdriver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#define SYNC   (RC_CHANNELS + 1)   ///< Pulse index while waiting for first pulse
#define UNSYNC (RC_CHANNELS + 2)   ///< Pulse index when not synchronized

#define PPM_SYNC_MIN        250000UL   ///< Minimum sync pulse length (5 ms). 
#define PPM_SYNC_MAX        750000UL   ///< Maximum sync pulse length (15 ms). 
#define PPM_LENGTH_MIN      45000UL    ///< Absolute minimum pulse length (0.9 ms).
#define PPM_LENGTH_MAX      105000UL   ///< Absolute maximum pulse length (2.1 ms).
#define PPM_LENGTH_NEUTRAL  75000UL    ///< Pulse length of servo neutral position (1.5 ms).

/*----------------------------------- Macros ---------------------------------*/

#define MISSING_PULSES (cOverflowCount >= 3)    ///< Missing pulses condition

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC unsigned long ulPulseBuffer[RC_CHANNELS];
VAR_STATIC unsigned char ucPulseIndex;
VAR_STATIC unsigned long ulCaptureTime;
VAR_STATIC unsigned long ulLastCapture;
VAR_STATIC unsigned long ulPulseLength;
VAR_STATIC signed char cOverflowCount;

/*--------------------------------- Prototypes -------------------------------*/

/*---------------------------------- Functions -------------------------------*/

#ifndef _WINDOWS

///----------------------------------------------------------------------------
///
///  DESCRIPTION PPM input capture interrupt handler.
/// \return      -
/// \remarks    Duration ~2 us with system clock = 50 MHz
///
///----------------------------------------------------------------------------
void 
PPMCaptureIntHandler(void) {

    GPIOPinIntClear(GPIO_PORTC_BASE, GPIO_PIN_4);           // Clear interrupt flag
    ulCaptureTime = TimerValueGet(TIMER1_BASE, TIMER_A);    // Store captured time
    ulPulseLength = ulCaptureTime - ulLastCapture;          // Compute time difference
    ulLastCapture = ulCaptureTime;                          // Now is also last edge time

    switch ( ucPulseIndex ) {
        case UNSYNC :                                       // NOT SYNCHRONIZED
            if (( ulPulseLength >= PPM_SYNC_MIN ) &&        // sync pulse detected  
                ( ulPulseLength <= PPM_SYNC_MAX )) {        // 
                ucPulseIndex = SYNC;                        // synchronized
            }
            break;

        case SYNC :                                         // SYNCHRONIZED
            if (( ulPulseLength >= PPM_LENGTH_MIN ) &&      // first channel's pulse detected 
                ( ulPulseLength <= PPM_LENGTH_MAX )) {      // 
                ulPulseBuffer[ 0 ] = ulPulseLength;         // pulse width is OK, save it
                ucPulseIndex = 1;                           // go for second channel
            } else {                                        // wrong pulse detected 
                ucPulseIndex = UNSYNC;                      // wait for next sync pulse
            }
            break;

        case RC_CHANNELS :                                  // LAST RC CHANNEL
            if (( ulPulseLength >= PPM_SYNC_MIN ) &&        // sync pulse detected 
                ( ulPulseLength <= PPM_SYNC_MAX )) {        // 
                ucPulseIndex++;                             // still synchronized 
            } else {                                        // wrong pulse detected 
                ucPulseIndex = UNSYNC;                      // wait for next sync pulse
            }
            break;

        default :                                           // CHANNELS 1 .. N - 1
            if (( ulPulseLength >= PPM_LENGTH_MIN ) &&      // good pulse detected 
                ( ulPulseLength <= PPM_LENGTH_MAX )) {      // 
                ulPulseBuffer[ ucPulseIndex ] = ulPulseLength;  // save it
            }
            ucPulseIndex++;                                 // always go for next channel
            break;
    } 
    if ( MISSING_PULSES ) {                                 // no pulses
        ucPulseIndex = UNSYNC;                              // wait for next sync pulse
    }
}

#endif // _WINDOWS

///----------------------------------------------------------------------------
///
///  DESCRIPTION PPM timer initialization
/// \return      -
/// \remarks     LM3S1968 board CCP2 on PB1 
///              LM3S9B90 board CCP2 on PC4 (PB1 used for VBUS)
///
///----------------------------------------------------------------------------
void
PPMInit(void)
{
#ifndef _WINDOWS
    ulLastCapture = 0;
    cOverflowCount = -1;
    ucPulseIndex = UNSYNC;

    //
    // Enable Timer 1
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); 
        
    //
    // Halt timer 1
    //
    TimerDisable(TIMER1_BASE, TIMER_A);

    //
    // Initialize Timer 1 
    //
    TimerConfigure(TIMER1_BASE, TIMER_CFG_32_BIT_PER |  // one 32 bit timer
                                TIMER_TAMR_TACDIR);     // timer A counts up

    //
    // Configure maximum timer period: 0xFFFFFFFF / 50 MHz = 85 s
    //
    TimerLoadSet(TIMER1_BASE, TIMER_A, 0xFFFFFFFF);

    //
    // Start timer 1
    //
    TimerEnable(TIMER1_BASE, TIMER_A);

    //
    // Disable interrupt on GPIO pin.
    //
    IntEnable(INT_GPIOC);

    //
    // Enable GPIO port for input capture.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    //
    // Configure the GPIO pins used as input.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4);

    //
    // Select GPIO pin rising edge interrupt.
    //
    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);

    //
    // Enable interrupt on GPIO pin.
    //
    IntEnable(INT_GPIOC);
    GPIOPinIntEnable(GPIO_PORTC_BASE, GPIO_PIN_4);

#endif // _WINDOWS
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Get value of radio channels
/// \return      Value of n-th channel between 1000 and 2000
/// \REMARKS     
///
///----------------------------------------------------------------------------
unsigned long 
PPMGetChannel(unsigned char ucChannel)
{
#ifndef _WINDOWS
    if ( ucChannel < RC_CHANNELS ) {
        return (ulPulseBuffer[ ucChannel ] / 50);
    } else {
        return (PPM_LENGTH_NEUTRAL / 50);
    }
#else 
    return (PPM_LENGTH_NEUTRAL / 50);
#endif // _WINDOWS
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION Returns status of radio signal.
///
/// \remarks   -
///
///----------------------------------------------------------------------------
unsigned char 
PPMSignalStatus( void ) {
    if (MISSING_PULSES) {
        return PPM_SIGNAL_BAD;          // bad radio signal
    } else {
        return PPM_SIGNAL_OK;           // radio signal is OK
    }
}
