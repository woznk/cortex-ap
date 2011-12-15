//============================================================================+
//
// $RCSfile: ppmdriver.c,v $ (SOURCE FILE)
// $Revision: 1.5 $
// $Date: 2009/10/22 21:13:17 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             PPM input driver
//
//  CHANGES     REMOVED TIMER TRIGGER ON BOTH TIMER 1A, 1B THAT CAUSED UNWANTED
//              TRIGGERS ON ADC SAMPLE SEQUENCE 
//              period of timer 1A, 1B set to maximum achievable (8 ms)
//
//============================================================================*/

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
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

#define RC_CHANNELS    7            ///< Modify according to RC type
#define SYNC   (RC_CHANNELS + 1)    // Pulse index when waiting for first pulse
#define UNSYNC (RC_CHANNELS + 2)    // Pulse index when not synchronized

#define PPM_SYNC_MIN        5000    ///< Modify according to RC type
#define PPM_SYNC_MAX        20000   ///< Modify according to RC type
#define PPM_LENGTH_MIN      900
#define PPM_LENGTH_MAX      2100
#define PPM_LENGTH_NEUTRAL  1500


/*----------------------------------- Macros ---------------------------------*/

#define MISSING_PULSES (cOverflowCount >= 3)

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

///----------------------------------------------------------------------------
///
///  DESCRIPTION The PPM input capture interrupt handler.
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
void 
PPMCaptureIntHandler(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_CAPA_EVENT);           // Clear capture interrupt flag
    ulCaptureTime = TimerMatchGet(TIMER1_BASE, TIMER_A);    // Store captured time
    ulPulseLength = ulCaptureTime - ulLastCapture;          // Compute time difference
    ulLastCapture = ulCaptureTime;                          // Now is also last edge time

    switch ( ucPulseIndex )
    {
        case UNSYNC :                                       // NOT SYNCHRONIZED
            if (( ulPulseLength >= PPM_SYNC_MIN ) &&        // sync pulse detected  
                ( ulPulseLength <= PPM_SYNC_MAX )) {        // 
                ucPulseIndex = SYNC;                        // synchronized
                cOverflowCount = 0;                         // We're not in an overflow condition any more
            }
            break;

        case SYNC :                                         // SYNCHRONIZED
            if (( ulPulseLength >= PPM_LENGTH_MIN ) &&      // first channel's pulse detected 
                ( ulPulseLength <= PPM_LENGTH_MAX )) {      // 
                ulPulseBuffer[ 0 ] = ulPulseLength;         // pulse width is OK, save it
                ucPulseIndex = 1;                           // go for second channel
                cOverflowCount = 0;                         // We're not in an overflow condition any more
            } else {                                        // wrong pulse detected 
                ucPulseIndex = UNSYNC;                      // wait for next sync pulse
            }
            break;

        case RC_CHANNELS :                                  // LAST RC CHANNEL
            if (( ulPulseLength >= PPM_SYNC_MIN ) &&        // sync pulse detected 
                ( ulPulseLength <= PPM_SYNC_MAX )) {        // 
                ucPulseIndex++;                             // still synchronized 
                cOverflowCount = 0;                         // We're not in an overflow condition any more
            } else {                                        // wrong pulse detected 
                ucPulseIndex = UNSYNC;                      // wait for next sync pulse
            }
            break;

        default :                                           // CHANNELS 1 .. N - 1
            if (( ulPulseLength >= PPM_LENGTH_MIN ) &&      // good pulse detected 
                ( ulPulseLength <= PPM_LENGTH_MAX )) {      // 
                ulPulseBuffer[ ucPulseIndex ] = ulPulseLength;   // save it
                cOverflowCount = 0;                         // We're not in an overflow condition any more
            }
            ucPulseIndex++;                                 // always go for next channel
            break;
    } 
    if ( MISSING_PULSES ) {                                 // no pulses
        ucPulseIndex = UNSYNC;                              // wait for next sync pulse
    }
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION The PPM input overflow interrupt handler.
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
void 
PPMOverflowIntHandler(void)
{
    TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);   // Clear timeout interrupt flag
    if ( ! MISSING_PULSES )
    {
        cOverflowCount++;
    }
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION PPM timer initialization
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
void
PPMInit(void)
{
    ulLastCapture = 0;
    cOverflowCount = -1;
    ucPulseIndex = UNSYNC;

    //
    // Enable Timer 1 (PPM input)
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); 

    //
    // Enable I/O port B.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Configure the GPIO used as CCP.
    //
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_1); 

    //
    // Initialize Timer 1 has two 16 bit timers.
    // Timer A as capture timer, timer B as a periodic timer
    //
    TimerConfigure(TIMER1_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_A_CAP_TIME | TIMER_CFG_B_PERIODIC);

    //
    // Trigger timer A capture on rising edges
    //
    TimerControlEvent(TIMER1_BASE, TIMER_A, TIMER_EVENT_POS_EDGE);

    //
    // Configure maximum timer period: 8 MHz / 65535 = 8 ms
    //
    TimerLoadSet(TIMER1_BASE, TIMER_BOTH, 65535);

    //
    // Enable interrupt on timer A capture and on timer B timeout
    //
    IntEnable(INT_TIMER1A);
    IntEnable(INT_TIMER1B);
    TimerIntEnable(TIMER1_BASE, TIMER_CAPA_EVENT | TIMER_TIMB_TIMEOUT);

    //
    // Enable timers
    //
    TimerEnable(TIMER1_BASE, TIMER_BOTH);
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Get value of n-th radio channel
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
unsigned long 
PPMGetChannel(unsigned char ucChannel)
{
    if ( ucChannel < RC_CHANNELS ) {
        return ulPulseBuffer[ ucChannel ];
    } else {
        return PPM_LENGTH_NEUTRAL;
    }
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
