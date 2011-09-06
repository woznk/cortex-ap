//============================================================================+
//
// $RCSfile: servodriver.c,v $ (SOURCE FILE)
// $Revision: 1.10 $
// $Date: 2010/04/14 17:39:18 $
// $Author: Lorenz $
//
//  LANGUAGE C
/// \file
/// \brief   Servo driver
//
//  CHANGES  aggiornati i commenti
//
//============================================================================*/

#include "math.h"
#include "inc/lm3s9b90.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "AileronCtrl.h"
#include "ElevatorCtrl.h"
#include "RudderCtrl.h"
#include "ppmdriver.h"
#include "servodriver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#define PRESCALER      16UL

#define SERVO_MIN       900UL   ///< Absolute minimum pulse length (0.9 ms).
#define SERVO_MAX      2100UL   ///< Absolute maximum pulse length (2.1 ms).
#define SERVO_NEUTRAL  1500UL   ///< Pulse length of servo neutral position (1.5 ms).

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC unsigned long ulFrequency;
VAR_STATIC long lPositionA;
VAR_STATIC long lPositionB;
VAR_STATIC float fElevatorGain = 500.0f;  //!< Elevator servo conversion gain
VAR_STATIC float fRudderGain = 500.0f;    //!< Rudder servo conversion gain
VAR_STATIC float fAileronGain = 500.0f;   //!< Aileron servo conversion gain

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
///  DESCRIPTION PWM timers initialization
/// \return      -
/// \remarks     Initialization sequences
///
///     PWM Timing mode:
///     1. Ensure the timer is disabled
///     2. Write the GPTM Configuration Register (GPTMCFG) with 0x0000.0004.
///     3. In the GPTM Timer Mode (GPTMTnMR) register, set TnAMS to 0x1, 
///        TnCMR to 0x0, TnMR field to 0x2.
///     4. Configure the output state of the PWM signal (whether or not 
///        it is inverted) in the TnEVENT field of the GPTMCTL register.
///     5. Load the timer start value into the GPTM Timer n Interval Load 
///        GPTMTnILR register.
///     6. Load the GPTM Timer n Match (GPTMTnMATCHR) register with the 
///        match value.
///     7. Set the TnEN bit in the GPTMCTL register to enable the timer 
///        and begin generation of the output PWM signal.
/// 
///     In PWM Timing mode, the timer continues running after the PWM signal 
///     has been generated. The PWM period can be adjusted at any time by 
///     writing the GPTMTnILR register, and the change takes effect at the 
///     next cycle after the write.
///
///     16-bit Periodic mode:
///     1. Ensure the timer is disabled (the TnEN bit is cleared) before making any changes.
///     2. Write the GPTM Configuration Register (GPTMCFG) with 0x0000.0004.
///     3. In the GPTM Timer Mode (GPTMTnMR) register, set the TnMR field to
///        0x2 for Periodic mode.
///     4. Optionally configure the TnSNAPS, TnWOT, TnMTE and TnCDIR bits in the GPTMTnMR register
///     to select whether to capture the value of the free-running timer at time-out, use an external
///     trigger to start counting, configure an additional trigger or interrupt, and count up or down.
///     5. write the prescale value to the GPTM Timer n Prescale Register (GPTMTnPR).
///     6. Load the start value into the GPTM Timer Interval Load Register (GPTMTnILR).
///     7. If interrupts are required, set the appropriate bit in the GPTM Interrupt Mask Register
///     (GPTMIMR).
///     8. Set the TnEN bit in the GPTM Control Register (GPTMCTL) to enable the timer and start
///     counting.
///     9. Poll the GPTMRIS register or wait for the interrupt to be generated (if enabled). In both cases,
///     the status flags are cleared by writing a 1 the appropriate bit of the GPTM Interrupt Clear
///     Register (GPTMICR).
///     If the TnMIE bit in the GPTMTnMR register is set, the RTCRIS bit in the GPTMRIS register is set,
///     and the timer continues counting. In One-Shot mode, the timer stops counting after the time-out
///     event. To re-enable the timer, repeat the sequence. A timer configured in Periodic mode reloads
///     the timer and continues counting after the time-out event.
/// 
///----------------------------------------------------------------------------
void
ServoInit(void) {

#if defined(PART_LM3S9B90)
  
    //
    // Compute the timer frequency (in KHz) based on the system clock.
    //
    ulFrequency = 2850; // SysCtlClockGet() / 1000UL;

    //
    // Enable GPIO ports for servo.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

/* PWM MODE -----------------------
    //
    // Set servo pins as CCP output.
    //
    GPIOPinTypeTimer(GPIO_PORTA_BASE, GPIO_PIN_6); 
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_5); 

    GPIOPinConfigure(GPIO_PA6_CCP1);
    GPIOPinConfigure(GPIO_PB5_CCP0);

    //
    // Enable Timer 0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
    //
    // Disable timer 0
    //
    TimerDisable(TIMER0_BASE, TIMER_BOTH);

    //
    // Initialize Timer 0 as two 16 bit PWM timers
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_16_BIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM);

    //
    // Configure PWM output level
    //
    TimerControlLevel(TIMER0_BASE, TIMER_BOTH, true);

    //
    // Configure timer period
    //
    TimerLoadSet(TIMER0_BASE, TIMER_BOTH, 65535);

    TimerMatchSet(TIMER0_BASE, TIMER_BOTH, (ulFrequency * SERVO_NEUTRAL) / 1000);

    //
    // Start timers
    //
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
---------------------- PWM MODE */

    //
    // Set servo pins as outputs, SW controlled
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6); 
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5); 

    //
    // Enable Timer 0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    
    //
    // Halt timer 0
    //
    TimerDisable(TIMER0_BASE, TIMER_BOTH);

    //
    // Initialize Timer 0 as two 16 bit periodic timers
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_16_BIT_PAIR |     // 16 bit timer pair
                                TIMER_CFG_A_PERIODIC |      // A periodic timer
                                TIMER_CFG_B_PERIODIC |      // B periodic timer
                                TIMER_TAMR_TAMIE |          // compare A match
                               (TIMER_TBMR_TBMIE << 8) |    // compare B match
                                TIMER_TAMR_TACDIR |         // timer A counts up
                               (TIMER_TBMR_TBCDIR << 8));   // timer B counts up

    //
    // Configure timer prescaler for 50 / 16 = 3.125 MHz
    //
    TimerPrescaleSet(TIMER0_BASE, TIMER_BOTH, PRESCALER);

    //
    // Configure timer period
    //
    TimerLoadSet(TIMER0_BASE, TIMER_BOTH, 65535);

    //
    // Initialize duty cycle
    //
    lPositionA = (ulFrequency * SERVO_NEUTRAL) / 1000;
    lPositionB = (ulFrequency * SERVO_NEUTRAL) / 1000;
    
    //
    // Configure duty cycle
    //
    TimerMatchSet(TIMER0_BASE, TIMER_BOTH, lPositionA);

    //
    // Enable interrupts
    //
    TimerIntEnable(TIMER0_BASE, TIMER_CAPB_MATCH |      // compare B match interrupt
                                TIMER_TIMA_TIMEOUT |    // Timer A timeout interrupt
                                TIMER_CAPA_MATCH);      // compare A match interrupt

    //
    // Enable interrupt on timer A / B
    //
    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER0B);

    //
    // Start timers
    //
    TimerEnable(TIMER0_BASE, TIMER_BOTH);
#endif
}


///----------------------------------------------------------------------------
///
///  DESCRIPTION Update servo positions.
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
void 
ServoUpdate(void) {

#if defined(PART_LM3S9B90)

    long lPosition;
    lPosition = (long)ceil(Elevator() * fElevatorGain);
    lPosition += SERVO_NEUTRAL;
    if (lPosition < SERVO_MIN) { lPosition = SERVO_MIN; }
    if (lPosition > SERVO_MAX) { lPosition = SERVO_MAX; }
//    TimerMatchSet(TIMER0_BASE, TIMER_A, (ulFrequency * lPosition) / 1000);
    
    //
    // Update servo A position atomically.
    //
    IntMasterDisable();
    lPositionA = (ulFrequency * lPosition) / 1000;
    IntMasterEnable();
    
    lPosition = (long)ceil(Ailerons() * fAileronGain);
    lPosition += SERVO_NEUTRAL;
    if (lPosition < SERVO_MIN) { lPosition = SERVO_MIN; }
    if (lPosition > SERVO_MAX) { lPosition = SERVO_MAX; }
//    TimerMatchSet(TIMER0_BASE, TIMER_B, (ulFrequency * lPosition) / 1000);
    
    //
    // Update servo B position atomically.
    //
    IntMasterDisable();
    lPositionB = (ulFrequency * lPosition) / 1000;
    IntMasterEnable();
/*
    lPosition = (long)ceil(Rudder() * fRudderGain);
    lPosition += SERVO_NEUTRAL;
    if (lPosition < SERVO_MIN) { lPosition = SERVO_MIN; }
    if (lPosition > SERVO_MAX) { lPosition = SERVO_MAX; }
    TimerMatchSet(TIMER0_BASE, TIMER_B, (ulFrequency * lPosition) / 1000);
*/
#endif
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION 
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
void 
ServoIntHandlerA(void) {

    unsigned long ul_status;
    ul_status = TimerIntStatus(TIMER0_BASE, false);

    if ((ul_status & TIMER_CAPA_MATCH) == TIMER_CAPA_MATCH) {
        TimerIntClear(TIMER0_BASE, TIMER_CAPA_MATCH);       // Clear interrupt flag
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, ~(0x40)); // Clear servo A output
    } else if ((ul_status & TIMER_TIMA_TIMEOUT) == TIMER_TIMA_TIMEOUT) {
        TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);     // Clear interrupt flag
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, 0x40);    // Set both servo outputs
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0x20);    // Set both servo outputs
        TimerMatchSet(TIMER0_BASE, TIMER_A, lPositionA);
        TimerMatchSet(TIMER0_BASE, TIMER_B, lPositionB);
    }  
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION 
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
void 
ServoIntHandlerB(void) {

    TimerIntClear(TIMER0_BASE, TIMER_CAPB_MATCH);       // Clear interrupt flag
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, ~(0x20)); // Clear servo B output
}
