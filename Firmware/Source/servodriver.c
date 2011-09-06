//============================================================================+
//
// $RCSfile: servodriver.c,v $ (SOURCE FILE)
// $Revision: 1.5 $
// $Date: 2009/10/26 22:09:22 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             Servo driver
//
//  CHANGES     long ulPosition renamed lPosition
//              
//              
//
//============================================================================*/

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

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

#define SERVO_FREQUENCY 50

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC unsigned long ulPeriod;

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
///  DESCRIPTION PWM timers initialization
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
void
ServoInit(void)
{
    //
    // Enable PWM
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);

    //
    // Enable I/O port G and H.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);

    //
    // Set PORTH.0, PORTH.1 as PWM2, PWM3 pins
    //
    GPIOPinTypePWM(GPIO_PORTH_BASE, GPIO_PIN_0); 
/*    GPIOPinTypePWM(GPIO_PORTH_BASE, GPIO_PIN_1); */

    //
    // Configure PWM clock as system clock / 4
    //
    SysCtlPWMClockSet(SYSCTL_PWMDIV_4);

    //
    // Compute the PWM period based on the system clock.
    //
    ulPeriod = SysCtlClockGet() / (SERVO_FREQUENCY * 4);

    //
    // Set the PWM period to 440 (A) Hz.
    //
    PWMGenConfigure(PWM_BASE, PWM_GEN_1,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_1, ulPeriod);

    //
    // Set PWM2, PWM3 to a duty cycle of 7.5%.
    //
    PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, (ulPeriod * 3) / 40); 
/*    PWMPulseWidthSet(PWM_BASE, PWM_OUT_3, (ulPeriod * 3) / 40); */

    //
    // Enable PWM2, PWM3 output signals.
    //
    PWMOutputState(PWM_BASE, PWM_OUT_2_BIT, true); 
/*    PWMOutputState(PWM_BASE, PWM_OUT_3_BIT, true); */

    //
    // Enable the PWM generators.
    //
    PWMGenEnable(PWM_BASE, PWM_GEN_1);

}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Set position of n-th servo
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
void
ServoSetPosition(unsigned char ucServo, long lPosition)
{
    if (lPosition < -500)
    {
        lPosition = -500;
    }
    if (lPosition > 500)
    {
        lPosition = 500;
    }
    lPosition += 1500;
    switch (ucServo)
    {
/*
        case 1:
            PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ((ulPeriod * ulPosition) / 20000));
            break;

        case 2:
            PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, ((ulPeriod * ulPosition) / 20000));
            break;
*/
        case 3:
            PWMPulseWidthSet(PWM_BASE, PWM_OUT_2, ((ulPeriod * lPosition) / 20000));
            break;

        default:
            break;
    }
}

