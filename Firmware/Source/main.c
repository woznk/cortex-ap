//============================================================================+
//
// $RCSfile: main.c,v $ (SOURCE FILE)
// $Revision: 1.19 $
// $Date: 2011/01/23 17:54:06 $
// $Author: Lorenz $
//
/// \brief IMU + AHRS based on Luminary LM3S1968 / LM3S9B90 evaluation boards
///
/// \file
///
//  CHANGES Simulation.h sostituito da Telemetry.h
//
//============================================================================*/

#include "math.h"
#include "inc/hw_ints.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"

#include "config.h"
#include "log.h"
#include "DCM.h"
#include "gps.h"
#include "nav.h"
#include "tick.h"
#include "diskio.h"
#include "adcdriver.h"
#include "ppmdriver.h"
#include "uartdriver.h"
#include "telemetry.h"
#include "servodriver.h"
#include "aileronctrl.h"
#include "elevatorctrl.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#if defined(PART_LM3S1968)
#   define LED_ON()     GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2,   0x04)
#   define LED_OFF()    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, ~(0x04))
#   define LED_TOGGLE() GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_2, \
                        GPIOPinRead(GPIO_PORTG_BASE, GPIO_PIN_2) ^ (0x04))
#elif defined(PART_LM3S9B90)
#   define LED_ON()     GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0,   0x01)
#   define LED_OFF()    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, ~(0x01))
#   define LED_TOGGLE() GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, \
                        GPIOPinRead(GPIO_PORTD_BASE, GPIO_PIN_0) ^ (0x01))
#else
#   error Microcontroller type not defined
#endif

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

#ifdef DEBUG
///----------------------------------------------------------------------------
///
/// Error routine
/// \return  -
/// \remarks It is called if the driver library encounters an error.
///
///
///----------------------------------------------------------------------------
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif


///----------------------------------------------------------------------------
///
///  DESCRIPTION main program
/// \return      -
/// \remarks
///
///    rmat[0] rmat[1] rmat[2]       0          0          0
///    rmat[3] rmat[4] rmat[5] =     0      cos(Ya^Ye)     0
///    rmat[6] rmat[7] rmat[8]   cos(Xa^Ze) cos(Ya^Ze)     0
///
///    rmat[4] = cosine of the angle between aircraft Y axis and earth Y axis.
///    It is equal to one when aircraft Y axis is aligned with earth Y axis.
///
///    rmat[6] = cosine of the angle between aircraft X axis and earth Z axis.
///    It is equal to zero when aircraft X axis is level with earth XY plane.
///
///    rmat[7] = cosine of the angle between aircraft Y axis and earth Z axis.
///    It is equal to zero when aircraft Y axis is level with earth XY plane.
///
///----------------------------------------------------------------------------
int
main(void)
{
    //
    // Set the clocking source.
    //
#if defined(PART_LM3S1968)
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);
#elif defined(PART_LM3S9B90)
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
#else
#error Microcontroller type not defined
#endif

    //
    // Enable GPIO port for LED.
    //
#if defined(PART_LM3S1968)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);  // GPIO G
#elif defined(PART_LM3S9B90)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  // GPIO D
#else
#error Microcontroller type not defined
#endif

    //
    // Set LED pin as output, SW controlled.
    //
#if defined(PART_LM3S1968)
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_2);
#elif defined(PART_LM3S9B90)
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);
#else
#error Microcontroller type not defined
#endif

    //
    // Enable SSI 0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

    //
    // Initialize drivers
    //
    UARTInit();             // UART 0, 1
    ADCInit();              // Sensors (ADC)
    ServoInit();            // Servos (PWM)
    PPMInit();              // PPM input (Timer 1)
    TickInit();             // Push buttons and disktimerproc (System tick)
    GPSInit();              // GPS

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Initialize state machines.
    //
    while (Nav_Init() == false);  // Navigation
    Log_Init();                   // Logging

    //
    // Wait for sensor input settling
    //
#if (SIMULATOR == SIM_NONE)
    while (ADCSettled() == false) {
    }
#else
    while (Sim_Settled() == false) {
      Telemetry_Parse();
    }
#endif

    //
    // Throw away any button presses that may have occurred so far.
    //
    HWREGBITW(&g_ulFlags, FLAG_BUTTON_PRESS) = 0;

    //
    // Loop forever.
    //
    while ( 1 ) {

        //
        // Wait until a 10 ms Tick has occurred.
        //
        if (HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK_10)) {

            //
            // Clear the 10 ms Tick flag.
            //
            HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK_10) = 0;

            //
            // Call the FatFs tick timer.
            //
            disk_timerproc();
        }

        //
        // Wait until a 20 ms Tick has occurred.
        //
        if (HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK_20)) {

            //
            // Clear the 20 ms Tick flag.
            //
            HWREGBITW(&g_ulFlags, FLAG_CLOCK_TICK_20) = 0;

            //
            // Toggle green LED.
            //
            LED_TOGGLE();

            //
            // Update logic and I/O.
            //
            Logic();

            //
            // Actual IMU and AHRS computation.
            //
            MatrixUpdate();       //
            CompensateDrift();    //
            Normalize();          //
            Aileron_Control();    //
            Elevator_Control();   //

            //
            // Send DCM to serial.
            //
            Log_DCM();            //

            //
            // Update aircraft controls
            //
            Telemetry_Send_Controls();  // Update simulator controls
            ServoUpdate();              // Update servos
        }

        //
        // Navigation
        //
#if (SIMULATOR == SIM_NONE)
        if (GPSParse()) {               // Parse GPS sentence
            Navigate();                 // Compute direction
            Telemetry_Send_Waypoint();  // Update waypoint number
        }
        Telemetry_Parse();              // Parse telemetry data
#else
        if (Telemetry_Parse()) {        // Parse telemetry data
            Navigate();                 // Compute direction
            Telemetry_Send_Waypoint();  // Update waypoint number
        }
#endif
    }
}

