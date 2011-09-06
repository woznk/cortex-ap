//============================================================================+
//
// $RCSfile: adcdriver.c,v $ (SOURCE FILE)
// $Revision: 1.9 $
// $Date: 2009/10/31 15:10:39 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             adc driver
//              ADC 0 : accel X
//              ADC 1 : accel Z
//              ADC 2 : accel Y
//              ADC 3 : gyro X
//              ADC 4 : gyro Y
//              ADC 5 : gyro Z
//                                                    SLP  V+  X   Y   Z  GND
//                        +-----------------+       +-----------------------+
//                        |   +-------+   o | ST    |                       |
//                        |   | LISY  |   o | PD    |  O   O   O   O   O   O|
//                        |   | 300   |   o | OUT   |                       |
//                        |   | AL    |   o | GND   |                       |
//                        |   +-------+   o | 3V3   |                       |
//                        +-----------------+       |                       |
//                                                  |                       |
//                   +----------------------+       |                       |
//                   |                      |       |                       |
//               3V3 | o  +-------+       o | Y     |                       |
//               GND | o  |  LPR  |  +-+  o | X     |                       |
//                4Y | o  |  530  |  | |  o | ST    |       +-------+       |
//                4X | o  |  AL   |  +-+  o | PD    |       |  MMA  |       |
//              VREF | o  +-------+       o | HP    |       |  7260 |       |
//                   |                      |       |       |  QT   |       |
//                   +----------------------+       |       +-------+       |
//                             -         -          |                       |
//                     Y+    /       Z+    \        |                       |
//                     <----|------(x)      | YAW+  |                       |
//                           \      |      /        |                       |
//                             - >  |  < -          |                       |
//                        PITCH+    |               |                       |
//                                  |   ^           +-----------------------+
//                              \   |   / ROLL+  
//                                -___-                  
//                                  |
//                                  |
//                                  V  X+
//              
//  CHANGES     corrected naming of reference axes
//
//============================================================================*/

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "adcdriver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

unsigned char ucADCsample;              // Sample counter
unsigned long ulSeq1DataBuffer[8];      // Sample buffer
unsigned long ulFiltDataBuffer[8];      // Filtered data buffer

//
// Used to change the polarity of the sensors
//
long SensorSign[] = {
   1, // ADC 0 : accel X
  -1, // ADC 1 : accel Z
   1, // ADC 2 : accel Y
   1, // ADC 3 : gyro X 
  -1, // ADC 4 : gyro Y 
   1, // ADC 5 : gyro Z 
};

//
// Sensor offsets
//
long SensorOffset[6];         

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
///  DESCRIPTION ADC initialization
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
void
ADCInit(void)
{
    //
    // Enable Timer 0 (ADC trigger)
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); 

    //
    // Initialize Timer 0 
    //
    TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);

    //
    // trigger an ADC conversion once every x milliseconds
    //
    TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / SAMPLES_PER_SECOND);
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);

    //
    // Enable ADC
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC);

    //
    // Enable 8 x oversample
    //
    ADCHardwareOversampleConfigure(ADC_BASE, 8);

    //
    // ADC sample at 250KSps
    //
    SysCtlADCSpeedSet(SYSCTL_ADCSPEED_250KSPS);

    //
    // Disable sample sequence 0
    //
    ADCSequenceDisable(ADC_BASE, 0);

    //
    // sample sequence 0: timer trigger, priority = 0
    //
    ADCSequenceConfigure(ADC_BASE, 0, ADC_TRIGGER_TIMER, 0);

    //
    // sample seq step 0
    //
    ADCSequenceStepConfigure(ADC_BASE, 0, 0, ADC_CTL_CH0);

    //
    // sample seq step 1
    //
    ADCSequenceStepConfigure(ADC_BASE, 0, 1, ADC_CTL_CH1);

    //
    // sample seq step 2
    //
    ADCSequenceStepConfigure(ADC_BASE, 0, 2, ADC_CTL_CH2);

    //
    // sample seq step 3
    //
    ADCSequenceStepConfigure(ADC_BASE, 0, 3, ADC_CTL_CH3);

    //
    // sample seq step 4
    //
    ADCSequenceStepConfigure(ADC_BASE, 0, 4, ADC_CTL_CH4);

    //
    // sample seq step 5
    //
    ADCSequenceStepConfigure(ADC_BASE, 0, 5, ADC_CTL_CH5 | ADC_CTL_IE | ADC_CTL_END);

    //
    // Enable sample sequence 0
    //
    ADCSequenceEnable(ADC_BASE, 0);

    //
    // Enable interrupt for sample sequence 0
    //
    IntEnable(INT_ADC0);
    ADCIntEnable(ADC_BASE, 0);

    //
    // Enable timer and start conversion
    //
    TimerEnable(TIMER0_BASE, TIMER_A);

    //
    // Clear ADC sample counter
    //
    ucADCsample = 0;
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION The ADC sample sequence 1 interrupt handler.
/// \RETURN      -
/// \REMARKS     
///
///----------------------------------------------------------------------------
void 
ADCS0IntHandler(void)
{
    //
    // Clear the ADC interrupt
    //
    ADCIntClear(ADC_BASE, 0);

    //
    // Check if overflow occurred.
    //
    if (ADCSequenceOverflow(ADC_BASE, 0) != 0)
    {
        //
        // Clear overflow.
        //
        ADCSequenceOverflowClear(ADC_BASE, 0);
    }

    //
    // Check if underflow occurred.
    //
    if (ADCSequenceUnderflow(ADC_BASE, 0) != 0)
    {
        //
        // Clear underflow.
        //
        ADCSequenceUnderflowClear(ADC_BASE, 0);
    }

    //
    // Filter data from previous sample sequence
    //
    for (int ulCnt = 0; ulCnt < 6; ulCnt++)
    {
        ulFiltDataBuffer[ulCnt] += (9 * ulSeq1DataBuffer[ulCnt]);
        ulFiltDataBuffer[ulCnt] /= 10;
    }

    //
    // Retrieve data from sample sequence
    //
    ADCSequenceDataGet(ADC_BASE, 0, ulSeq1DataBuffer);

    //
    // Increase sample counter
    //
    ucADCsample ++;
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Wait ADC input settling and get sensor's offset
/// \RETURN      -
/// \REMARKS     We must consider gravity sensed by Z accelerometer by adding
///              or subtracting GRAVITY to its offset.
///
///----------------------------------------------------------------------------
int
ADCSettled(void)
{
    int c;
    
    // 
    // Wait some ADC conversions
    //
    if (ucADCsample == 100)
    {
        //
        // Get offset
        // 
        for (c = 0; c < 6; c++)
        {
            SensorOffset[c] = (long)ulFiltDataBuffer[c];
        }

        //
        // Subtract gravity from Z axis
        // 
        SensorOffset[1] += GRAVITY;

        return 1;
    }
    else
    {
        return 0;
    }
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Interface to ADC sample counter.
/// \RETURN      Number of samples modulo 256
/// \REMARKS     
///
///----------------------------------------------------------------------------
unsigned char
ADCSamples(void)
{
    return ucADCsample;
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Interface to sensor data.
/// \RETURN      offset- and sign-corrected value of n-th sensor
/// \REMARKS     
///
///----------------------------------------------------------------------------
float
ADCGetData(int n)
{
    long lTemp;

    switch (n)
    {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
            lTemp = ((long)ulFiltDataBuffer[n] - SensorOffset[n]) * SensorSign[n];
            break;

        default:
            lTemp = 0;
            break;
    }
    return (float) lTemp;
}
