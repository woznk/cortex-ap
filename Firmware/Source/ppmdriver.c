//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief
///  PPM input driver
/// \file
///                                                 content of
///     ucPulseIndex   aliases                      ulPulseBuffer[]
///
///         0          SYNC                         channel 0 pulse
///         1          AILERON_CHANNEL              channel 1 pulse
///         2          -                            channel 2 pulse
///         3          -                            channel 3 pulse
///         4          MODE_CHANNEL                 channel 4 pulse
///         5          -                            channel 5 pulse
///         6          -                            channel 6 pulse
///         7          RC_CHANNELS / WAITING_SYNC   none
///
//  CHANGES Checked overflow number before computing time difference.
//          Forced invalid pulse length when > 2 overflows occurred.
//          Overflow counter always cleared when a pulse is detected.
//          Added counter of good pulses for signal strength indication.
//          Simplified capture interrupt routine.
//
//============================================================================*/

#include "stm32f10x.h"
#include "led.h"
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

#define SYNC                0           // Index when waiting for first pulse
#define WAITING_SYNC        RC_CHANNELS // Index when waiting for synchronization

#define PPM_SYNC_MIN        5000        ///< Modify according to RC type
#define PPM_SYNC_MAX        20000       ///< Modify according to RC type
#define PPM_LENGTH_MIN      900
#define PPM_LENGTH_MAX      2100
#define PPM_LENGTH_NEUTRAL  1500

#define PERIOD              65535
#define PRESCALER           23
#define MAX_STRENGTH        20          //
#define LOW_STRENGTH        5
#define HIGH_STRENGTH       15

/*----------------------------------- Macros ---------------------------------*/

#define MISSING_PULSES (cOverflowCount >= 3)

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC uint16_t ulCaptureTime;
VAR_STATIC uint16_t ulLastCapture;
VAR_STATIC uint16_t ulPulseLength;
VAR_STATIC uint16_t ulPulseBuffer[RC_CHANNELS];
VAR_STATIC uint8_t ucPulseIndex;
VAR_STATIC uint8_t ucSignalStrength;
VAR_STATIC int8_t cOverflowCount;

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
TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {            // Overflow interrupt
     TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
     if (!MISSING_PULSES) {
        cOverflowCount++;
     }
  }

  if (TIM_GetITStatus(TIM2, TIM_IT_CC2)) {               // Capture interrupt
     ulCaptureTime = TIM_GetCapture2 (TIM2);             // read captured time
     if (cOverflowCount <= 1) {                          // at most one overflow
        ulPulseLength = ulCaptureTime - ulLastCapture;   // compute time difference
     } else {                                            // more than one overflow
        ulPulseLength = 0;                               // force invalid pulse length
        ucSignalStrength = 0;                            // no signal
     }
     ulLastCapture = ulCaptureTime;                      // now is also last edge time
     cOverflowCount = 0;                                 // clear overflow condition

     switch ( ucPulseIndex ) {
        case WAITING_SYNC :                              // WAITING FOR SYNCHRONIZATION
           if (( ulPulseLength >= PPM_SYNC_MIN ) &&      // sync pulse detected
               ( ulPulseLength <= PPM_SYNC_MAX )) {      //
               ucPulseIndex = 0;                         // synchronized
               ucSignalStrength++;                       // increase signal strength
               ucSignalStrength %= MAX_STRENGTH;         //
           } else if (ucSignalStrength > 0) {            // bad pulse detected
               ucSignalStrength--;                       // decrease signal strength
           }
           break;

        default :                                        // CHANNELS 1 .. N - 1
           if (( ulPulseLength >= PPM_LENGTH_MIN ) &&    // good pulse detected
               ( ulPulseLength <= PPM_LENGTH_MAX )) {    //
               ulPulseBuffer[ucPulseIndex] = ulPulseLength; // save it
               ucSignalStrength++;                       // increase signal strength
               ucSignalStrength %= MAX_STRENGTH;         //
           } else if (ucSignalStrength > 0) {            // bad pulse detected
               ucSignalStrength--;                       // decrease signal strength
           }
           ucPulseIndex++;                               // always go for next channel
           break;
     }
  }
}


///----------------------------------------------------------------------------
///
///  DESCRIPTION PPM timer initialization
/// \brief   Timer 2 initialization for RRC input capture
/// \return  -
/// \remarks TIM2CLK = 24 MHz, Prescaler = 23, TIM2 counter clock = 1 MHz
///
///----------------------------------------------------------------------------
void PPM_Init(void) {

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  ulLastCapture = 0;
  cOverflowCount = -1;
  ucSignalStrength = 0;
  ucPulseIndex = WAITING_SYNC;

  /* Clear current configuration */
  TIM_DeInit(TIM2);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PERIOD;
  TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

  /* Input capture configuration: channel 2 */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);

  /* Capture and update interrupts initialization */
  TIM_ClearFlag(TIM2, TIM_FLAG_CC2 | TIM_IT_Update);
  TIM_ITConfig(TIM2, TIM_IT_CC2 | TIM_IT_Update, ENABLE);

  /* Enable the TIM2 global interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure auto reload register */
  TIM_ARRPreloadConfig(TIM2, ENABLE);

  /* Enable compare channel 2 */
  TIM_CCxCmd(TIM2, TIM_Channel_2, ENABLE);

  /* TIM2 enable counter */
  TIM_Cmd(TIM2, ENABLE);
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Get value of n-th radio channel
/// \RETURN      -
/// \REMARKS
///
///----------------------------------------------------------------------------
uint16_t PPMGetChannel(uint8_t ucChannel)
{
    if ( ucChannel < RC_CHANNELS ) {
        return ulPulseBuffer[ ucChannel ];
    } else {
        return PPM_LENGTH_NEUTRAL;
    }
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Get mode from MODE_CHANNEL
/// \RETURN      -
/// \REMARKS
///
///----------------------------------------------------------------------------
uint8_t PPMGetMode(void)
{
    uint16_t uiWidth;
    uiWidth = ulPulseBuffer[MODE_CHANNEL];

    if ( uiWidth < 1100 ) {
        return MODE_STABILIZE;
    } else if (( uiWidth > 1400 ) && ( uiWidth < 1600 )) {
        return MODE_AUTO;
    } else if ( uiWidth > 1900 ) {
        return MODE_MANUAL;
    } else {
        return MODE_UNDEFINED;
    }
}

//----------------------------------------------------------------------------
//
//  DESCRIPTION Returns status of radio signal.
///
/// \remarks   -
///
///----------------------------------------------------------------------------
uint8_t PPMSignalStatus(void)
{
    if (ucSignalStrength > HIGH_STRENGTH) {
        return PPM_SIGNAL_OK;                      // radio signal is OK
    } else if (ucSignalStrength > LOW_STRENGTH) {
        return PPM_SIGNAL_BAD;                     // bad radio signal
    } else {
        return PPM_NO_SIGNAL;                      // no radio signal
    }
}

