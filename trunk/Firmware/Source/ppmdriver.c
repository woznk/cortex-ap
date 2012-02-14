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
///  ucPulseIndex     aliases                      ulPulseBuffer[]
///
///        0          -                            channel 0 pulse
///        1          AILERON_CHANNEL              channel 1 pulse
///        2          -                            channel 2 pulse
///        3          -                            channel 3 pulse
///        4          MODE_CHANNEL                 channel 4 pulse
///        5          -                            channel 5 pulse
///        6          -                            channel 6 pulse
///        7          RC_CHANNELS                  none
///
///  Before computing time difference, overflow number is checked.
///  If 2 or more overflows occurred, an invalid pulse length is forced.
///  Overflow counter is always cleared when a pulse is detected.
///  Pulse length is saved in a temporary buffer.
///  Temporary buffer is copied into final buffer only if pulse length
///  of all channels are good and synchro pulse is good to.
///  Added counter of channel pulses with correct pulse length.
///  Counter is copied into a module variable for signal strength indication.
///
//  CHANGES added missing initialization of pulse length array
//
//============================================================================*/

#include "stm32f10x.h"
#include "led.h"
#include "freertosconfig.h"
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

#define PPM_SYNC_MIN        4999        ///< Modify according to RC type
#define PPM_SYNC_MAX        20001       ///< Modify according to RC type
#define PPM_LENGTH_MIN      899
#define PPM_LENGTH_MAX      2101
#define PPM_LENGTH_NEUTRAL  1500

#define PERIOD              65535
#define PRESCALER           23

/*----------------------------------- Macros ---------------------------------*/

#define MISSING_PULSES (cOverflowCount >= 3)

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC int8_t cPulseCount;
VAR_STATIC int8_t cSignalLevel;
VAR_STATIC int8_t cOverflowCount;
VAR_STATIC uint8_t ucPulseIndex;
VAR_STATIC uint16_t ulCaptureTime;
VAR_STATIC uint16_t ulLastCapture;
VAR_STATIC uint16_t ulPulseLength;
VAR_STATIC uint16_t ulTemp[RC_CHANNELS];
VAR_STATIC uint16_t ulPulseBuffer[RC_CHANNELS];

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
  uint8_t j;

  if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {           // Overflow interrupt
     TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
     if (!MISSING_PULSES) {
        cOverflowCount++;
     }
  }

  if (TIM_GetITStatus(TIM2, TIM_IT_CC2)) {              // Capture interrupt
     ulCaptureTime = TIM_GetCapture2 (TIM2);            // read captured time
     if (cOverflowCount < 2) {                          // at most one overflow
        ulPulseLength = ulCaptureTime - ulLastCapture;  // compute time difference
     } else {                                           // more than one overflow
        ulPulseLength = 0;                              // force invalid pulse length
     }
     ulLastCapture = ulCaptureTime;                     // now is also last edge time
     cOverflowCount = 0;                                // clear overflow condition

     switch (ucPulseIndex) {
        case RC_CHANNELS:                               // waiting sync pulse
           if (( ulPulseLength > PPM_SYNC_MIN ) &&      // sync pulse detected
               ( ulPulseLength < PPM_SYNC_MAX )) {      //
              ucPulseIndex = 0;                         // reset index
              if (cPulseCount == RC_CHANNELS) {         // all channels were good
                 for (j = 0; j < RC_CHANNELS; j++) {    // copy pulse lengths
                    ulPulseBuffer[j] = ulTemp[j];       //
                 }
              }
           }
           cSignalLevel = cPulseCount;                  // update signal level
           cPulseCount = 0;                             // clear pulse counter
        break;

        default:                                        // waiting channel pulse
           ulTemp[ucPulseIndex++] = ulPulseLength;      // save pulse length
           if ((ulPulseLength > PPM_LENGTH_MIN) &&      // good pulse length
               (ulPulseLength < PPM_LENGTH_MAX)) {
              cPulseCount++;                            // increase counter
           }
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

  cPulseCount = 0;
  ulLastCapture = 0;
  cOverflowCount = -1;
  for (ucPulseIndex = 0; ucPulseIndex < RC_CHANNELS; ucPulseIndex ++) {
    ulPulseBuffer[ucPulseIndex] = PPM_LENGTH_NEUTRAL;
  }
  ucPulseIndex = RC_CHANNELS;

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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
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

    if (cSignalLevel == 0) {
        return MODE_RTL;
    } else if ( uiWidth < 1100 ) {
        return MODE_MANUAL;
    } else if (( uiWidth > 1400 ) && ( uiWidth < 1600 )) {
        return MODE_STABILIZE;
    } else if ( uiWidth > 1900 ) {
        return MODE_AUTO;
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
    return (uint8_t) cSignalLevel;
}

