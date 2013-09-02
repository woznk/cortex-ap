//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief PPM input driver
///
/// \file
///                                                 content of
///  ucPulseIndex     aliases                      uiPulseBuffer[]
///
///        0          see ppmdriver.h              channel 0 pulse
///        1          see ppmdriver.h              channel 1 pulse
///        2          see ppmdriver.h              channel 2 pulse
///        3          see ppmdriver.h              channel 3 pulse
///        4          see ppmdriver.h              channel 4 pulse
///        5          see ppmdriver.h              channel 5 pulse
///        6          see ppmdriver.h              channel 6 pulse
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
//  Change (Lint) corrected file #inclusion, removed #undef and VAR_GLOBAL,
//         PERIOD renamed PPM_PERIOD
//
//============================================================================*/

#include "stm32f10x_tim.h"

#include "config.h"
#include "led.h"
#include "freertosconfig.h"
#include "ppmdriver.h"

/*--------------------------------- Definitions ------------------------------*/

#ifndef VAR_STATIC
#define VAR_STATIC static
#endif

#define PPM_SYNC_MIN        4999    ///< min length of sync pulse, modify according to RC type
#define PPM_SYNC_MAX        20001   ///< max length of sync pulse, modify according to RC type
#define PPM_PULSE_MIN       900     ///< length of channel pulse for full scale low
#define PPM_PULSE_MAX       2100    ///< length of channel pulse for full scale high
#define PPM_PULSE_NEUTRAL   1500    ///< length of channel pulse for neutral position

#define PPM_PERIOD          65535   ///< capture timer period
#define PRESCALER           23      ///< capture timer prescaler

/*----------------------------------- Macros ---------------------------------*/

#define MISSING_PULSES (cOverflowCount >= 3)    ///< missing pulses condition

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC int8_t cPulseCount;                  ///< counter of sync pulses
VAR_STATIC int8_t cSignalLevel;                 ///< signal level estimation
VAR_STATIC int8_t cOverflowCount;               ///< counter of timer overflows
VAR_STATIC uint8_t ucPulseIndex;                ///< channel (pulse) index
VAR_STATIC uint16_t uiCaptureTime;              ///< captured timer value
VAR_STATIC uint16_t uiLastCapture;              ///< last captured timer value
VAR_STATIC uint16_t uiPulseLength;              ///< length of channel pulse
VAR_STATIC uint16_t uiTemp[RC_CHANNELS];        ///< temporary for channel pulses
VAR_STATIC uint16_t uiPulseBuffer[RC_CHANNELS]; ///< rc channel pulses
VAR_STATIC int16_t iReverse[RC_CHANNELS] = {    ///< channel reverse
        //  # | TYCHO    | LEUKO    | EASYSTAR | EPPFPV
        // ---+----------+----------+----------+----------
     1, //  0 | aileron  | delta 1  | throttle | aileron
     1, //  1 | elevator | delta 2  | aileron  | elevator
     1, //  2 | throttle | throttle | elevator | throttle
     1, //  3 | rudder   | rudder   | rudder   | rudder
    -1, //  4 | kp       | mode     | mode     | mode
     1, //  5 | ki       | kp       | -        | -
     1  //  6 | -        | ki       | kp       | kp
};

/*--------------------------------- Prototypes -------------------------------*/

/*---------------------------------- Functions -------------------------------*/

///----------------------------------------------------------------------------
///
///  DESCRIPTION The PPM input capture interrupt handler.
/// \return      -
/// \remarks
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
     uiCaptureTime = TIM_GetCapture2 (TIM2);            // read captured time
     TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
     if (cOverflowCount < 2) {                          // at most one overflow
        uiPulseLength = uiCaptureTime - uiLastCapture;  // compute time difference
     } else {                                           // more than one overflow
        uiPulseLength = 0;                              // force invalid pulse length
     }
     uiLastCapture = uiCaptureTime;                     // now is also last edge time
     cOverflowCount = 0;                                // clear overflow condition

     switch (ucPulseIndex) {
        case RC_CHANNELS:                               // waiting sync pulse
           if (( uiPulseLength > PPM_SYNC_MIN ) &&      // sync pulse detected
               ( uiPulseLength < PPM_SYNC_MAX )) {      //
              ucPulseIndex = 0;                         // reset index
              if (cPulseCount == RC_CHANNELS) {         // all channels were good
                 for (j = 0; j < RC_CHANNELS; j++) {    // copy pulse lengths
                    uiPulseBuffer[j] = uiTemp[j];       //
                 }
              }
           }
           cSignalLevel = cPulseCount;                  // update signal level
           cPulseCount = 0;                             // clear pulse counter
        break;

        default:                                        // waiting channel pulse
           uiTemp[ucPulseIndex++] = uiPulseLength;      // save pulse length
           if ((uiPulseLength > PPM_PULSE_MIN) &&       // good pulse length
               (uiPulseLength < PPM_PULSE_MAX)) {
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
  uiLastCapture = 0;
  cOverflowCount = -1;
  for (ucPulseIndex = 0; ucPulseIndex < RC_CHANNELS; ucPulseIndex ++) {
    uiPulseBuffer[ucPulseIndex] = PPM_PULSE_NEUTRAL;
  }
  ucPulseIndex = RC_CHANNELS;

  /* Clear current configuration */
  TIM_DeInit(TIM2);

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = PPM_PERIOD;
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
  TIM_ClearFlag(TIM2, TIM_FLAG_CC2 | TIM_FLAG_Update);
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
///  DESCRIPTION Get position of n-th radio channel
/// \return      Pulse length of n-th radio channel in microsecond
/// \remarks
///
///----------------------------------------------------------------------------
int16_t PPMGetChannel(uint8_t ucChannel)
{
    int16_t position;
    if ( ucChannel < RC_CHANNELS ) {
        position = (int16_t)uiPulseBuffer[ucChannel];
        position -= PPM_PULSE_NEUTRAL;
        position *= iReverse[ucChannel];
        position += PPM_PULSE_NEUTRAL;
    } else {
        position = PPM_PULSE_NEUTRAL;
    }
	return position;
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Get mode from MODE_CHANNEL
/// \return      -
/// \remarks
///
///----------------------------------------------------------------------------
uint8_t PPMGetMode(void)
{
    uint16_t uiWidth;
    uiWidth = uiPulseBuffer[MODE_CHANNEL];

    if (cSignalLevel == 0) {
        return MODE_RTL;
    } else if ( uiWidth < 1100 ) {
        return MODE_MANUAL;
    } else if (( uiWidth > 1400 ) && ( uiWidth < 1600 )) {
        return MODE_STAB;
    } else if ( uiWidth > 1900 ) {
        return MODE_NAV;
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

