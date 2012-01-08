//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief
///  PPM input driver
///
//  CHANGES Merged overflow interrupt handler into TIM2 interrupt
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

#define SYNC   (RC_CHANNELS + 1)    // Pulse index when waiting for first pulse
#define UNSYNC (RC_CHANNELS + 2)    // Pulse index when not synchronized

#define PPM_SYNC_MIN        5000    ///< Modify according to RC type
#define PPM_SYNC_MAX        20000   ///< Modify according to RC type
#define PPM_LENGTH_MIN      900
#define PPM_LENGTH_MAX      2100
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

VAR_STATIC uint16_t ulCaptureTime;
VAR_STATIC uint16_t ulLastCapture;
VAR_STATIC uint16_t ulPulseLength;
VAR_STATIC uint16_t ulPulseBuffer[RC_CHANNELS];
VAR_STATIC uint8_t ucPulseIndex;
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
TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {            // Overflow interrupt
     TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
     if (!MISSING_PULSES) {
        cOverflowCount++;
     }
  }

  if (TIM_GetITStatus(TIM2, TIM_IT_CC2)) {               // Capture interrupt
     ulCaptureTime = TIM_GetCapture2 (TIM2);             // Store captured time
     ulPulseLength = ulCaptureTime - ulLastCapture;      // Compute time difference
     ulLastCapture = ulCaptureTime;                      // Now is also last edge time

     switch ( ucPulseIndex ) {
        case UNSYNC :                                    // NOT SYNCHRONIZED
           if (( ulPulseLength >= PPM_SYNC_MIN ) &&      // sync pulse detected
               ( ulPulseLength <= PPM_SYNC_MAX )) {      //
               ucPulseIndex = SYNC;                      // synchronized
               cOverflowCount = 0;                       // We're not in an overflow condition any more
           }
           break;

        case SYNC :                                      // SYNCHRONIZED
           if (( ulPulseLength >= PPM_LENGTH_MIN ) &&    // first channel's pulse detected
               ( ulPulseLength <= PPM_LENGTH_MAX )) {    //
               ulPulseBuffer[ 0 ] = ulPulseLength;       // pulse width is OK, save it
               ucPulseIndex = 1;                         // go for second channel
               cOverflowCount = 0;                       // We're not in an overflow condition any more
           } else {                                      // wrong pulse detected
               ucPulseIndex = UNSYNC;                    // wait for next sync pulse
           }
           break;

        case RC_CHANNELS :                               // LAST RC CHANNEL
           if (( ulPulseLength >= PPM_SYNC_MIN ) &&      // sync pulse detected
               ( ulPulseLength <= PPM_SYNC_MAX )) {      //
               ucPulseIndex++;                           // still synchronized
               cOverflowCount = 0;                       // We're not in an overflow condition any more
           } else {                                      // wrong pulse detected
               ucPulseIndex = UNSYNC;                    // wait for next sync pulse
           }
           break;

        default :                                        // CHANNELS 1 .. N - 1
           if (( ulPulseLength >= PPM_LENGTH_MIN ) &&    // good pulse detected
               ( ulPulseLength <= PPM_LENGTH_MAX )) {    //
               ulPulseBuffer[ ucPulseIndex ] = ulPulseLength; // save it
               cOverflowCount = 0;                       // We're not in an overflow condition any more
           }
           ucPulseIndex++;                               // always go for next channel
           break;
     }
     if ( MISSING_PULSES ) {                             // no pulses
         ucPulseIndex = UNSYNC;                          // wait for next sync pulse
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
  ucPulseIndex = UNSYNC;

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
PPMSignalStatus( void )
{
    if (MISSING_PULSES) {
        return PPM_SIGNAL_BAD;          // bad radio signal
    } else {
        return PPM_SIGNAL_OK;           // radio signal is OK
    }
}

