//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief test program
///
// Change: prefixes MSP_ replaced with MWI_
//
//============================================================================*/

#include <stdint.h>

#include "stm32f10x.h"

#include "config.h"
#include "usart1driver.h"
#include "pid.h"

/** @addtogroup test
  * @{
  */

/** @addtogroup telemetry
  * @{
  */

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#define KP_ONLY_CYCLES   10
#define KI_ONLY_CYCLES   500
#define KD_ONLY_CYCLES   10


/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC xPID Test_Pid;
float fIn, fOut, fSet;

/*--------------------------------- Prototypes -------------------------------*/

/*--------------------------------- Functions --------------------------------*/


///----------------------------------------------------------------------------
///
/// \brief   main
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
int32_t main(void)
{
    uint16_t j; 

    Test_Pid.fGain = 1.0f;  // 
    Test_Pid.fMin = -1.0f;  //
    Test_Pid.fMax = 1.0f;   //

    /* Proportional only gain */
    Test_Pid.fKp = 1.0f;    // init gains 
    Test_Pid.fKi = 0.0f;    //
    Test_Pid.fKd = 0.0f;    //
    PID_Init(&Test_Pid);    // initialize PID

    fSet = 0.0f;
    for (j = 0; j < KP_ONLY_CYCLES; j++) {                       // settle
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.1f;                                     // 0 -> 1 step
    for (j = 0; j < KP_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     // 1 -> 0 step
    for (j = 0; j < KP_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = -0.1f;                                    // 0 -> -1 step
    for (j = 0; j < KP_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.1f;                                     // -1 -> 1 step
    for (j = 0; j < KP_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     // 1 -> 0 step
    for (j = 0; j < KP_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }

    /* Integral only gain */
    Test_Pid.fKp = 0.0f;    // init gains 
    Test_Pid.fKi = 1.0f;    //
    Test_Pid.fKd = 0.0f;    //
    PID_Init(&Test_Pid);    // initialize PID

	fSet = 0.1f;                                     // 0 -> 1 step
    for (j = 0; j < KI_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     // 1 -> 0 step
    for (j = 0; j < KI_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = -0.1f;                                    // 0 -> -1 step
    for (j = 0; j < KI_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.1f;                                     // -1 -> 1 step
    for (j = 0; j < KI_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     // 1 -> 0 step
    for (j = 0; j < KI_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }

    /* Derivative only gain */
    Test_Pid.fKp = 0.0f;    // init gains 
    Test_Pid.fKi = 0.0f;    //
    Test_Pid.fKd = 1.0f;    //
    PID_Init(&Test_Pid);    // initialize PID

    fSet = 0.0f;
    for (j = 0; j < KD_ONLY_CYCLES; j++) {                       // settle
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.1f;                                     // 0 -> 1 step
    for (j = 0; j < KD_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     // 1 -> 0 step
    for (j = 0; j < KD_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = -0.1f;                                    // 0 -> -1 step
    for (j = 0; j < KD_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.1f;                                     // -1 -> 1 step
    for (j = 0; j < KD_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     // 1 -> 0 step
    for (j = 0; j < KD_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }

    while (1) {
	}

}

#ifdef  USE_FULL_ASSERT
///----------------------------------------------------------------------------
///
/// \brief   Reports the name of the source file and the source line number
///          where the assert_param error has occurred.
/// \param   file: pointer to the source file name
/// \param   line: assert_param error line source number
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/*****END OF FILE****/
