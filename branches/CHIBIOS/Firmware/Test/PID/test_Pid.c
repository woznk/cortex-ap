/*============================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date: $
 * $Author: $
 *
 * @brief PID test program
 *
 * Change:
 *
 *============================================================================*/

#include <stdint.h>

#include "config.h"
#include "pid.h"

/** @addtogroup test
  * @{
  */

/** @addtogroup pid
  * @{
  */

/*--------------------------------- Definitions ------------------------------*/

#define KP_ONLY_CYCLES   10
#define KI_ONLY_CYCLES   500
#define KD_ONLY_CYCLES   10

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

static xPID Test_Pid;
float fIn, fOut, fSet;

/*--------------------------------- Prototypes -------------------------------*/

/*--------------------------------- Functions --------------------------------*/


/*----------------------------------------------------------------------------
 *
 * @brief   main
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
int32_t main(void)
{
    uint16_t j;

    Test_Pid.fGain = 1.0f;  /*  */
    Test_Pid.fMin = -1.0f;  /*  */
    Test_Pid.fMax = 1.0f;   /*  */

    /* Proportional only gain */
    Test_Pid.fKp = 1.0f;    /* init gains  */
    Test_Pid.fKi = 0.0f;    /*  */
    Test_Pid.fKd = 0.0f;    /*  */
    PID_Init(&Test_Pid);    /* initialize PID */

    fSet = 0.0f;
    for (j = 0; j < KP_ONLY_CYCLES; j++) {          /* settle */
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.1f;                                     /* 0 -> 1 step */
    for (j = 0; j < KP_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     /* 1 -> 0 step */
    for (j = 0; j < KP_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = -0.1f;                                    /* 0 -> -1 step */
    for (j = 0; j < KP_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.1f;                                     /* -1 -> 1 step */
    for (j = 0; j < KP_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     /* 1 -> 0 step */
    for (j = 0; j < KP_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }

    /* Integral only gain */
    Test_Pid.fKp = 0.0f;    /* init gains  */
    Test_Pid.fKi = 1.0f;    /*  */
    Test_Pid.fKd = 0.0f;    /*  */
    PID_Init(&Test_Pid);    /* initialize PID */

	fSet = 0.1f;                                     /* 0 -> 1 step */
    for (j = 0; j < KI_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     /* 1 -> 0 step */
    for (j = 0; j < KI_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = -0.1f;                                    /* 0 -> -1 step */
    for (j = 0; j < KI_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.1f;                                     /* -1 -> 1 step */
    for (j = 0; j < KI_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     /* 1 -> 0 step */
    for (j = 0; j < KI_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }

    /* Derivative only gain */
    Test_Pid.fKp = 0.0f;    /* init gains  */
    Test_Pid.fKi = 0.0f;    /*  */
    Test_Pid.fKd = 1.0f;    /*  */
    PID_Init(&Test_Pid);    /* initialize PID */

    fSet = 0.0f;
    for (j = 0; j < KD_ONLY_CYCLES; j++) {                       /* settle */
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.1f;                                     /* 0 -> 1 step */
    for (j = 0; j < KD_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     /* 1 -> 0 step */
    for (j = 0; j < KD_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = -0.1f;                                    /* 0 -> -1 step */
    for (j = 0; j < KD_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.1f;                                     /* -1 -> 1 step */
    for (j = 0; j < KD_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }
	fSet = 0.0f;                                     /* 1 -> 0 step */
    for (j = 0; j < KD_ONLY_CYCLES; j++) {
       fOut = PID_Compute(&Test_Pid, 0.0f, fSet);
    }

    while (1) {
	}

}

/**
  * @}
  */

/**
  * @}
  */

/*****END OF FILE****/
