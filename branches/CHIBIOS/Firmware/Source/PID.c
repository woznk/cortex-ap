/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief PID loops
 *
 * @file
 *
 * Change
 *
 *============================================================================*/

#include "config.h"
#include "pid.h"

/*--------------------------------- Definitions ------------------------------*/

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

/*---------------------------------- Functions -------------------------------*/


/*----------------------------------------------------------------------------
 *
 * @brief   PID initialization
 * @param   pxPid = pointer to PID structure
 * @return  -
 * @remarks
 *
 *---------------------------------------------------------------------------*/
void PID_Init(xPID * pxPid)
{
    pxPid->fLastInput = 0.0f;
    pxPid->fIntegral = 0.0f;
}

/*----------------------------------------------------------------------------
 *
 * @brief   PID computing
 * @param   pxPid = pointer to PID structure
 * @return  PID output
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
float PID_Compute(xPID * pxPid)
{
   float fError, fDelta, fOutput;

    /* Compute error */
    fError = pxPid->fSetpoint - pxPid->fInput;

    /* Compute integral term */
    pxPid->fIntegral += (fError * DELTA_T);

    /* Avoid windup */
    if (pxPid->fIntegral > pxPid->fMax) {
       pxPid->fIntegral = pxPid->fMax;
    }
    if (pxPid->fIntegral < pxPid->fMin) {
       pxPid->fIntegral = pxPid->fMin;
    }

    /* Compute differential term */
    /* Multiply by SAMPLES_PER_SECOND instead of dividing by DELTA_T. */
    fDelta = (pxPid->fInput - pxPid->fLastInput) * SAMPLES_PER_SECOND;

    /* Compute output */
    fOutput = pxPid->fKp * fError +
              pxPid->fKi * pxPid->fIntegral -
              pxPid->fKd * fDelta;

    /* Saturate output */
    if (fOutput > pxPid->fMax) {
       fOutput = pxPid->fMax;
    } else if (fOutput < pxPid->fMin) {
       fOutput = pxPid->fMin;
    } else {
    }

    /* Multiply by output gain */
    fOutput = pxPid->fGain * fOutput;

    /* Store current input */
    pxPid->fLastInput = pxPid->fInput;

    return fOutput;
}

