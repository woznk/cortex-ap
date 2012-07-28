//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief PID controls
/// \file
///
//  CHANGES removed minor defects detectd by static analysis
//
//============================================================================*/

#include "stm32f10x.h"
#include "config.h"
#include "pid.h"

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

/*--------------------------------- Prototypes -------------------------------*/

/*---------------------------------- Functions -------------------------------*/


///----------------------------------------------------------------------------
///
/// \brief   PID initialization
/// \param
/// \return  -
/// \remarks
///
///----------------------------------------------------------------------------
void PID_Init(xPID * pxPid)
{
    pxPid->fLastInput = 0.0f;
    pxPid->fIntegral = 0.0f;
}


///----------------------------------------------------------------------------
///
/// \brief   PID computing
/// \param
/// \param
/// \return  -
/// \remarks
///
///----------------------------------------------------------------------------
float PID_Compute(xPID * pxPid, const float fSetpoint, const float fInput)
{
   float fError, fDelta, fOutput;

    // Compute error
    fError = fSetpoint - fInput;

    // Compute integral term
    // Avoid windup
    if ((pxPid->fIntegral < pxPid->fMax) &&
        (pxPid->fIntegral > pxPid->fMin)) {
        pxPid->fIntegral += (fError * DELTA_T);
    }

    // Compute differential term
    // Multiply by SAMPLES_PER_SECOND instead of dividing by DELTA_T.
    fDelta = (fInput - pxPid->fLastInput) * SAMPLES_PER_SECOND;

    // Compute output
    fOutput = pxPid->fKp * fError +
              pxPid->fKi * pxPid->fIntegral -
              pxPid->fKd * fDelta;

    // Saturate output
    if (fOutput > pxPid->fMax) {
       fOutput = pxPid->fMax;
    } else if (fOutput < pxPid->fMin) {
       fOutput = pxPid->fMin;
    } else {
    }

    // Multiply by output gain
    fOutput = pxPid->fGain * fOutput;

    // Store current input
    pxPid->fLastInput = fInput;

    return fOutput;
}

