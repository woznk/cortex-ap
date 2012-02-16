//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief PID controls
/// \file
///
//  CHANGES output gain multiplication moved after output saturation
//
//============================================================================*/

#include "stm32f10x.h"
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
float PID_Compute(xPID * pxPid, float fSetpoint, float fInput)
{
   float fError, fDelta, fOutput;

    // Compute error
    fError = fSetpoint - fInput;

    // Compute integral term
    pxPid->fIntegral += (pxPid->fKi * fError);

    // Saturate integral term
    if (pxPid->fIntegral > pxPid->fMax) {
       pxPid->fIntegral = pxPid->fMax;
    } else if (pxPid->fIntegral < pxPid->fMin) {
       pxPid->fIntegral = pxPid->fMin;
    }

    // Compute input difference
    fDelta = (fInput - pxPid->fLastInput);

    // Compute output
    fOutput = pxPid->fKp * fError + pxPid->fIntegral - pxPid->fKd * fDelta;

    // Saturate output
    if (fOutput > pxPid->fMax) {
       fOutput = pxPid->fMax;
    } else if (fOutput < pxPid->fMin) {
       fOutput = pxPid->fMin;
    }

    // Multiply outpput by its gain
    fOutput = pxPid->fGain * fOutput;

    // Store current input
    pxPid->fLastInput = fInput;

    return fOutput;
}

