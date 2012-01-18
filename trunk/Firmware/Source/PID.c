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
//  CHANGES Changed PID computation working principle
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
    pxPid->fIntegral += (pxPid->Ki * fError);

    // Saturate integral term
    if (pxPid->fIntegral > pxPid->fMax) {
       pxPid->fIntegral = pxPid->fMax;
    } else if (pxPid->fIntegral < pxPid->fMin) {
       pxPid->fIntegral = pxPid->fMin;
    }

    // Compute input difference
    fDelta = (fInput - pxPid->fLastInput);

    // Compute output
    fOutput = pxPid->Kp * fError + pxPid->fIntegral - pxPid->Kd * fDelta;

    // Saturate output
    if (fOutput > pxPid->fMax) {
       fOutput = pxPid->fMax;
    } else if (fOutput < pxPid->fMin) {
       fOutput = pxPid->fMin;
    }

    // Store current input
    pxPid->fLastInput = fInput;

    return fOutput;
}
