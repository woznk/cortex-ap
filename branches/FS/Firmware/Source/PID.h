//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief
///  PID controls
/// \file
///
///
//  CHANGES Changed PID computation working principle
//
//============================================================================*/

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

typedef struct
{
    float fMin;        ///< min saturation
    float fMax;        ///< max saturation
    float fLastInput;  ///< last input
    float fIntegral;   ///< integral term
    float Kp;          ///< proportional gain
    float Ki;          ///< integral gain
    float Kd;          ///< derivative gain
} xPID;

void PID_Init(xPID * pxPid);
float PID_Compute(xPID * pxPid, float fSetpoint, float fInput);
