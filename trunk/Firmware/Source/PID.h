//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief
///  PID controls
/// \file
///
///
//  CHANGES Added 'f' prefix to P I D coefficients
//          added output gain
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

typedef struct {       ///< PID data structure
    float fGain;       ///< controller output gain
    float fMin;        ///< min windup guard
    float fMax;        ///< max windup guard
    float fLastInput;  ///< last input
    float fIntegral;   ///< integral term
    float fKp;         ///< proportional gain
    float fKi;         ///< integral gain
    float fKd;         ///< derivative gain
} xPID;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

void PID_Init(xPID * pxPid);
float PID_Compute(xPID * pxPid, float fSetpoint, float fInput);
