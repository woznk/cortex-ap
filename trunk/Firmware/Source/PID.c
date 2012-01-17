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
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
///
//  CHANGES
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
void PID_Init(stPID * psPid, bool bReset)
{
	// Derive coefficient B2
    psPid->B2 = psPid->Kp + psPid->Ki + psPid->Kd;

    // Derived coefficient B1
    psPid->B1 = (-psPid->Kp) - ((float) 2.0 * psPid->Kd);

    // Derived coefficient B0
    psPid->B0 = psPid->Kd;

    if (bReset) {
    	psPID->State[0] = 0;
    	psPID->State[1] = 0;
    	psPID->State[2] = 0;
    }
}


///----------------------------------------------------------------------------
///
/// \brief   PID computing
/// \param
/// \param
/// \return  -
/// \remarks
///
/// y[n] = y[n-1] + B2 * x[n] + B1 * x[n-1] + B0 * x[n-2]
///
/// Ideal = (State[2])+ (B2 * e) + (B1 * State[0]) + (B0 * State[1]);
///
/// Saturate(Ideal)
///
/// Update state
/// State[0] = In;			// x(k-1)
/// State[1] = State[0];	// x(k-2)
/// State[2] = Out;			// y(k-1)
///
///----------------------------------------------------------------------------
float PID_Compute(stPID * psPID, float fError)
{
	float Ucm;
	float Uid;
	float Elim;

	Uid = psPID->State[0] +
          (psPID->Kp + psPID->Ki + psPID->Kd) * fError -
          (psPID->Kd * psPID->State[1]);

	// Saturate
	if (Uid > psPID->Endpoint[1]) {
		Ucm = psPID->Endpoint[1];
	} else if (Uid < psPID->Endpoint[0]) {
		Ucm = psPID->Endpoint[0];
    } else {
		Ucm = Uid;
    }

	Elim = fError - (Uid - Ucm) / (psPID->Kp + psPID->Ki + psPID->Kd);
	S->State[0] +=  (psPID->Ki * Elim);
	S->State[1] = fError;

	return (Ucm);
}
