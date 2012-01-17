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
//  CHANGES
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
	float B0;          ///< derived gain, B0 = Kp + Ki + Kd
    float B1;          ///< derived gain, B1 = -Kp - 2Kd
    float B2;          ///< derived gain, B2 = Kd
    float State[3];    ///< state array
    float Endpoint[2]; ///< min and max saturation
    float Kp;          ///< proportional gain
    float Ki;	       ///< integral gain
    float Kd;		   ///< derivative gain
} stPID;

void PID_Init(stPID*, bool bReset);
float PID_Compute(stPID * psPid, float fError);
