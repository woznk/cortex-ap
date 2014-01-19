/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief PID controls header file
 *
 * @file
 *
 * Change
 *
 *============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/* PID data structure */
typedef struct {
    float fGain;       /* controller output gain */
    float fMin;        /* min windup guard */
    float fMax;        /* max windup guard */
    float fLastInput;  /* last input */
    float fIntegral;   /* integral term */
    float fKp;         /* proportional gain */
    float fKi;         /* integral gain */
    float fKd;         /* derivative gain */
    float fSetpoint;
    float fInput;
} xPID;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

void PID_Init(xPID * pxPid);
float PID_Compute(xPID * pxPid);
