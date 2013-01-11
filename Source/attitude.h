//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief attitude control header file
///
/// \file
///
//  Change duplicated interface functions for both deg and rad units
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

void Attitude_Task(void *pvParameters);

float Attitude_Pitch_Deg(void);
float Attitude_Roll_Deg(void);
float Attitude_Yaw_Deg(void);

float Attitude_Pitch_Rad(void);
float Attitude_Roll_Rad(void);
float Attitude_Yaw_Rad(void);
