//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \file
/// \brief  Navigation task header file
//  CHANGES added initialization of navigation
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*------------------------------------ Types ---------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

void Navigation_Task( void *pvParameters );
void Navigation_Init( void );
int16_t Nav_Bearing ( void );
int16_t Nav_Heading ( void );
uint16_t Nav_Distance ( void );
uint16_t Nav_WaypointIndex ( void );
uint16_t Nav_Ground_Speed ( void );

