//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \file
/// \brief  Navigation task header file
///
//  CHANGES added Nav_Pitch() function
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
float Nav_Heading ( void );
float Nav_Bearing ( void );
float Nav_Bank ( void );
float Nav_Pitch ( void ) ;
float Nav_Throttle ( void );
uint16_t Gps_Speed ( void );
uint16_t Gps_Heading ( void );
uint16_t Nav_Distance ( void );
uint16_t Nav_Wpt_Index ( void );
uint16_t Nav_Wpt_Altitude ( void );

