//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief  Navigation task header file
///
/// \file
///
//  Change added functions Gps_Latitude() and Gps_Longitude()
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
float Nav_Heading ( void );
float Nav_Bearing ( void );
float Nav_Bank ( void );
float Nav_Pitch ( void ) ;
float Nav_Throttle ( void );
uint16_t Nav_Distance ( void );
uint16_t Nav_Wpt_Index ( void );
uint16_t Nav_Wpt_Altitude ( void );
uint16_t Gps_Speed ( void );
uint16_t Gps_Heading ( void );
int32_t Gps_Latitude ( void );
int32_t Gps_Longitude ( void );

