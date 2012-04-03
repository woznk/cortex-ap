//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \file
/// \brief  Navigation task header file
//  CHANGES Nav_Ground_Speed() renamed Gps_Speed()
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
uint16_t Gps_Speed ( void );
uint16_t Gps_Heading ( void );
uint16_t Nav_Distance ( void );
uint16_t Nav_Wpt_Index ( void );
uint16_t Nav_Wpt_Altitude ( void );

