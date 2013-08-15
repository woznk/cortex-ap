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
//  Change: Nav_Get_Wpt, Nav_Set_Wpt renamed Nav_Wpt_Get, Nav_Wpt_Set
//          added function Nav_Wpt_Number
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

/// waypoint structure
typedef struct {
    float Lon;      //!< longitude
    float Lat;      //!< latitude
    float Alt;      //!< altitude
} STRUCT_WPT;

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

uint8_t Gps_Fix( void );
uint16_t Gps_Speed ( void );
uint16_t Gps_Heading_Deg ( void );
int32_t Gps_Latitude ( void );
int32_t Gps_Longitude ( void );

void Navigation_Task( void *pvParameters );
float Nav_Altitude ( void );
float Nav_Heading_Deg ( void );
float Nav_Bearing_Deg ( void );
float Nav_Bank_Rad ( void );
float Nav_Pitch_Rad ( void ) ;
float Nav_Throttle ( void );
uint16_t Nav_Distance ( void );
uint16_t Nav_Wpt_Number ( void );
uint16_t Nav_Wpt_Index ( void );
uint16_t Nav_Wpt_Altitude ( void );
void Nav_Wpt_Get ( uint16_t index, STRUCT_WPT *wpt );
void Nav_Wpt_Set ( uint16_t index, STRUCT_WPT wpt );

