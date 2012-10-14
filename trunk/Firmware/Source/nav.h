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
//  Change added function Nav_Altitude()
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

void Navigation_Task( void *pvParameters );
float Nav_Altitude ( void );
float Nav_Heading ( void );
float Nav_Bearing ( void );
float Nav_Bank ( void );
float Nav_Pitch ( void ) ;
float Nav_Throttle ( void );
uint16_t Nav_Distance ( void );
uint16_t Gps_Speed ( void );
uint16_t Gps_Heading ( void );
int32_t Gps_Latitude ( void );
int32_t Gps_Longitude ( void );
uint8_t Nav_Wpt_Index ( void );
uint16_t Nav_Wpt_Altitude ( void );
STRUCT_WPT Nav_Get_Wpt ( uint8_t index );
void Nav_Set_Wpt ( uint8_t index, STRUCT_WPT wpt );

