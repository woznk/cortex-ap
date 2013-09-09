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
//  Change: added suffix U to explicit unsigned values 
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

#define GPS_FIX         3U       //!< GPS status: satellite fix
#define GPS_NOFIX       0U       //!< GPS status: waiting for first fix

#define BUFFER_LENGTH   96U      //!< length of buffer for file and USART

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
uint16_t Gps_Alt_M ( void );
uint16_t Gps_Speed_Kt ( void );
uint16_t Gps_Heading_Deg ( void );
int32_t Gps_Latitude ( void );
int32_t Gps_Longitude ( void );
uint8_t Gps_Buffer_Index ( void );
uint8_t * Gps_Buffer_Pointer ( void );

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

