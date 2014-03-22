/**===========================================================================
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief GPS header file
 *
 * @file
 *
 * Change:
 *
 *============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#define GPS_FIX             3       /* GPS status: satellite fix */
#define GPS_NOFIX           0       /* GPS status: waiting for first fix */
#define GPS_BUFFER_LENGTH   96      /* length of buffer for USART */

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*------------------------------------ Types ---------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

void Init_GPS ( void );
void Parse_GPS ( void );
uint8_t Gps_Status ( void );
uint16_t Gps_Speed_Kt ( void );
uint16_t Gps_Alt_M ( void );
uint16_t Gps_COG_Deg ( void );
float Gps_Latitude ( void );
float Gps_Longitude ( void );
