//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \file
/// \brief  GPS manager header file
//  CHANGES tBoolean replaced with bool
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

#define GPS_STATUS_FIX      1       // GPS status: fix
#define GPS_STATUS_FIRST    2       // GPS status: waiting for first fix
#define GPS_DEBUG           1

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*------------------------------------ Types ---------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

VAR_GLOBAL float fCurrLat;
VAR_GLOBAL float fCurrLon;
VAR_GLOBAL int Heading;                 //
VAR_GLOBAL unsigned char Gps_Status;    //

/*---------------------------------- Interface -------------------------------*/

void GPSInit ( void );
bool GPSFix ( void );
bool GPSParse( void );
bool GPSPosition ( void );
int GPSHeading ( void );
int GPSNorth ( void );
unsigned int GPSSpeed ( void );
