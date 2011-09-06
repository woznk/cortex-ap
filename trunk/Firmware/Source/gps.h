//============================================================================
//
// $RCSfile: gps.h,v $ (HEADER FILE)
// $Revision: 1.4 $
// $Date: 2010/01/25 21:39:47 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             GPS manager header file
//  CHANGES     navigazione e gestione waypoints spostata in nav.c
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
tBoolean GPSFix ( void );
tBoolean GPSParse( void );
tBoolean GPSPosition ( void );
int GPSHeading ( void );
int GPSNorth ( void );
unsigned int GPSSpeed ( void );
