//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief navigation stub
///
/// \file
///
// Change:
//
//============================================================================*/

#include <stdint.h>

#include "nav.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#   undef VAR_STATIC
#endif
#define   VAR_STATIC static
#ifdef    VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

#define MAX_WAYPOINTS       8       //!< Maximum number of waypoints

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC float fBank;                                 //!< bank angle setpoint [rad]
VAR_STATIC float fThrottle = 0.3f;                      //!< throttle
VAR_STATIC float fPitch = 5.2f;                         //!< pitch setpoint [rad]
VAR_STATIC float fLat_Dest;                             //!< destination latitude
VAR_STATIC float fLon_Dest;                             //!< destination longitude
VAR_STATIC float fAlt_Dest = 250.0f;                    //!< destination altitude
VAR_STATIC float fLat_Curr;                             //!< current latitude
VAR_STATIC float fLon_Curr;                             //!< current longitude
VAR_STATIC float fAlt_Curr = 150.0f;                    //!< current altitude
VAR_STATIC float fLon_Temp;                             //!< temporary longitude during parse
VAR_STATIC float fLat_Temp;                             //!< temporary altitude during parse
VAR_STATIC float fBearing = 15.4f;                      //!< angle to destination [°]
VAR_STATIC float fHeading = 130.0f;                     //!< aircraft navigation heading [°]
VAR_STATIC uint16_t uiGps_Heading = 230;                //!< aircraft GPS heading [°]
VAR_STATIC uint32_t ulTempCoord;                        //!< temporary for coordinate parser
VAR_STATIC uint16_t uiSpeed = 10;                       //!< speed [kt]
VAR_STATIC uint16_t uiDistance;                         //!< distance to destination [m]
VAR_STATIC uint8_t uiWptNumber;                         //!< total number of waypoints
VAR_STATIC uint8_t uiWptIndex;                          //!< waypoint index
VAR_STATIC uint8_t ucGps_Status;                        //!< status of GPS
VAR_STATIC STRUCT_WPT Waypoint[MAX_WAYPOINTS];         	//!< waypoints array

/*--------------------------------- Prototypes -------------------------------*/


//----------------------------------------------------------------------------
//
/// \brief   Get waypoint index
/// \param   -
/// \returns waypoint index
/// \remarks -
///
//----------------------------------------------------------------------------
uint8_t Nav_Wpt_Index ( void ) {
  return uiWptIndex;
}

//----------------------------------------------------------------------------
//
/// \brief   Get waypoint altitude [m]
/// \param   -
/// \returns altitude
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Nav_Wpt_Altitude ( void ) {
  return (uint16_t)Waypoint[uiWptIndex].Alt;
}

//----------------------------------------------------------------------------
//
/// \brief   Get waypoint data
/// \param   index = waypoint number
/// \returns waypoint data structure
/// \remarks -
///
//----------------------------------------------------------------------------
STRUCT_WPT Nav_Get_Wpt ( uint8_t index ) {
  if (index > uiWptNumber) {
     index = 0;
  }
  return Waypoint[index];
}

//----------------------------------------------------------------------------
//
/// \brief   Set waypoint data
/// \param   index = waypoint number
/// \param   wpt = waypoint data structure
/// \returns -
/// \remarks -
///
//----------------------------------------------------------------------------
void Nav_Set_Wpt ( uint8_t index, STRUCT_WPT wpt ) {
}

//----------------------------------------------------------------------------
//
/// \brief   Get computed bearing [°]
/// \param   -
/// \returns bearing angle in degrees, between 0° and 360°
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Bearing ( void ) {
  if (fBearing < 0.0f)  {
    return 360.0f + (fBearing * 180.0f);
  } else {
    return (fBearing * 180.0f);
  }
}

//----------------------------------------------------------------------------
//
/// \brief   Get current heading [°]
/// \param   -
/// \returns heading angle in degrees, between 0° and 360°
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Heading ( void ) {
  if (fHeading < 0.0f)  {
    return 360.0f + (fHeading * 180.0f);
  } else {
    return (fHeading * 180.0f);
  }
}

//----------------------------------------------------------------------------
//
/// \brief   Get distance to destination [m]
/// \param   -
/// \returns distance in meters
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Nav_Distance ( void ) {
  return uiDistance;
}

//----------------------------------------------------------------------------
//
/// \brief   Get bank angle [rad]
/// \param   -
/// \returns bank angle
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Bank ( void ) {
  return fBank;
}

//----------------------------------------------------------------------------
//
/// \brief   Get throttle
/// \param   -
/// \returns throttle
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Throttle ( void ) {
  return fThrottle;
}

//----------------------------------------------------------------------------
//
/// \brief   Get pitch setpoint [rad]
/// \param   -
/// \returns throttle
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Pitch ( void ) {
  return fPitch;
}

//----------------------------------------------------------------------------
//
/// \brief   Get altitude [m]
/// \param   -
/// \returns throttle
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Altitude ( void ) {
  return fAlt_Curr;
}

//----------------------------------------------------------------------------
//
/// \brief   Get ground speed detected by GPS [kt]
/// \param   -
/// \returns GS in knots
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Gps_Speed ( void ) {
  return uiSpeed;
}

//----------------------------------------------------------------------------
//
/// \brief   Get GPS heading [°]
/// \param   -
/// \returns heading angle in degrees, between 0° and 360°
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Gps_Heading ( void ) {
  return uiGps_Heading;
}

//----------------------------------------------------------------------------
//
/// \brief   Get latitude [°]
/// \param   -
/// \returns latitude angle in 1 / 10000000 degrees
/// \remarks -
///
//----------------------------------------------------------------------------
int32_t Gps_Latitude ( void ) {
  return (int32_t)(fLat_Curr * 10000000.0f);
}

//----------------------------------------------------------------------------
//
/// \brief   Get longitude [°]
/// \param   -
/// \returns longitude angle in 1 / 10000000 degrees
/// \remarks -
///
//----------------------------------------------------------------------------
int32_t Gps_Longitude ( void ) {
  return (int32_t)(fLon_Curr * 10000000.0f);
}

