//============================================================================+
//
// $RCSfile: GPS.cpp,v $ (SOURCE FILE)
// $Revision: 1.5 $
// $Date: 2011/01/23 17:53:08 $
// $Author: Lorenz $
//
/// \brief GPS driver
///
/// \file
///
//  CHANGES Simulation.h sostituito da Telemetry.h
//
//============================================================================*/

#include "stdafx.h"

#include "inc/hw_types.h"
#ifndef _WINDOWS
#include "uartdriver.h"
#endif
#include "telemetry.h"
#include "gps.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#   undef VAR_STATIC
#endif
#define   VAR_STATIC static
#ifdef    VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

#ifdef _WINDOWS
#   define Gps_GetChar(c) TRUE
#elif (GPS_DEBUG == 0)
#   define Gps_GetChar UART1GetChar
#else
#   define Gps_GetChar Debug_GetChar
#endif

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL float fCurrLat;
VAR_GLOBAL float fCurrLon;
VAR_GLOBAL int Heading;                 //
VAR_GLOBAL unsigned char Gps_Status;    //

/*----------------------------------- Locals ---------------------------------*/


VAR_STATIC int North;                   // angle to north
VAR_STATIC unsigned int Speed;          // speed

#if (GPS_DEBUG == 1)
VAR_STATIC const char s_pcSentence[] = "$GPRMC,194617.04,A,4534.6714,N,01128.8559,E,000.0,287.0,091008,001.9,E,A*31\n$GPRMC,194618.04,A,4534.6714,N,01128.8559,E,000.0,287.0,091008,001.9,E,A*3E";
#endif

/*--------------------------------- Prototypes -------------------------------*/

#if (GPS_DEBUG == 1)
tBoolean Debug_GetChar ( char *ch );
#endif

//----------------------------------------------------------------------------
//
/// \brief   Initialize gps interface
///
/// \remarks The second UART will be configured in 4800 baud, 8-n-1 mode.
///
//----------------------------------------------------------------------------
void
GPSInit( void )
{
    //
    // Wait for first fix
    //
    Gps_Status = GPS_STATUS_FIRST;
}


//----------------------------------------------------------------------------
//
/// \brief   Parse NMEA sentence for coordinates
///
/// \param   integer : (pointer to) integer part of coordinate
/// \param   decimal : (pointer to) decimal part of coordinate
/// \param   c       : character of NMEA sentence
/// \remarks -
///
//----------------------------------------------------------------------------
void
Parse_Coord( int * integer, int * decimal, char c ) {

  if ( c == ',' ) {
    *decimal = 0;
  } else if ( c != '.' ) {
    *decimal *= 10;
    *decimal += (c - '0');
  } else {
    *integer = *decimal;
    *decimal = 0;
  }

}


void
Parse_Lat( float * fLat, char c ) {

    static unsigned long ulLat = 0UL;

    if ( c == ',' ) {
        *fLat = (ulLat % 1000000UL) / 6000.0f;     // convert ' to °
        *fLat += (ulLat / 100000UL);               // add °
        ulLat = 0UL;
    } else if ( c != '.' ) {
        ulLat = ulLat * 10UL + (unsigned long)(c - '0');
    }
}


void
Parse_Lon( float * fLon, char c ) {

    static unsigned long ulLon = 0UL;

    if ( c == ',' ) {
        *fLon = (ulLon % 1000000UL) / 6000.0f;     // convert ' to °
        *fLon += (ulLon / 100000UL);               // add °
        ulLon = 0UL;
    } else if ( c != '.' ) {
        ulLon = ulLon * 10UL + (unsigned long)(c - '0');
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Parse GPS sentences
///
/// \returns true if new coordinate data are available,
///          false otherwise
/// \remarks
///
///
//----------------------------------------------------------------------------
tBoolean GPSParse( void )
{
    char c;
    tBoolean result = false;
    static unsigned char commas;

    if (Gps_GetChar(&c))              // received another character
    {
        if ( c == '$' ) commas = 0; // start of NMEA sentence

        if ( c == ',' ) commas++;   // count commas

        if ( commas == 2 )          // get fix info
        {
           if (c == 'A')
           {
             Gps_Status |= GPS_STATUS_FIX;
           }
           else if (c == 'V')
           {
             Gps_Status &= ~GPS_STATUS_FIX;
           }
        }

        if ( commas == 3 )          // get latitude data (3rd comma)
        {
          Parse_Lat (&fCurrLat, c);
        }

        if ( commas == 4 ) {        // TO DO : change sign of Lat based on N / S
        }

        if ( commas == 5 )          // get longitude data (5th comma)
        {
          Parse_Lon (&fCurrLon, c);
        }

        if ( commas == 6 ) {        // TO DO : change sign of Lon based on E / W
        }

        if ( commas == 7 )          // get speed (7th comma)
        {
          if ( c == ',' )
          {
            Speed = 0;
          }
          else if ( c != '.' )
          {
            Speed *= 10;
            Speed += (c - '0');
          }
        }

        if ( commas == 8 )          // get heading (8th comma)
        {
          if ( c == ',' )
          {
            Heading = 0;
          }
          else if ( c != '.' )
          {
            Heading *= 10;
            Heading += (c - '0');
          }
        }

        if ((commas == 9) && (Gps_Status & GPS_STATUS_FIX)) // end of NMEA sentence
        {
          commas = 10;
          Heading /= 10;
          North = 360 - Heading;
          result = true;
        }
    }
    return result;
}


//----------------------------------------------------------------------------
//
/// \brief   Get GPS fix status
///
/// \returns true if fix
///
/// \remarks -
///
///
//----------------------------------------------------------------------------
tBoolean GPSFix ( void )
{
  return ((Gps_Status | GPS_STATUS_FIX) == GPS_STATUS_FIX);
}


//----------------------------------------------------------------------------
//
/// \brief   Get current GPS heading
///
/// \returns heading angle in degrees,  between 0° and 360°
///
/// \remarks -
///
///
//----------------------------------------------------------------------------
int GPSHeading ( void )
{
  return Heading;
}


//----------------------------------------------------------------------------
//
/// \brief   Get current North angle
///
/// \returns North angle in degrees,  between 0° and 360°
///
/// \remarks -
///
///
//----------------------------------------------------------------------------
int GPSNorth ( void )
{
  return North;
}


//----------------------------------------------------------------------------
//
/// \brief   Get speed
///
/// \returns speed in m / s
///
/// \remarks -
///
///
//----------------------------------------------------------------------------
unsigned int GPSSpeed ( void )
{
  unsigned long ulTemp;

    ulTemp = ((unsigned long)Speed * 1852) / 36000;
    return (unsigned int)ulTemp;
}


#if (GPS_DEBUG == 1)
//----------------------------------------------------------------------------
//
/// \brief   get a character from preloaded string
///
/// \returns
/// \remarks
///
///
//----------------------------------------------------------------------------
tBoolean
Debug_GetChar ( char *ch )
{
    char c;
    VAR_STATIC unsigned char j = 0;

    c = s_pcSentence[j];
    if (c == 0) { j = 0; } else { j++; }
    *ch = c;
    return true;
}
#endif
