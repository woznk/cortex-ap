//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \file
///             GPS driver
//
//  CHANGES     UART 1 management moved to uartdriver.c
//              removed unused include files
//              added SimulatedGPSChar for debugging
//
//============================================================================*/

#include "inc/hw_types.h"
#include "uartdriver.h"
#include "cordic.h"
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

#define GPS_STATUS_FIX      1       // GPS status: fix
#define GPS_STATUS_FIRST    2       // GPS status: waiting for first fix
#define GPS_DEBUG           0

#if (GPS_DEBUG == 0)
#   define GpsChar UART1GetChar
#else
#   define GpsChar SimulatedGPSChar
#endif

/****************************************************************************
*
*                                                               Pi
*                     radius of waypoint circle [meters] * -------------
*                                                          180 [deg/rad]
* Angle to waypoint = --------------------------------------------------- =
*                              60 [min/deg] *  2170 [meters/min]
*
*
*                     radius of waypoint circle * Pi
*                   = ------------------------------
*                               23436000
*
****************************************************************************/

#define WAYPOINT_ANGLE  (90)  // come floating sarebbe 0.0000090

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

#if (GPS_DEBUG == 1)
VAR_STATIC const char s_pucSentence[] = "$GPRMC,194617.04,A,4534.6714,N,01128.8559,E,000.0,287.0,091008,001.9,E,A*31\n$GPRMC,194618.04,A,4534.6714,N,01128.8559,E,000.0,287.0,091008,001.9,E,A*3E";
#endif

VAR_STATIC int Curr_Lat_Int;
VAR_STATIC int Curr_Lat_Dec;
VAR_STATIC int Curr_Lon_Int;
VAR_STATIC int Curr_Lon_Dec;
VAR_STATIC int Dest_Lat_Int;
VAR_STATIC int Dest_Lat_Dec;
VAR_STATIC int Dest_Lon_Int;
VAR_STATIC int Dest_Lon_Dec;
VAR_STATIC int North;                   // angle to north
VAR_STATIC int Bearing;                 // angle to destination
VAR_STATIC int Heading;                 // course over ground
VAR_STATIC int Direction;               // direction error
VAR_STATIC unsigned int Distance;       // distance to destination
VAR_STATIC unsigned int Speed;          // speed
VAR_STATIC unsigned char Gps_Status;    //

/*--------------------------------- Prototypes -------------------------------*/

#if (GPS_DEBUG == 1)
tBoolean SimulatedGPSChar ( char *ch );
#endif
static unsigned int root ( unsigned long x );

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
Parse_Coord( int * integer, int * decimal, char c )
{
  if ( c == ',' )
  {
    *decimal = 0;
  }
  else if ( c != '.' )
  {
    *decimal *= 10;
    *decimal += (c - '0');
  }
  else
  {
    *integer = *decimal;
    *decimal = 0;
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
    VAR_STATIC unsigned char commas;

    if (GpsChar(&c))              // received another character
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
          Parse_Coord (&Curr_Lat_Int, &Curr_Lat_Dec, c);
        }

        if ( commas == 5 )          // get longitude data (5th comma)
        {
          Parse_Coord (&Curr_Lon_Int, &Curr_Lon_Dec, c);
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
          Navigate();
        }
    }
    return result;
}


//----------------------------------------------------------------------------
//
/// \brief   Main navigation function
///
/// \remarks Must be called only when GPSParse() retuns true, otherwise
///          values of Lat, Lon, Heading are unpredictable.
///
///                     destination   +---
///                                  /|  ^
///                                 / |  |
///                                /  |  |
///                               /   |  |
///                              /    |  |
///                    distance /     |  |  latitiude
///                            /      |  |  difference
///                           /       |  |
///                          /        |  |
///                         /         |  |
///                        /          |  |
///                       /  bearing  |  |
///                      / ) angle    |  V
///          actual     +-------------|---
///          position   |             |
///                     |<----------->|
///
///                       longitude
///                       difference
///
///
//----------------------------------------------------------------------------
void Navigate( void )
{
    unsigned long temp;

    //
    // Store home coordinates at first GPS fix
    //
    if (Gps_Status & GPS_STATUS_FIRST)
    {
      if (Gps_Status & GPS_STATUS_FIX)
      {
        Gps_Status &= ~GPS_STATUS_FIRST;
        Dest_Lat_Dec = Curr_Lat_Dec;
        Dest_Lat_Int = Curr_Lat_Int;
        Dest_Lon_Dec = Curr_Lon_Dec;
        Dest_Lon_Int = Curr_Lon_Int;
      }
      else
      {
        return;
      }
    }

    //
    // calculate bearing to destination, works for short distances
    // reuses current lat and lon to save RAM
    //
    Curr_Lat_Dec = Dest_Lat_Dec - Curr_Lat_Dec;
    Curr_Lat_Int = Dest_Lat_Int - Curr_Lat_Int;
    if (Curr_Lat_Int > 0)
    {
      Curr_Lat_Dec += 10000;
    }
    else if (Curr_Lat_Int < 0)
    {
      Curr_Lat_Dec -= 10000;
    }

    Curr_Lon_Dec = Dest_Lon_Dec - Curr_Lon_Dec;
    Curr_Lon_Int = Dest_Lon_Int - Curr_Lon_Int;
    if (Curr_Lon_Int > 0)
    {
      Curr_Lon_Dec += 10000;
    }
    else if (Curr_Lon_Int < 0)
    {
      Curr_Lon_Dec -= 10000;
    }

    Bearing = cordic_atan(Curr_Lon_Dec, Curr_Lat_Dec);

    // bearing is positive in CCW direction, heading is positive in CW direction
    // bearing reference is rotated 90° CW with respect to heading reference
    Bearing = 90 - (Bearing / 100);
    if (Bearing < 0) Bearing = Bearing + 360;

    // compute direction error, reuse Bearing to save RAM
    Direction = Bearing - Heading;
    if (Direction < 0) Direction = Direction + 360;

    // compute distance to destination
    temp =  (unsigned long)((long)Curr_Lat_Dec * (long)Curr_Lat_Dec);
    temp += (unsigned long)((long)Curr_Lon_Dec * (long)Curr_Lon_Dec);
    temp = (unsigned long)root(temp);
    temp = (temp * 1000) / 11865;
    Distance = (unsigned int)temp;
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
/// \brief   Get direction to home position
///
/// \returns direction angle in degrees, between 0° and 360°
///
/// \remarks -
///
///
//----------------------------------------------------------------------------
int GPSDirection ( void )
{
  return Direction;
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
/// \brief   Get computed bearing
///
/// \returns bearing angle in degrees,  between -180° and + 180°
///
/// \remarks -
///
///
//----------------------------------------------------------------------------
int GPSBearing ( void )
{
  return Bearing;
}


//----------------------------------------------------------------------------
//
/// \brief   Get distance to destination
///
/// \returns distance in meters
///
/// \remarks -
///
///
//----------------------------------------------------------------------------
unsigned int GPSDistance ( void )
{
  return Distance;
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


//----------------------------------------------------------------------------
//
/// \brief   Computes square root
///
/// \returns square root
///
/// \remarks
///
///
//----------------------------------------------------------------------------
static unsigned int root( unsigned long val )
{
    unsigned long rem = 0;
    unsigned long root = 0;
    unsigned char i;

    for (i = 0; i < 16; i++)
    {
        root <<= 1;
        rem = ((rem << 2) + (val >> 30));
        val <<= 2;
        root++;
        if (root <= rem)
        {
            rem -= root;
            root++;
        }
        else
        {
            root--;
        }
    }
    return (unsigned int)(root >> 1);
}

#if (GPS_DEBUG == 1)
//----------------------------------------------------------------------------
//
/// \brief   get a character from preloaded sentence
///
/// \returns
/// \remarks
///
///
//----------------------------------------------------------------------------
tBoolean
SimulatedGPSChar ( char *ch )
{
    char c;
    VAR_STATIC unsigned char j = 0;

    c = s_pucSentence[j];
    if (c == 0)
    {
        j = 0;
    }
    else
    {
        j++;
    }
    *ch = c;
    return true;
}
#endif
