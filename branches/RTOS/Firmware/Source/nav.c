//============================================================================+
//
// $RCSfile: nav.cpp,v $ (SOURCE FILE)
// $Revision: 1.4 $
// $Date: 2011/01/23 17:54:28 $
// $Author: Lorenz $
//
/// \brief Navigation manager
///
/// \file
/// La funzione di inizializzazione Nav_Init() serve a leggere da SD card il
/// file di testo contenente i waypoints, tradurre le stringhe in coordinate,
/// salvare i waypoint nell'array Waypoint[] e aggiornare il numero totale di
/// waypoints disponibili.
/// In assenza di SD card o in caso di errore nella lettura del file, il numero
/// totale dei waypoints rimane a 0.
/// La funzione di navigazione Navigate() attende il fix del GPS, salva le
/// coordinate del punto di partenza nel primo elemento dell'array Waypoint[],
/// calcola la direzione verso e la distanza dal successivo waypoint.
/// In assenza di waypoints disponibili, la funzione calcola la direzione e la
/// distanza rispetto al punto di partenza (RTL).
///
//  CHANGES moved from tff (tiny fat filesystem) to ff (fat filesystem)
//
//============================================================================*/


#include "stm32f10x.h"
#include "math.h"
#include "telemetry.h"
#include "config.h"
#include "gps.h"
#include "ff.h"
#include "tick.h"
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

#define MAX_WAYPOINTS       8
#define MIN_DISTANCE        100

#define FILE_BUFFER_LENGTH  64
#define MAX_LINE_LENGTH     48

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

typedef struct {    // waypoint structure
    float Lon;      // longitude
    float Lat;      // latitude
    float Alt;      // altitude
} STRUCT_WPT;

typedef enum {      // navigation status
    NAV_OPEN_FILE,  // opening waypoint file
    NAV_READ_FILE,  // reading waypoint file
    NAV_END         // end
} ENUM_NAV_STATUS;

typedef enum {      // navigation mode
    NAV_RTL,        // return to launch
    NAV_WPT         // waypoint following
} ENUM_NAV_MODE;

/*---------------------------------- Constants -------------------------------*/

/*
const STRUCT_WPT DefaultWaypoint[] = {   // LIPT 1
    { 11.568685f, 45.590291f, 0.0f },
    { 11.567511f, 45.567338f, 0.0f },
    { 11.529570f, 45.566736f, 0.0f },
    { 11.529501f, 45.591318f, 0.0f }
};

const STRUCT_WPT DefaultWaypoint[] = {   // LIPT 2
    { 11.539628f, 45.580001f, 31.0f },
    { 11.541140f, 45.567272f, 31.0f },
    { 11.529628f, 45.579835f, 31.0f },
    { 11.529958f, 45.566831f, 31.0f }
};

const STRUCT_WPT DefaultWaypoint[] = {   // HERON
    { 11.433509f, 45.540183f, 130.0f },
    { 11.432652f, 45.543691f, 130.0f },
    { 11.427580f, 45.543351f, 130.0f },
    { 11.431036f, 45.538116f, 130.0f }
};
*/
const STRUCT_WPT DefaultWaypoint[] = {   // LIPT 3
    { 11.528960f, 45.570528f, 150.0f },
    { 11.529764f, 45.566249f, 150.0f },
    { 11.534911f, 45.566383f, 150.0f },
    { 11.535398f, 45.570759f, 150.0f }
};

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC ENUM_NAV_STATUS eNavStatus = NAV_OPEN_FILE;
VAR_STATIC const char szFileName[16] = "path.txt";  // File name
VAR_STATIC char szLine[MAX_LINE_LENGTH];            // Input line
VAR_STATIC char pcBuffer[FILE_BUFFER_LENGTH];       // File data buffer
#ifndef _WINDOWS
VAR_STATIC FATFS stFat;                             // FAT
VAR_STATIC FIL stFile;                              // File object
VAR_STATIC UINT wFileBytes;                         // Counter of read bytes
#endif
VAR_STATIC int Bearing;                             // angle to destination [°]
VAR_STATIC unsigned int Distance;                   // distance to destination [m]
VAR_STATIC float fDestLat;                          // destination latitude
VAR_STATIC float fDestLon;                          // destination longitude
VAR_STATIC unsigned int uiWptIndex;                 // waypoint index
VAR_STATIC unsigned int uiWptNumber = 1;            // number of waypoints
VAR_STATIC STRUCT_WPT Waypoint[MAX_WAYPOINTS];      // waypoints array

/*--------------------------------- Prototypes -------------------------------*/

bool Parse_Waypoint(char * pszLine);


//----------------------------------------------------------------------------
//
/// \brief   Initialize navigation
///
/// \remarks
///
//----------------------------------------------------------------------------
bool
Nav_Init( void ) {

    char c;
    static char cCounter;
    char * pcBufferPointer;
    static char * pszLinePointer;
    bool bError = FALSE, bResult = FALSE;

#ifdef _WINDOWS

    for (char j = 0; j < 4; j++) {
        Waypoint[j + 1] = DefaultWaypoint[j];           // default waypoints
    }
    uiWptNumber = 4;                                    // waypoints available

#else

    switch (eNavStatus) {

        case NAV_OPEN_FILE:                             // Open waypoint file.
            if (FR_OK == f_mount(0, &stFat)) {          // Mount the file system.
                if (FR_OK == f_open(&stFile, szFileName, FA_READ)) {
                    pszLinePointer = szLine;            // Init line pointer
                    cCounter = MAX_LINE_LENGTH - 1;     // Init char counter
                    eNavStatus = NAV_READ_FILE;         // File succesfully open
                } else {                                // Error opening file
                    uiWptNumber = 0;                    // No waypoint available
                    eNavStatus = NAV_END;
                }
            } else {                                    // Error mounting FS
                uiWptNumber = 0;                        // No waypoint available
                eNavStatus = NAV_END;
            }
            break;

        case NAV_READ_FILE:                             // Read waypoint file.
            if ( bError ||
               ( FR_OK != f_read(&stFile, pcBuffer, FILE_BUFFER_LENGTH, &wFileBytes))) {
                f_close(&stFile);                       // Error reading file
                uiWptNumber = 0;                        // No waypoint available
                eNavStatus = NAV_END;                   // End initialization
            } else if ( wFileBytes == 0 ) {             // End of file
                f_close(&stFile);                       // Close file
                eNavStatus = NAV_END;                   // End initialization
            } else {                                    // File read successfull
                pcBufferPointer = pcBuffer;             // Init buffer pointer
                while ((wFileBytes != 0) && (!bError)) {// Buffer not empty and no error
                     wFileBytes--;                      // Decrease overall counter
                     c = *pcBufferPointer++;            // Read another char
                     if ( c == 13 ) {                   // Carriage return
                         cCounter = 0;                  // Reached end of line
                     } else if ( c == 10 ) {            // New line
                         cCounter--;                    // Increase counter
                     } else {
                         *pszLinePointer++ = c;         // Copy char
                         cCounter--;                    // Increase counter
                     }
                     if (cCounter == 0) {               // End of line
                        *pszLinePointer = 0;            // Append line delimiter
                        pszLinePointer = szLine;        // Reset line pointer
                        cCounter = MAX_LINE_LENGTH - 1; // Reset char counter
                        bError = Parse_Waypoint(szLine);// Parse line
                    }
                }
            }
            break;

        case NAV_END:
            bResult = TRUE;
            break;
    }
#endif
    return bResult;
}


//----------------------------------------------------------------------------
//
/// \brief   Main navigation function
///
/// \remarks Must be called only when GPSParse() returns true, otherwise
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
void
Navigate( void ) {
/*
   float temp, dlat, dlon;

    //
    // First GPS fix: save launch position
    //
    if ((Gps_Status & GPS_STATUS_FIRST) == GPS_STATUS_FIRST ) {
      if ((Gps_Status & GPS_STATUS_FIX) == GPS_STATUS_FIX) {
        Gps_Status &= ~GPS_STATUS_FIRST;
        Waypoint[0].Lon = fCurrLon;             // save launch position
        Waypoint[0].Lat = fCurrLat;
        if (uiWptNumber != 0) {                 // waypoint file available
            uiWptIndex = 1;                     // read first waypoint
        } else {                                // no waypoint file
            uiWptIndex = 0;                     // read launch position
        }
        fDestLon = Waypoint[uiWptIndex].Lon;    //
        fDestLat = Waypoint[uiWptIndex].Lat;    //
      } else {
        return;
      }
    }

    //
    // calculate bearing to destination, works for short distances
    // bearing is positive in CCW direction, heading is positive in CW direction
    // bearing reference is rotated 90° CW with respect to heading reference
    //
    dlon = (fDestLon - fCurrLon);
    dlat = (fDestLat - fCurrLat);
    Bearing = 90 - (int)((atan2f(dlat, dlon) * 180.0f) / PI);
    if (Bearing < 0) Bearing = Bearing + 360;

    //
    // compute distance to destination
    //
    temp = (sqrtf((dlat * dlat) + (dlon * dlon)) * 111113.7f);
    Distance = (unsigned int)temp;

    //
    // Waypoint reached: next waypoint
    //
    if (Distance < MIN_DISTANCE) {
        if ( uiWptNumber != 0 ) {
            if ( ++uiWptIndex == uiWptNumber ) {
                uiWptIndex = 1;
            }
        }
        fDestLon = Waypoint[uiWptIndex].Lon;
        fDestLat = Waypoint[uiWptIndex].Lat;
    }
*/
}


//----------------------------------------------------------------------------
//
/// \brief   Get waypoint index
///
/// \returns
///
/// \remarks -
///
///
//----------------------------------------------------------------------------
unsigned int
Nav_WaypointIndex ( void ) {
  return uiWptIndex;
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
int
Nav_Bearing ( void ) {
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
unsigned int
Nav_Distance ( void ) {
  return Distance;
}


//----------------------------------------------------------------------------
//
/// \brief   Parse waypoint coordinates
///
/// \returns true if an error occurred, FALSE otherwise
///
/// \remarks format of waypoint coordinate is:
///          xx.xxxxxx,[ ]yy.yyyyyy,[ ]aaa[.[a]]\0
///          where x = longitude, y = latitude, a = altitude
///          [ ] are zero or more spaces,
///          [.[a]] is an optional decimal point with an optional decimal data
///
//----------------------------------------------------------------------------
bool
Parse_Waypoint ( char * pszLine ) {
    char c;
    float fTemp, fDiv;
    unsigned char ucField = 0, ucCounter = MAX_LINE_LENGTH;

    while (( ucField < 3 ) && ( ucCounter > 0 )) {
        fDiv = 1.0f;                                // initialize divisor
        fTemp = 0.0f;                               // initialize temporary
        c = *pszLine++;                             // initialize char
        // leading spaces
        while (( c == ' ' ) && ( ucCounter > 0 )) {
            c = *pszLine++;                         // next char
            ucCounter--;                            // count characters
        }
        // start of integer part
        if (( c < '0' ) || ( c > '9' )) {           //
            return TRUE;                            // first char not numeric
        }
        // integer part
        while (( c >= '0' ) && ( c <= '9' ) && ( ucCounter > 0 )) {
            fTemp = fTemp * 10.0f + (float)(c - '0'); // accumulate
            c = *pszLine++;                         // next char
            ucCounter--;                            // count characters
        }
        // decimal point
        if (( c != '.' ) && ( ucField != 2 )) {     // altitude may lack decimal
            return TRUE;
        } else {
            c = *pszLine++;                         // skip decimal point
            ucCounter--;                            // count characters
        }
        // fractional part
        while (( c >= '0' ) && ( c <= '9' ) && ( ucCounter > 0 )) {
            if (fDiv < 1000000.0f) {
                fTemp = fTemp * 10.0f + (float)(c - '0'); // accumulate
                fDiv = fDiv * 10.0f;                // update divisor
            }
            c = *pszLine++;                         // next char
            ucCounter--;                            // count characters
        }
        // delimiter
        if (( c != ',' ) && ( c != 0 )) {
            return TRUE;                            // error
        } else {
            fTemp = fTemp / fDiv;                   // convert
        }
        // assign
        switch ( ucField++ ) {
            case 0: Waypoint[uiWptNumber].Lon = fTemp; break;
            case 1: Waypoint[uiWptNumber].Lat = fTemp; break;
            case 2: Waypoint[uiWptNumber++].Alt = fTemp; break;
            default: break;
        }
    }
    return FALSE;
}
