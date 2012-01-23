//============================================================================+
//
// $RCSfile: nav.cpp,v $ (SOURCE FILE)
// $Revision: 1.4 $
// $Date: 2011/01/23 17:54:28 $
// $Author: Lorenz $
//
/// \brief Navigation task
///
/// \file
/// - Initialization:
///   reads from SD card a text file containing waypoints, translates strings
///   into coordinates and altitude, saves waypoints in the array Waypoint[],
///   updates number of available waypoints.
///   If SD card is missing or in case of error during file read, the number
///   of available waypoints is set to 0.
/// - Navigation:
///   waits for GPS fix, saves coordinates of launch point in the first entry
///   of array Waypoint[], computes heading and distance to next waypoint.
///   If available waypoints are 0, computes heading and distance to launch
///   point (RTL).
///
//  CHANGES updated header
//
//============================================================================*/


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "math.h"
#include "telemetry.h"
#include "config.h"
#include "gps.h"
#include "ff.h"
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

VAR_GLOBAL xQueueHandle xGps_Queue;

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC const char szFileName[16] = "path.txt";  // File name
VAR_STATIC char szLine[MAX_LINE_LENGTH];            // Input line
VAR_STATIC char pcBuffer[FILE_BUFFER_LENGTH];       // File data buffer
VAR_STATIC FATFS stFat;                             // FAT
VAR_STATIC FIL stFile;                              // File object
VAR_STATIC UINT wFileBytes;                         // Counter of read bytes
VAR_STATIC int16_t Bearing;                         // angle to destination [°]
VAR_STATIC uint16_t Distance;                       // distance to destination [m]
VAR_STATIC float fDestLat;                          // destination latitude
VAR_STATIC float fDestLon;                          // destination longitude
VAR_STATIC uint16_t uiWptIndex;                     // waypoint index
VAR_STATIC uint16_t uiWptNumber = 1;                // number of waypoints
VAR_STATIC STRUCT_WPT Waypoint[MAX_WAYPOINTS];      // waypoints array


float fCurrLat, fCurrLon;
unsigned char Gps_Status;

/*--------------------------------- Prototypes -------------------------------*/

bool Parse_Waypoint(char * pszLine);


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
/// Bearing formula works for short distances.
/// Bearing is positive in CCW direction, heading is positive in CW direction
/// Bearing reference is rotated 90° CW with respect to heading reference
///
///
//----------------------------------------------------------------------------
void Navigation_Task( void *pvParameters ) {

    float temp, dlat, dlon;
    char c, cCounter;
    char * pcBufferPointer;
    char * pszLinePointer;
    bool bError = TRUE;
    xGps_Message message;

    pszLinePointer = szLine;                        // Init line pointer
    pcBufferPointer = pcBuffer;                     // Init buffer pointer
    cCounter = MAX_LINE_LENGTH - 1;                 // Init char counter

    /* Mount file system and open waypoint file */

    if (FR_OK != f_mount(0, &stFat)) {              // Mount the file system.
        uiWptNumber = 0;                            // No waypoint available
    } else if (FR_OK != f_open(&stFile, szFileName, FA_READ)) {
        uiWptNumber = 0;                            // No waypoint available
    } else {
        bError = FALSE;                             // File succesfully open
    }

    /* Read waypoint file */
    while ((!bError) &&
           (FR_OK == f_read(&stFile, pcBuffer, FILE_BUFFER_LENGTH, &wFileBytes))) {
        while ((wFileBytes != 0) && (!bError)) {    // Buffer not empty and no error
            wFileBytes--;                           // Decrease overall counter
            c = *pcBufferPointer++;                 // Read another char
            if ( c == 13 ) {                        // Carriage return
                cCounter = 0;                       // Reached end of line
            } else if ( c == 10 ) {                 // New line
                cCounter--;                         // Decrease counter
            } else {
                *pszLinePointer++ = c;              // Copy char
                cCounter--;                         // Decrease counter
            }
            if (cCounter == 0) {                    // End of line
                *pszLinePointer = 0;                // Append line delimiter
                pszLinePointer = szLine;            // Reset line pointer
                cCounter = MAX_LINE_LENGTH - 1;     // Reset char counter
                bError = Parse_Waypoint(szLine);    // Parse line
            }
        }
    }
    f_close( &stFile );                             // Close file

    /* Wait first GPS fix and save launch position */
    while ((Gps_Status & GPS_STATUS_FIX) != GPS_STATUS_FIX) {
    }
    Waypoint[0].Lon = fCurrLon;                 // save launch position
    Waypoint[0].Lat = fCurrLat;
    if (uiWptNumber != 0) {                     // waypoint file available
        uiWptIndex = 1;                         // read first waypoint
    } else {                                    // no waypoint file
        uiWptIndex = 0;                         // read launch position
    }
    fDestLon = Waypoint[uiWptIndex].Lon;        //
    fDestLat = Waypoint[uiWptIndex].Lat;        //

    while (1) {
        while (xQueueReceive( xGps_Queue, &message, portMAX_DELAY ) != pdPASS) {
        }

        /* Calculate bearing to destination */
        dlon = (fDestLon - fCurrLon);
        dlat = (fDestLat - fCurrLat);
        Bearing = 90 - (int)((atan2f(dlat, dlon) * 180.0f) / PI);
        if (Bearing < 0) Bearing = Bearing + 360;

        /* compute distance to destination */
        temp = (sqrtf((dlat * dlat) + (dlon * dlon)) * 111113.7f);
        Distance = (unsigned int)temp;

        /* Waypoint reached: next waypoint */
        if (Distance < MIN_DISTANCE) {
            if ( uiWptNumber != 0 ) {
                if ( ++uiWptIndex == uiWptNumber ) {
                    uiWptIndex = 1;
                }
            }
            fDestLon = Waypoint[uiWptIndex].Lon;
            fDestLat = Waypoint[uiWptIndex].Lat;
        }
    }
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
uint16_t Nav_WaypointIndex ( void ) {
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
int16_t Nav_Bearing ( void ) {
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
uint16_t Nav_Distance ( void ) {
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
bool Parse_Waypoint ( char * pszLine ) {
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

