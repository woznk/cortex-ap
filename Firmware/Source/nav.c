//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
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
//  CHANGES corrected again parsing of GPS coordinates,
//          tentatively computed heading from DCM matrix (not working)
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "dcm.h"
#include "math.h"
#include "telemetry.h"
#include "config.h"
#include "ff.h"
#include "pid.h"
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

#define MAX_WAYPOINTS       8       // Maximum number of waypoints
#define MIN_DISTANCE        100     // Minimum distance from waypoint [m]

#define BUFFER_LENGTH       96      // Length of buffer for file and USART
#define LINE_LENGTH         48      // Length of lines read from file

#define GPS_STATUS_FIX      1       // GPS status: satellite fix
#define GPS_STATUS_FIRST    2       // GPS status: waiting for first fix

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

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC char ucGpsBuffer[BUFFER_LENGTH];
//"$GPRMC,194617.04,A,4534.6714,N,01128.8559,E,000.0,287.0,091008,001.9,E,A*31\n";
//"$GPRMC,194618.04,A,4534.6714,N,01128.8559,E,000.0,287.0,091008,001.9,E,A*3E\n";         // data buffer
VAR_STATIC STRUCT_WPT Waypoint[MAX_WAYPOINTS];      // waypoints array
VAR_STATIC const char szFileName[16] = "path.txt";  // file name
VAR_STATIC char szLine[LINE_LENGTH];                // input line
VAR_STATIC FATFS stFat;                             // FAT
VAR_STATIC FIL stFile;                              // file object
VAR_STATIC UINT wFileBytes;                         // counter of read bytes
VAR_STATIC float fBank;                             // bank angle setpoint [rad]
VAR_STATIC float fLat_Dest;                         // destination latitude
VAR_STATIC float fLon_Dest;                         // destination longitude
VAR_STATIC float fLat_Curr;                         // current latitude
VAR_STATIC float fLon_Curr;                         // current longitude
VAR_STATIC float fBearing;                          // angle to destination [°]
VAR_STATIC float fHeading;                          // aircraft navigation heading [°]
VAR_STATIC uint16_t uiGps_Heading;                  // aircraft GPS heading [°]
VAR_STATIC uint32_t ulTempCoord;                    // temporary for coordinate parser
VAR_STATIC uint16_t uiSpeed;                        // speed [kt]
VAR_STATIC uint16_t uiDistance;                     // distance to destination [m]
VAR_STATIC uint16_t uiWptIndex;                     // waypoint index
VAR_STATIC uint16_t uiWptNumber;                    // total number of waypoints
VAR_STATIC uint8_t ucGps_Status;                    // status of GPS
VAR_STATIC uint8_t ucCommas;                        // counter of commas in NMEA sentence
VAR_STATIC uint8_t ucWindex;                        // USART buffer write index
VAR_STATIC uint8_t ucRindex;                        // USART buffer read index
VAR_STATIC xPID Nav_Pid;                            // Navigation PID loop

/*--------------------------------- Prototypes -------------------------------*/

static void Load_Path( void );
static void GPS_Init( void );
static bool Parse_Waypoint ( char * pszLine );
static void Parse_Coord( float * fCoord, char c );
static bool Parse_GPS( void );

//----------------------------------------------------------------------------
//
/// \brief   navigation task
///
/// \remarks Computation must be started only when Parse_GPS() returns TRUE,
///          otherwise values of lat, lon, heading are undefined.
///
//----------------------------------------------------------------------------
void Navigation_Task( void *pvParameters ) {

    float temp, desired_x, desired_y, actual_x, actual_y, dot_prod, cross_prod;

    fBank = PI / 2.0f;                              // default bank angle
    fBearing = 0.0f;                                // angle to destination [°]
    uiGps_Heading = 0;                              // aircraft GPS heading [°]
    uiSpeed = 0;                                    // speed [kt]
    uiDistance = 0;                                 // distance to destination [m]
    uiWptNumber = 0;                                // no waypoint yet

    Nav_Pid.fGain = PI / 6.0f;                      // limit bank angle to 30°
    Nav_Pid.fMin = -1.0f;
    Nav_Pid.fMax = 1.0f;
    Nav_Pid.fKp = 1.0f;
    Nav_Pid.fKi = 0.0f;
    Nav_Pid.fKd = 0.0f;

    PID_Init(&Nav_Pid);                             // initialize navigation PID
    Load_Path();                                    // load path from SD card
    GPS_Init();                                     // initialize USART for GPS

    /* Wait first GPS fix */
    while ((Parse_GPS() == FALSE) &&                // NMEA sentence not completed
          ((ucGps_Status & GPS_STATUS_FIX) == 0)) { // no satellite fix
        ;                                           // keep parsing NMEA sentences
    }

    /* Save launch position */
    Waypoint[0].Lon = fLon_Curr;                    // save launch position
    Waypoint[0].Lat = fLat_Curr;
    if (uiWptNumber != 0) {                         // waypoint file available
        uiWptIndex = 1;                             // read first waypoint
    } else {                                        // no waypoint file
        uiWptIndex = 0;                             // use launch position
    }
    fLon_Dest = Waypoint[uiWptIndex].Lon;           // load destination longitude
    fLat_Dest = Waypoint[uiWptIndex].Lat;           // load destination latitude

    while (1) {
        if (Parse_GPS()) {                          // NMEA sentence completed

            /* Update PID gains */
            Nav_Pid.fKp = Telemetry_Get_Gain(TEL_NAV_KP);
            Nav_Pid.fKi = Telemetry_Get_Gain(TEL_NAV_KI);

            /* Get X and Y components of bearing */
            desired_x = fLon_Dest - fLon_Curr;
            desired_y = fLat_Dest - fLat_Curr;

            /* Get X and Y components of heading */
            actual_x = DCM_Matrix[0][0];
            actual_y = DCM_Matrix[1][0];

            /* Compute heading */
            fHeading = (atan2f(actual_x, actual_y) * 180.0f) / PI;
            if (fHeading < 0.0f) fHeading = fHeading + 360.0f;

            /* Compute bearing to waypoint */
            fBearing = 90.0f - ((atan2f(desired_y, desired_x) * 180.0f) / PI);
            if (fBearing < 0.0f) fBearing = fBearing + 360.0f;

            /* Compute cosine of angle between bearing and heading */
            dot_prod = (actual_x * desired_x) + (actual_y * desired_y);

            /* Compute sine of angle between bearing and heading */
            cross_prod = (actual_x * desired_y) - (actual_y * desired_x);

            /* Saturate sine between -1 and 1 */
            if (dot_prod < 0.0f) {                  // angle is outside -90°, +90°
                if (cross_prod >= 0.0f) {           // angle is above 90°
                    cross_prod = 1.0f;              // saturate at 90°
                } else {                            // angle is below -90°
                    cross_prod = -1.0f;             // saturate at -90°
                }
            }

            /* Navigation PID controller */
            fBank = (PI / 2.0f) + PID_Compute(&Nav_Pid, cross_prod , 0.0f);

            /* Compute distance to waypoint */
            temp = sqrtf((desired_y * desired_y) + (desired_x * desired_x));
            uiDistance = (unsigned int)(temp * 111113.7f);

            /* Waypoint reached: next waypoint */
            if (uiDistance < MIN_DISTANCE) {
                if (uiWptNumber != 0) {
                    if (++uiWptIndex == uiWptNumber) {
                        uiWptIndex = 1;
                    }
                }
                fLon_Dest = Waypoint[uiWptIndex].Lon;
                fLat_Dest = Waypoint[uiWptIndex].Lat;
            }
        }
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Load path from file on SD card
/// \param   -
/// \return  -
/// \remarks ucGpsBuffer[] array is used for file reading.
///          USART 2 must be disabled because it uses same array for reception.
///
//----------------------------------------------------------------------------
static void Load_Path( void ) {

    char c, cCounter;
    char * ucGpsBufferPointer;
    char * pszLinePointer;
    bool bError = TRUE;

    pszLinePointer = szLine;                        // init line pointer
    cCounter = LINE_LENGTH - 1;                     // init char counter

    /* Mount file system and open waypoint file */
    if (FR_OK != f_mount(0, &stFat)) {              // file system not mounted
        uiWptNumber = 0;                            // no waypoint available
    } else if (FR_OK != f_open(&stFile, szFileName, FA_READ)) { // error opening file
        uiWptNumber = 0;                            // no waypoint available
    } else {                                        //
        bError = FALSE;                             // file succesfully open
    }

    /* Read waypoint file */
    while ((!bError) &&
           (FR_OK == f_read(&stFile, ucGpsBuffer, BUFFER_LENGTH, &wFileBytes))) {
        bError = (wFileBytes == 0);                 // force error if end of file
        ucGpsBufferPointer = ucGpsBuffer;           // init buffer pointer
        while ((wFileBytes != 0) && (!bError)) {    // buffer not empty and no error
            wFileBytes--;                           // decrease overall counter
            c = *ucGpsBufferPointer++;              // read another char
            if ( c == 13 ) {                        // found carriage return
                cCounter = 0;                       // reached end of line
            } else if ( c == 10 ) {                 // found new line
                cCounter--;                         // decrease counter
            } else {                                // alphanumeric character
                *pszLinePointer++ = c;              // copy character
                cCounter--;                         // decrease counter
            }
            if (cCounter == 0) {                    // end of line
                *pszLinePointer = 0;                // append line delimiter
                pszLinePointer = szLine;            // reset line pointer
                cCounter = LINE_LENGTH - 1;         // reset char counter
                bError = Parse_Waypoint(szLine);    // parse line for waypoint
            }
        }
    }
    f_close( &stFile );                             // close file
}

//----------------------------------------------------------------------------
//
/// \brief   Initialize GPS
/// \param   -
/// \return  -
/// \remarks configures USART2 for receiving GPS data and initializes indexes.
///          If simulator option is active, only indexes are initialized.
///          For direct register initialization of USART see:
/// http://www.micromouseonline.com/2009/12/31/stm32-usart-basics/#ixzz1eG1EE8bT
///
//----------------------------------------------------------------------------
static void GPS_Init( void ) {

    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    ucGps_Status = GPS_STATUS_FIRST;                // init GPS status
    ucWindex = 0;                                   // clear write index
    ucRindex = 0;                                   // clear read index
    ulTempCoord = 0UL;                              // clear temporary coordinate

    /* Initialize USART structure */
    USART_InitStructure.USART_BaudRate = 4800;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART2, &USART_InitStructure);       // configure USART2

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);  // enable USART2 interrupt
    //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

    /* Configure NVIC for USART 2 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    USART_Cmd(USART2, ENABLE);                      // enable the USART2
}


//----------------------------------------------------------------------------
//
/// \brief   Parse string for waypoint coordinates
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
static bool Parse_Waypoint ( char * pszLine ) {
    char c;
    float fTemp, fDiv;
    uint8_t ucField = 0, ucCounter = LINE_LENGTH;

    while (( ucField < 3 ) && ( ucCounter > 0 )) {
        fDiv = 1.0f;                                // initialize divisor
        fTemp = 0.0f;                               // initialize temporary
        c = *pszLine++;                             // initialize char
        /* leading spaces */
        while (( c == ' ' ) && ( ucCounter > 0 )) {
            c = *pszLine++;                         // next char
            ucCounter--;                            // count characters
        }
        /* start of integer part */
        if (( c < '0' ) || ( c > '9' )) {           //
            return TRUE;                            // first char not numeric
        }
        /* integer part */
        while (( c >= '0' ) && ( c <= '9' ) && ( ucCounter > 0 )) {
            fTemp = fTemp * 10.0f + (float)(c - '0'); // accumulate
            c = *pszLine++;                         // next char
            ucCounter--;                            // count characters
        }
        /* decimal point */
        if (( c != '.' ) && ( ucField != 2 )) {     // altitude may lack decimal
            return TRUE;
        } else {
            c = *pszLine++;                         // skip decimal point
            ucCounter--;                            // count characters
        }
        /* fractional part */
        while (( c >= '0' ) && ( c <= '9' ) && ( ucCounter > 0 )) {
            if (fDiv < 1000000.0f) {
                fTemp = fTemp * 10.0f + (float)(c - '0'); // accumulate
                fDiv = fDiv * 10.0f;                // update divisor
            }
            c = *pszLine++;                         // next char
            ucCounter--;                            // count characters
        }
        /* delimiter */
        if (( c != ',' ) && ( c != 0 )) {
            return TRUE;                            // error
        } else {
            fTemp = fTemp / fDiv;                   // convert
        }
        /* assign */
        switch ( ucField++ ) {
            case 0: Waypoint[uiWptNumber].Lon = fTemp; break;
            case 1: Waypoint[uiWptNumber].Lat = fTemp; break;
            case 2: Waypoint[uiWptNumber++].Alt = fTemp; break;
            default: break;
        }
    }
    return FALSE;
}

//----------------------------------------------------------------------------
//
/// \brief   Parse NMEA sentence for coordinates
///
/// \param   float : (pointer to) coordinate
/// \param   c     : character of NMEA sentence
/// \remarks -
///
//----------------------------------------------------------------------------
static void Parse_Coord( float * fCoord, char c )
{
    switch (c) {
        case '0' :
        case '1' :
        case '2' :
        case '3' :
        case '4' :
        case '5' :
        case '6' :
        case '7' :
        case '8' :
        case '9' :
            ulTempCoord = ulTempCoord * 10UL + (uint32_t)(c - '0');
            break;
        case '.' :
            *fCoord = (float)(ulTempCoord % 100UL) / 60.0f;  // decimal part
            *fCoord += (float)(ulTempCoord / 100UL);         // integer part
            ulTempCoord = 0UL;
            break;
        case ',' :
            *fCoord += (float)ulTempCoord / 600000.0f;       // decimal part
            ulTempCoord = 0UL;
            break;
        default :
            ulTempCoord = 0UL;
            break;
    }
}


//----------------------------------------------------------------------------
//
/// \brief   Parse GPS sentences
/// \param   -
/// \returns true if new coordinate data are available, false otherwise
/// \remarks -
///
//----------------------------------------------------------------------------
static bool Parse_GPS( void )
{
    char c;
    bool bResult = FALSE;

    while ((bResult == FALSE) &&            // NMEA sentence not completed
           (ucRindex != ucWindex)) {        // received another character

        c = ucGpsBuffer[ucRindex++];        // read character

        if (ucRindex >= BUFFER_LENGTH) {    // update read index
            ucRindex = 0;
        }

        if ( c == '$' ) ucCommas = 0;       // start of NMEA sentence

        if ( c == ',' ) ucCommas++;         // count commas

        if ( ucCommas == 2 ) {              // get fix info
           if (c == 'A') {
             ucGps_Status |= GPS_STATUS_FIX;
           } else if (c == 'V') {
             ucGps_Status &= ~GPS_STATUS_FIX;
           }
        }

        if (( ucCommas == 3 ) ||
            ( ucCommas == 4 )) {            // get latitude data (3rd comma)
          Parse_Coord (&fLat_Curr, c);
        }

        if (( ucCommas == 5 ) ||
            ( ucCommas == 6 )) {            // get longitude data (5th comma)
          Parse_Coord (&fLon_Curr, c);
        }

        if ( ucCommas == 7 ) {              // get speed (7th comma)
          if ( c == ',' ) {
            uiSpeed = 0;
          } else if ( c != '.' ) {
            uiSpeed *= 10;
            uiSpeed += (c - '0');
          }
        }

        if ( ucCommas == 8 ) {              // get heading (8th comma)
          if ( c == ',' ) {
            uiGps_Heading = 0;
          } else if ( c != '.' ) {
            uiGps_Heading *= 10;
            uiGps_Heading += (c - '0');
          }
        }

        if ((ucCommas == 9) &&              // end of NMEA sentence
            (ucGps_Status & GPS_STATUS_FIX)) {
          ucCommas = 10;
          uiGps_Heading /= 10;
          bResult = TRUE;
        }
    }
    return bResult;
}

//----------------------------------------------------------------------------
//
/// \brief   GPS USART interrupt handler
/// \param   -
/// \returns -
/// \remarks -
///
//----------------------------------------------------------------------------
void USART2_IRQHandler( void )
{
//  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//  portCHAR cChar;

	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
//		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
		ucGpsBuffer[ucWindex++] = USART_ReceiveData( USART2 );
        if (ucWindex >= BUFFER_LENGTH) {
            ucWindex = 0;
        }
	}
//	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

//----------------------------------------------------------------------------
//
/// \brief   Get waypoint index
/// \param   -
/// \returns waypoint index
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Nav_Wpt_Index ( void ) {
  return uiWptIndex;
}


//----------------------------------------------------------------------------
//
/// \brief   Get computed bearing [°]
/// \param   -
/// \returns bearing angle in degrees, between -180° and + 180°
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Bearing ( void ) {
  return fBearing;
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
  return fHeading;
}

//----------------------------------------------------------------------------
//
/// \brief   Get GPS heading [°]
/// \param   -
/// \returns heading angle in degrees, between 0° and 360°
/// \remarks -
///
//----------------------------------------------------------------------------
float Gps_Heading ( void ) {
  return uiGps_Heading;
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
/// \brief   Get ground speed detected by GPS [kt]
/// \param   -
/// \returns GS in knots
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Nav_Ground_Speed ( void ) {
  return uiSpeed;
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
/// \brief   Get bank angle [rad]
/// \param   -
/// \returns bank angle
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Bank ( void ) {
  return fBank;
}

