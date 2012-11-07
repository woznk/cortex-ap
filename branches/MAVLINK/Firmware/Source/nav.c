//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief navigation task
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
///   PID loop setpoint is the difference (bearing - heading), sign corrected
///   when < -180° or > 180°. Cross produt and dot product of heading vector
///   with bearing vector doesn't work because bearing vector is not a versor.
///
/// \todo
/// 1) Send configuration command to enable $GPGGA sentence only :
/// 25 25 f1 04 01 00 00 01 00 00 00 00 f5 0d 0a
///
/// \todo
/// 2) Compute longitude and latitude differences as :
/// \code
///     Delta Lat = Lat2 - Lat1
///     Delta Lon = (Lon2 - Lon1) * cos((Lat1 + Lat2)/2)
/// \endcode
///
/// \todo
/// 3) Compute distance from waypoint as :
/// \code
///     Distance = sqrt(Delta Lon ^ 2 + Delta Lat ^ 2) * 111320
/// \endcode
///
// Change: corrected parsing of waypoint data from SD card,
//         added functions Nav_Altitude(),
//         removed commented code
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include "bmp085_driver.h"
#include "ppmdriver.h"
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

#define MAX_WAYPOINTS       8       //!< Maximum number of waypoints
#define MIN_DISTANCE        100     //!< Minimum distance from waypoint [m]

#define BUFFER_LENGTH       96      //!< Length of buffer for file and USART
#define LINE_LENGTH         48      //!< Length of lines read from file

#define GPS_STATUS_FIX      1       //!< GPS status: satellite fix
#define GPS_STATUS_FIRST    2       //!< GPS status: waiting for first fix

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/// navigation modes
typedef enum {
    NAV_RTL,        //!< return to launch
    NAV_WPT         //!< waypoint following
} ENUM_NAV_MODE;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC uint8_t ucGpsBuffer[BUFFER_LENGTH];          //!< data buffer
VAR_STATIC STRUCT_WPT Waypoint[MAX_WAYPOINTS];         	//!< waypoints array
VAR_STATIC const uint8_t szFileName[16] = "path.txt";   //!< file name
VAR_STATIC uint8_t szLine[LINE_LENGTH];                 //!< input line
VAR_STATIC FATFS stFat;                                 //!< FAT
VAR_STATIC FIL stFile;                                  //!< file object
VAR_STATIC UINT wFileBytes;                             //!< counter of read bytes
VAR_STATIC float fBank;                                 //!< bank angle setpoint [rad]
VAR_STATIC float fThrottle;                             //!< throttle
VAR_STATIC float fPitch;                                //!< pitch setpoint [rad]
VAR_STATIC float fLat_Dest;                             //!< destination latitude
VAR_STATIC float fLon_Dest;                             //!< destination longitude
VAR_STATIC float fAlt_Dest;                             //!< destination altitude
VAR_STATIC float fLat_Curr;                             //!< current latitude
VAR_STATIC float fLon_Curr;                             //!< current longitude
VAR_STATIC float fAlt_Curr;                             //!< current altitude
VAR_STATIC float fLon_Temp;                             //!< temporary longitude during parse
VAR_STATIC float fLat_Temp;                             //!< temporary altitude during parse
VAR_STATIC float fBearing;                              //!< angle to destination [°]
VAR_STATIC float fHeading;                              //!< aircraft navigation heading [°]
VAR_STATIC uint16_t uiGps_Heading;                      //!< aircraft GPS heading [°]
VAR_STATIC uint32_t ulTempCoord;                        //!< temporary for coordinate parser
VAR_STATIC uint16_t uiSpeed;                            //!< speed [kt]
VAR_STATIC uint16_t uiDistance;                         //!< distance to destination [m]
VAR_STATIC uint8_t uiWptNumber;                         //!< total number of waypoints
VAR_STATIC uint8_t uiWptIndex;                          //!< waypoint index
VAR_STATIC uint8_t ucGps_Status;                        //!< status of GPS
VAR_STATIC uint8_t ucCommas;                            //!< counter of commas in NMEA sentence
VAR_STATIC uint8_t ucWindex;                            //!< USART buffer write index
VAR_STATIC uint8_t ucRindex;                            //!< USART buffer read index
VAR_STATIC xPID Nav_Pid;                                //!< Navigation PID loop
VAR_STATIC float fHeight_Margin = HEIGHT_MARGIN;        //!< altitude hold margin
VAR_STATIC float fThrottle_Min = ALT_HOLD_THROTTLE_MIN; //!< altitude hold min throttle
VAR_STATIC float fThrottle_Max = ALT_HOLD_THROTTLE_MAX; //!< altitude hold max throttle

/*--------------------------------- Prototypes -------------------------------*/

static void Load_Path( void );
static void GPS_Init( void );
static bool Parse_Waypoint ( uint8_t * pszLine );
static void Parse_Coord( float * fCoord, uint8_t c );
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

    float fTemp, fDx, fDy;

    fBank = PI / 2.0f;                              // default bank angle
    fBearing = 0.0f;                                // angle to destination [°]
    fThrottle = -1.0f;                              // default throttle
    fPitch = PI / 2.0f;                             // default pitch angle
    uiGps_Heading = 0;                              // aircraft GPS heading [°]
    uiSpeed = 0;                                    // speed [kt]
    uiDistance = 0;                                 // distance to destination [m]
    uiWptNumber = 0;                                // no waypoint yet

    Nav_Pid.fGain = PI / 6.0f;                      // limit bank angle to -30°, 30°
    Nav_Pid.fMin = -1.0f;                           //
    Nav_Pid.fMax = 1.0f;                            //
    Nav_Pid.fKp = NAV_KP;                           // init gains with default values
    Nav_Pid.fKi = NAV_KI;                           //
    Nav_Pid.fKd = NAV_KD;                           //
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
    fAlt_Dest = Waypoint[uiWptIndex].Alt;           // load destination altitude

    while (1) {
        if (Parse_GPS()) {                          // NMEA sentence completed

            /* Update PID gains and possibly reset PID */
            Nav_Pid.fKp = Telemetry_Get_Gain(TEL_NAV_KP);
            Nav_Pid.fKi = Telemetry_Get_Gain(TEL_NAV_KI);
            if (PPMGetMode() == MODE_MANUAL) {
                PID_Init(&Nav_Pid);
            }

#if (SIMULATOR == SIM_NONE)                             // normal mode
            fAlt_Curr = (float)BMP085_Get_Altitude();   // get barometric altitude
#else                                                   // simulation mode
            fAlt_Curr = Telemetry_Get_Altitude();       // get simulator altitude
#endif

            /* Get X and Y components of bearing */
            fDy = fLon_Dest - fLon_Curr;
            fDx = fLat_Dest - fLat_Curr;

            /* Compute heading */
            fHeading = atan2f(DCM_Matrix[1][0], DCM_Matrix[0][0]) / PI;

            /* Compute bearing to waypoint */
            fBearing = atan2f(fDy, fDx) / PI;

            /* Navigation PID controller */
            fTemp = fBearing - fHeading;
            if (fTemp < -1.0f) {
                fTemp += 2.0f;
            } else if (fTemp > 1.0f) {
                fTemp = 2.0f - fTemp;
            }
            fBank = (PI / 2.0f) - PID_Compute(&Nav_Pid, fTemp, 0.0f);

            /* Compute distance to waypoint */
            fTemp = sqrtf((fDy * fDy) + (fDx * fDx));
            uiDistance = (unsigned int)(fTemp * 111113.7f);

            /* Waypoint reached: next waypoint */
            if (uiDistance < MIN_DISTANCE) {
                if (uiWptNumber != 0) {
                    if (++uiWptIndex == uiWptNumber) {
                        uiWptIndex = 1;
                    }
                }
                fLon_Dest = Waypoint[uiWptIndex].Lon;   // new destination longitude
                fLat_Dest = Waypoint[uiWptIndex].Lat;   // new destination latitude
                fAlt_Dest = Waypoint[uiWptIndex].Alt;   // new destination altitude
            }

            /* Compute throttle and pitch based on altitude error */
            fTemp = fAlt_Curr - fAlt_Dest;              // altitude error
            if (fTemp > fHeight_Margin) {               // above maximum
                fThrottle = fThrottle_Min;              // minimum throttle
                fPitch = (PI / 2.0f) + PITCHATMINTHROTTLE;
            } else if (fTemp < - fHeight_Margin) {      // below minimum
                fThrottle = fThrottle_Max;              // max throttle
                fPitch = (PI / 2.0f) + PITCHATMAXTHROTTLE;
            } else {                                    // interpolate
                fTemp = (fTemp - HEIGHT_MARGIN) / (2 * HEIGHT_MARGIN);
                fThrottle = fTemp * (fThrottle_Min - fThrottle_Max) + fThrottle_Min;
                fPitch = (PI / 2.0f) + fTemp * (PITCHATMINTHROTTLE - PITCHATMAXTHROTTLE) + PITCHATMINTHROTTLE;
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

    uint8_t c, cCounter;
    uint8_t * ucGpsBufferPointer;
    uint8_t * pszLinePointer;
    bool bError = TRUE;

    pszLinePointer = szLine;                        // init line pointer
    cCounter = LINE_LENGTH - 1;                     // init char counter

    /* Mount file system and open waypoint file */
    if (FR_OK != f_mount(0, &stFat)) {              // file system not mounted
        uiWptNumber = 0;                            // no waypoint available
    } else if (FR_OK != f_open(&stFile,
                               (const XCHAR *)szFileName,
                                FA_READ)) {         // error opening file
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
            if ( c == 10 ) {                        // found new line
                cCounter = 0;                       // reached end of line
            } else if ( c == 13 ) {                 // found carriage return
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
    ( void )f_close( &stFile );                     // close file
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
static bool Parse_Waypoint ( uint8_t * pszLine ) {
    uint8_t c;
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
static void Parse_Coord( float * fCoord, uint8_t c )
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
/// \remarks structure of NMEA sentences
///
///    $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
///    1    = UTC of position fix
///    2    = Data status (V=navigation receiver warning)
///    3    = Latitude of fix
///    4    = N or S
///    5    = Longitude of fix
///    6    = E or W
///    7    = Speed over ground [kts]
///    8    = Track made good in [deg] True
///    9    = UT date
///    10   = Magnetic variation [deg] (Easterly var. subtracts from true course)
///    11   = E or W
///    12   = Checksum
///
///    $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
///    1    = UTC of Position
///    2    = Latitude
///    3    = N or S
///    4    = Longitude
///    5    = E or W
///    6    = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
///    7    = Number of satellites in use [not those in view]
///    8    = Horizontal dilution of position
///    9    = Antenna altitude above/below mean sea level (geoid)
///    10   = Meters (Antenna height unit)
///    11   = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
///           mean sea level.-= geoid is below WGS-84 ellipsoid)
///    12   = Meters  (Units of geoidal separation)
///    13   = Age in seconds since last update from diff. reference station
///    14   = Differential reference station ID#
///    15   = Checksum
///
//----------------------------------------------------------------------------
static bool Parse_GPS( void )
{
    uint8_t c;
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
          Parse_Coord (&fLat_Temp, c);
        }

        if (( ucCommas == 5 ) ||
            ( ucCommas == 6 )) {            // get longitude data (5th comma)
          Parse_Coord (&fLon_Temp, c);
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
          fLat_Curr = fLat_Temp;
          fLon_Curr = fLon_Temp;
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

