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
//  CHANGES modified computation of bank angle setpoint
//          added initialization of variables at beginning of navigation task
//          modified Nav_Ground_Speed(), returns speed in knots
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

#define BUFFER_LENGTH       64      // Length of buffer for file and USART
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

VAR_STATIC STRUCT_WPT Waypoint[MAX_WAYPOINTS];      // waypoints array
VAR_STATIC const char szFileName[16] = "path.txt";  // file name
VAR_STATIC char szLine[LINE_LENGTH];                // input line
VAR_STATIC char pcBuffer[BUFFER_LENGTH];            // data buffer
VAR_STATIC FATFS stFat;                             // FAT
VAR_STATIC FIL stFile;                              // file object
VAR_STATIC UINT wFileBytes;                         // counter of read bytes
VAR_STATIC int iLat_Int;                            // integer part of current latitude
VAR_STATIC int iLat_Dec;                            // decimal part of current latitude
VAR_STATIC int iLon_Int;                            // integer part of current longitude
VAR_STATIC int iLon_Dec;                            // decimal part of current longitude
VAR_STATIC float fBank;                             // bank angle setpoint [rad]
VAR_STATIC float fLat_Dest;                         // destination latitude
VAR_STATIC float fLon_Dest;                         // destination longitude
VAR_STATIC float fLat_Curr;                         // current latitude
VAR_STATIC float fLon_Curr;                         // current longitude
VAR_STATIC float fBearing;                          // angle to destination [°]
VAR_STATIC uint16_t uiHeading;                      // aircraft GPS heading [°]
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
static void Parse_Coord( int * integer, int * decimal, char c );
static bool Parse_GPS( void );

//----------------------------------------------------------------------------
//
/// \brief   navigation task
///
/// \remarks Computation must be started only when Parse_GPS() returns TRUE,
///          otherwise values of lat, lon, heading are unpredictable.
///
//----------------------------------------------------------------------------
void Navigation_Task( void *pvParameters ) {

    float temp, desired_x, desired_y, actual_x, actual_y, dot_prod, cross_prod;

    fBank = PI / 2.0f;                              // default bank angle
    fBearing = 0.0f;                                // angle to destination [°]
    uiHeading = 0;                                  // aircraft GPS heading [°]
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
    while ((ucGps_Status & GPS_STATUS_FIX) != GPS_STATUS_FIX) {
        Parse_GPS();                                // keep parsing NMEA sentences
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
        if (Parse_GPS()) {                          // new coordinate available

            /* Get X and Y components of bearing */
            desired_x = fLon_Dest - fLon_Curr;
            desired_y = fLat_Dest - fLat_Curr;

            /* Get X and Y components of heading */
            actual_x = DCM_Matrix[0][0];
            actual_y = DCM_Matrix[1][0];

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
            fBank = PID_Compute(&Nav_Pid, cross_prod , 0.0f) + (PI / 2.0f);

            /* Compute distance to destination */
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
/// \remarks pcBuffer[] array is used for file reading.
///          USART 2 must be disabled because it uses same array for reception.
///
//----------------------------------------------------------------------------
static void Load_Path( void ) {

    char c, cCounter;
    char * pcBufferPointer;
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
           (FR_OK == f_read(&stFile, pcBuffer, BUFFER_LENGTH, &wFileBytes))) {
        bError = (wFileBytes == 0);                 // force error if end of file
        pcBufferPointer = pcBuffer;                 // init buffer pointer
        while ((wFileBytes != 0) && (!bError)) {    // buffer not empty and no error
            wFileBytes--;                           // decrease overall counter
            c = *pcBufferPointer++;                 // read another char
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
/// \brief   Initialize navigation
/// \param   -
/// \return  -
/// \remarks configures USART2.
///          See http://www.micromouseonline.com/2009/12/31/stm32-usart-basics/#ixzz1eG1EE8bT
///          for direct register initialization of USART
///
//----------------------------------------------------------------------------
static void GPS_Init( void ) {

    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Initialize USART structure */
    USART_InitStructure.USART_BaudRate = 4800;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* Configure USART2 */
    USART_Init(USART2, &USART_InitStructure);

    /* Enable USART2 interrupt */
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

    /* Configure NVIC for USART 2 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);
/*
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN;              // enable clock for GPIOA
    GPIOA_CRH   |= (0x0BUL << 4);                   // Tx (PA9) alt. out push-pull
    GPIOA_CRH   |= (0x04UL << 8);                   // Rx (PA10) in floating
    RCC_APB2ENR |= RCC_APB2ENR_USART1EN;            // enable clock for USART1
    USART1_BRR  = 64000000L/115200L;                // set baudrate
    USART1_CR1 |= (USART1_CR1_RE | USART1_CR1_TE);  // RX, TX enable
    USART1_CR1 |= USART1_CR1_UE;                    // USART enable
*/
    ucGps_Status = GPS_STATUS_FIRST;                // init GPS status
    ucWindex = 0;                                   // clear write index
    ucRindex = 0;                                   // clear read index
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
/// \param   integer : (pointer to) integer part of coordinate
/// \param   decimal : (pointer to) decimal part of coordinate
/// \param   c       : character of NMEA sentence
/// \remarks -
///
//----------------------------------------------------------------------------
static void Parse_Coord( int * integer, int * decimal, char c )
{
    if ( c == ',' )  {          // comma delimiter
        *decimal = 0;           // clear decimal part
    } else if ( c != '.' ) {    // numeric character
        *decimal *= 10;         // multiply decimal part by 10
        *decimal += (c - '0');  // add digit to decimal part
    } else  {                   // decimal point
        *integer = *decimal;    // save integer part
        *decimal = 0;           // clear decimal part
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

        c = pcBuffer[ucRindex++];           // read character

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

        if ( ucCommas == 3 ) {              // get latitude data (3rd comma)
          Parse_Coord (&iLat_Int, &iLat_Dec, c);
        }

        if ( ucCommas == 5 ) {              // get longitude data (5th comma)
          Parse_Coord (&iLon_Int, &iLon_Dec, c);
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
            uiHeading = 0;
          } else if ( c != '.' ) {
            uiHeading *= 10;
            uiHeading += (c - '0');
          }
        }

        if ((ucCommas == 9) &&              // end of NMEA sentence
            (ucGps_Status & GPS_STATUS_FIX)) {
          ucCommas = 10;
          uiHeading /= 10;
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
		pcBuffer[ucWindex++] = USART_ReceiveData( USART2 );
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
uint16_t Nav_WaypointIndex ( void ) {
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
int16_t Nav_Bearing ( void ) {
  return (int16_t)fBearing;
}

//----------------------------------------------------------------------------
//
/// \brief   Get current heading [°]
/// \param   -
/// \returns heading angle in degrees, between 0° and 360°
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Nav_Heading ( void )
{
  return uiHeading;
}

//----------------------------------------------------------------------------
//
/// \brief   Get distance to destination [m]
/// \param   -
/// \returns distance in meters
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Nav_Distance ( void )
{
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
uint16_t Nav_Ground_Speed ( void )
{
  return uiSpeed;
}

