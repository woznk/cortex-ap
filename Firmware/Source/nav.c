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
/// 1) Compute longitude and latitude differences as :
/// \code
///     Delta Lat = Lat2 - Lat1
///     Delta Lon = (Lon2 - Lon1) * cos((Lat1 + Lat2)/2)
/// \endcode
///
/// \todo
/// 2) Compute distance from waypoint as :
/// \code
///     Distance = sqrt(Delta Lon ^ 2 + Delta Lat ^ 2) * 111320
/// \endcode
///
/// Change: added parsing of $GPGGA sentence
///         GPS altitude temporarily assumed as aircraft altitude 
///         some major renaming according coding rules
//
//============================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
#include "misc.h"
#include "bmp085_driver.h"
#include "ppmdriver.h"
#include "dcm.h"
#include "math.h"
#include "simulator.h"
#include "mav_telemetry.h"
#include "attitude.h"
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

#define USART2_DR_Base  0x40004404

#define MAX_WAYPOINTS   8       //!< Maximum number of waypoints
#define MIN_DISTANCE    100     //!< Minimum distance from waypoint [m]

#define BUFFER_LENGTH   96      //!< Length of buffer for file and USART
#define LINE_LENGTH     48      //!< Length of lines read from file

#define GPS_FIX         3       //!< GPS status: satellite fix
#define GPS_NOFIX       0       //!< GPS status: waiting for first fix

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/// navigation modes
typedef enum {
    NAV_RTL,        //!< return to launch
    NAV_WPT         //!< waypoint following
} ENUM_NAV_MODE;

/// NMEA string types
typedef enum {
    NMEA_GPRMC,     //!< recommended minimum specific GPS/transit data
    NMEA_GPGGA,     //!< global positioning system fix data
    NMEA_INVALID    //!< invalid NMEA string
} ENUM_NMEA_TYPE;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC uint8_t ucGpsBuffer[BUFFER_LENGTH];          //!< gps data buffer
VAR_STATIC STRUCT_WPT Waypoint[MAX_WAYPOINTS];         	//!< waypoints array
VAR_STATIC const uint8_t szFile[16] = "path.txt";     //!< file name
VAR_STATIC uint8_t szLine[LINE_LENGTH];                 //!< input line
VAR_STATIC FATFS stFat;                                 //!< FAT
VAR_STATIC FIL stFile;                                  //!< file object
VAR_STATIC UINT wFile_Bytes;                            //!< counter of read bytes
VAR_STATIC float fBank;                                 //!< bank angle setpoint [rad]
VAR_STATIC float fThrottle;                             //!< throttle
VAR_STATIC float fPitch;                                //!< pitch setpoint [rad]
VAR_STATIC float fDest_Lat;                             //!< destination latitude
VAR_STATIC float fDest_Lon;                             //!< destination longitude
VAR_STATIC float fDest_Alt;                             //!< destination altitude
VAR_STATIC float fCurr_Lat;                             //!< current latitude
VAR_STATIC float fCurr_Lon;                             //!< current longitude
VAR_STATIC float fCurr_Alt;                             //!< current altitude
VAR_STATIC float fTemp_Lon;                             //!< temporary longitude during parse
VAR_STATIC float fTemp_Lat;                             //!< temporary latitude during parse
VAR_STATIC float fBearing;                              //!< angle to destination [°]
VAR_STATIC float fHeading;                              //!< aircraft navigation heading [°]
VAR_STATIC uint32_t ulTemp_Coord;                       //!< temporary for coordinate parser
VAR_STATIC uint16_t uiDistance;                         //!< distance to destination [m]
VAR_STATIC uint16_t uiWpt_Number;                       //!< total number of waypoints
VAR_STATIC uint16_t uiWpt_Index;                        //!< waypoint index
VAR_STATIC uint16_t uiGps_Heading;                      //!< aircraft GPS heading [°]
VAR_STATIC uint16_t uiGps_Speed;                        //!< speed [kt/10]
VAR_STATIC uint16_t uiGps_Alt;                          //!< altitude [m]
VAR_STATIC uint8_t ucGps_Status;                        //!< status of GPS
VAR_STATIC uint8_t ucWindex;                            //!< USART buffer write index
VAR_STATIC uint8_t ucRindex;                            //!< USART buffer read index
VAR_STATIC xPID Nav_Pid;                                //!< Navigation PID loop
VAR_STATIC float fHeight_Margin = HEIGHT_MARGIN;        //!< altitude hold margin
VAR_STATIC float fThrottle_Min = ALT_HOLD_THROTTLE_MIN; //!< altitude hold min throttle
VAR_STATIC float fThrottle_Max = ALT_HOLD_THROTTLE_MAX; //!< altitude hold max throttle

/*--------------------------------- Prototypes -------------------------------*/

static void load_path( void );
static void gps_init( void );
static bool parse_waypoint ( uint8_t * pszline );
static void parse_coord( float * fCoord, uint8_t c );
static bool parse_gps( void );
static bool cmp_prefix( uint8_t * src , const uint8_t * dest );

//----------------------------------------------------------------------------
//
/// \brief   navigation task
///
/// \remarks Computation must be started only when parse_gps() returns TRUE,
///          otherwise values of lat, lon, heading are undefined.
///
//----------------------------------------------------------------------------
void Navigation_Task( void *pvParameters ) {

    float ftemp, fdx, fdy;

    fBank = 0.0f;                                   // default bank angle
    fBearing = 0.0f;                                // angle to destination [°]
    fThrottle = MINIMUMTHROTTLE;                    // default throttle
    fPitch = 0.0f;                                  // default pitch angle
    uiGps_Heading = 0;                              // aircraft GPS heading [°]
    uiGps_Speed = 0;                                // aircraft GPS speed [kt]
    uiDistance = 0;                                 // distance to destination [m]
    uiWpt_Number = 0;                               // no waypoint yet

    Nav_Pid.fGain = PI / 6.0f;                      // limit bank angle to -30°, 30°
    Nav_Pid.fMin = -1.0f;                           //
    Nav_Pid.fMax = 1.0f;                            //
    Nav_Pid.fKp = NAV_KP;                           // init gains with default values
    Nav_Pid.fKi = NAV_KI;                           //
    Nav_Pid.fKd = NAV_KD;                           //
    PID_Init(&Nav_Pid);                             // initialize navigation PID
    load_path();                                    // load path from SD card
    /* WARNING: GPS UART must be initialized after loading mission file !!! */
    gps_init();                                     // initialize USART for GPS

    /* Wait first GPS fix */
    while ((parse_gps() == FALSE) ||                // NMEA sentence not completed
           (ucGps_Status != GPS_FIX)) {             // no satellite fix
        ;                                           // keep parsing NMEA sentences
    }

    /* Save launch position */
    Waypoint[0].Lon = fCurr_Lon;                    // save launch position
    Waypoint[0].Lat = fCurr_Lat;
    if (uiWpt_Number != 0) {                        // waypoint file available
        uiWpt_Index = 1;                            // read first waypoint
    } else {                                        // no waypoint file
        uiWpt_Index = 0;                            // use launch position
    }
    fDest_Lon = Waypoint[uiWpt_Index].Lon;          // load destination longitude
    fDest_Lat = Waypoint[uiWpt_Index].Lat;          // load destination latitude
    fDest_Alt = Waypoint[uiWpt_Index].Alt;          // load destination altitude

    while (1) {
        if (parse_gps()) {                          // NMEA sentence completed

#if (SIMULATOR == SIM_NONE)                                // normal mode
            Nav_Pid.fKp = Telemetry_Get_Gain(TEL_NAV_KP);  // update PID gains
            Nav_Pid.fKi = Telemetry_Get_Gain(TEL_NAV_KI);
//            fCurr_Alt = (float)BMP085_Get_Altitude();    // get barometric altitude
            fCurr_Alt = (float)uiGps_Alt;                  // get GPS altitude
#else                                                      // simulation mode
            Nav_Pid.fKp = Simulator_Get_Gain(SIM_NAV_KP);  // update PID gains
            Nav_Pid.fKi = Simulator_Get_Gain(SIM_NAV_KI);
            fCurr_Alt = Simulator_Get_Altitude();          // get simulator altitude
#endif
            if (PPMGetMode() == MODE_MANUAL) {             // possibly reset PID
                PID_Init(&Nav_Pid);
            }

            /* Get X and Y components of bearing */
            fdy = fDest_Lon - fCurr_Lon;
            fdx = fDest_Lat - fCurr_Lat;

            /* Compute heading */
            fHeading = Attitude_Yaw_Rad() / PI;

            /* Compute bearing to waypoint */
            fBearing = atan2f(fdy, fdx) / PI;

            /* Navigation PID controller */
            ftemp = fHeading - fBearing;
            if (ftemp < -1.0f) {
                ftemp += 2.0f;
            } else if (ftemp > 1.0f) {
                ftemp = 2.0f - ftemp;
            }
            fBank = PID_Compute(&Nav_Pid, ftemp, 0.0f);

            /* Compute distance to waypoint */
            ftemp = sqrtf((fdy * fdy) + (fdx * fdx));
            uiDistance = (unsigned int)(ftemp * 111113.7f);

            /* Waypoint reached: next waypoint */
            if (uiDistance < MIN_DISTANCE) {
                if (uiWpt_Number != 0) {
                    if (++uiWpt_Index == uiWpt_Number) {
                        uiWpt_Index = 1;
                    }
                }
                fDest_Lon = Waypoint[uiWpt_Index].Lon;   // new destination longitude
                fDest_Lat = Waypoint[uiWpt_Index].Lat;   // new destination latitude
                fDest_Alt = Waypoint[uiWpt_Index].Alt;   // new destination altitude
            }

            /* Compute throttle and pitch based on altitude error */
            ftemp = fCurr_Alt - fDest_Alt;              // altitude error
            if (ftemp > fHeight_Margin) {               // above maximum
                fThrottle = fThrottle_Min;              // minimum throttle
                fPitch = PITCHATMINTHROTTLE;
            } else if (ftemp < - fHeight_Margin) {      // below minimum
                fThrottle = fThrottle_Max;              // max throttle
                fPitch = PITCHATMAXTHROTTLE;
            } else {                                    // interpolate
                ftemp = (ftemp - HEIGHT_MARGIN) / (2 * HEIGHT_MARGIN);
                fThrottle = ftemp * (fThrottle_Min - fThrottle_Max) + fThrottle_Min;
                fPitch = ftemp * (PITCHATMINTHROTTLE - PITCHATMAXTHROTTLE) + PITCHATMINTHROTTLE;
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
static void load_path( void ) {

    uint8_t c, uccounter;
    uint8_t * pgps_buffer_pointer;
    uint8_t * pszline_pointer;
    bool berror = TRUE;

    pszline_pointer = szLine;                        // init line pointer
    uccounter = LINE_LENGTH - 1;                     // init char counter

    /* Mount file system and open waypoint file */
    if (FR_OK != f_mount(0, &stFat)) {              // file system not mounted
        uiWpt_Number = 0;                           // no waypoint available
    } else if (FR_OK != f_open(&stFile, (const XCHAR *)szFile,FA_READ)) {
                                                    // error opening file
        uiWpt_Number = 0;                           // no waypoint available
    } else {                                        //
        berror = FALSE;                             // file succesfully open
        uiWpt_Number = 1;                           // waypoint available
    }

    /* Read waypoint file */
    while ((!berror) &&
           (FR_OK == f_read(&stFile, ucGpsBuffer, BUFFER_LENGTH, &wFile_Bytes))) {
        berror = (wFile_Bytes == 0);                // force error if end of file
        pgps_buffer_pointer = ucGpsBuffer;          // init buffer pointer
        while ((wFile_Bytes != 0) && (!berror)) {   // buffer not empty and no error
            wFile_Bytes--;                          // decrease overall counter
            c = *pgps_buffer_pointer++;             // read another char
            if ( c == 10 ) {                        // found new line
                uccounter = 0;                      // reached end of line
            } else if ( c == 13 ) {                 // found carriage return
                uccounter--;                        // decrease counter
            } else {                                // alphanumeric character
                *pszline_pointer++ = c;             // copy character
                uccounter--;                        // decrease counter
            }
            if (uccounter == 0) {                   // end of line
                *pszline_pointer = 0;               // append line delimiter
                pszline_pointer = szLine;           // reset line pointer
                uccounter = LINE_LENGTH - 1;        // reset char counter
                berror = parse_waypoint(szLine);    // parse line for waypoint
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
///          For direct register initialization of USART see:
/// http://www.micromouseonline.com/2009/12/31/stm32-usart-basics/#ixzz1eG1EE8bT
///
//----------------------------------------------------------------------------
static void gps_init( void ) {

    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    ucGps_Status = GPS_NOFIX;                       // init GPS status
    ucWindex = 0;                                   // clear write index
    ucRindex = 0;                                   // clear read index
    ulTemp_Coord = 0UL;                             // clear temporary coordinate

    /* Initialize USART2 */
    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);                      // enable the USART2

    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);  // enable USART2 RX DMA request

    /* Initialize DMA1 channel, triggered by UART 2 RX */
    DMA_DeInit(DMA1_Channel6);
    DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Base;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ucGpsBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = BUFFER_LENGTH;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel6, ENABLE);                 // enable USART2 RX DMA channel

    /* Enable DMA 1 half transfer- and transfer complete interrupts */
	DMA_ITConfig(DMA1_Channel6, DMA_IT_TC | DMA_IT_HT, ENABLE);

    /* Initialize DMA interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_EnableIRQ(DMA1_Channel6_IRQn);             // enable DMA interrupt
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
static bool parse_waypoint ( uint8_t * pszline ) {

    uint8_t c, ucfield = 0, uccounter = LINE_LENGTH;
    float ftemp, fdiv;

    while (( ucfield < 3 ) && ( uccounter > 0 )) {
        fdiv = 1.0f;                                // initialize divisor
        ftemp = 0.0f;                               // initialize temporary
        c = *pszline++;                             // initialize char
        /* leading spaces */
        while (( c == ' ' ) && ( uccounter > 0 )) {
            c = *pszline++;                         // next char
            uccounter--;                            // count characters
        }
        /* start of integer part */
        if (( c < '0' ) || ( c > '9' )) {           //
            return TRUE;                            // first char not numeric
        }
        /* integer part */
        while (( c >= '0' ) && ( c <= '9' ) && ( uccounter > 0 )) {
            ftemp = ftemp * 10.0f + (float)(c - '0'); // accumulate
            c = *pszline++;                         // next char
            uccounter--;                            // count characters
        }
        /* decimal point */
        if (( c != '.' ) && ( ucfield != 2 )) {     // altitude may lack decimal
            return TRUE;
        } else {
            c = *pszline++;                         // skip decimal point
            uccounter--;                            // count characters
        }
        /* fractional part */
        while (( c >= '0' ) && ( c <= '9' ) && ( uccounter > 0 )) {
            if (fdiv < 1000000.0f) {
                ftemp = ftemp * 10.0f + (float)(c - '0'); // accumulate
                fdiv = fdiv * 10.0f;                // update divisor
            }
            c = *pszline++;                         // next char
            uccounter--;                            // count characters
        }
        /* delimiter */
        if (( c != ',' ) && ( c != 0 )) {
            return TRUE;                            // error
        } else {
            ftemp = ftemp / fdiv;                   // convert
        }
        /* assign */
        switch ( ucfield++ ) {
            case 0: Waypoint[uiWpt_Number].Lon = ftemp; break;
            case 1: Waypoint[uiWpt_Number].Lat = ftemp; break;
            case 2: Waypoint[uiWpt_Number++].Alt = ftemp; break;
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
static void parse_coord( float * fcoord, uint8_t c )
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
            ulTemp_Coord = ulTemp_Coord * 10UL + (uint32_t)(c - '0');
            break;
        case '.' :
            *fcoord = (float)(ulTemp_Coord % 100UL) / 60.0f;  // decimal part
            *fcoord += (float)(ulTemp_Coord / 100UL);         // integer part
            ulTemp_Coord = 0UL;
            break;
        case ',' :
            *fcoord += (float)ulTemp_Coord / 600000.0f;       // decimal part
            ulTemp_Coord = 0UL;
            break;
        default :
            ulTemp_Coord = 0UL;
            break;
    }
}


//----------------------------------------------------------------------------
//
/// \brief   Parse GPS sentences
/// \param   -
/// \returns true if new coordinate data are available, false otherwise
/// \remarks structure of NMEA sentences
///              1
///  $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
///  hhmmss.ss = UTC of position fix
///  A         = Data status (V = navigation receiver warning)
///  llll.ll   = Latitude of fix
///  a         = N or S
///  yyyy.yy   = Longitude of fix
///  a         = E or W
///  x.x       = Speed over ground [kts]
///  x.x       = Track made good in [deg] True
///  ddmmyy    = UT date
///  x.x       = Magnetic variation [deg] (Easterly var. subtracts from true course)
///  a         = E or W
///  hh        = Checksum
///
///  $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
///  hhmmss.ss = UTC of Position
///  llll.ll   = Latitude
///  a         = N or S
///  yyyy.yy   = Longitude
///  a         = E or W
///  x         = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
///  xx        = Number of satellites in use [not those in view]
///  x.x       = Horizontal dilution of position
///  x.x       = Antenna altitude above/below mean sea level (geoid)
///  M         = Meters (Antenna height unit)
///  x.x       = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
///              mean sea level.-= geoid is below WGS-84 ellipsoid)
///  M         = Meters  (Units of geoidal separation)
///  x.x       = Age in seconds since last update from diff. reference station
///  xxxx      = Differential reference station ID#
///  hh        = Checksum
///
//----------------------------------------------------------------------------
static bool parse_gps( void )
{
    uint8_t c, j = 0;
    bool bcompleted = FALSE;                //!< true when NMEA sentence completed
    static uint8_t uccommas;                //!< counter of commas in NMEA sentence
    static ENUM_NMEA_TYPE enmea_type;

    while ((bcompleted == FALSE) &&             // NMEA sentence not completed
           (ucRindex != ucWindex)) {            // received another character

        c = ucGpsBuffer[ucRindex++];            // read character
        if (ucRindex >= BUFFER_LENGTH) {        // update read index
            ucRindex = 0;
        }

        if (c == '$') uccommas = 0;             // start of NMEA sentence
        if (c == ',') uccommas++;               // count commas

        switch (uccommas) {
            case 0:
                if (j < 6) {
                    szLine[j++] = c;            // read prefix
                } else {
                    szLine[j] = 0;              // terminate prefix
                }
                break;

            case 1:                             // check prefix
                if (cmp_prefix(szLine, "$GPRMC")) {
                    enmea_type = NMEA_GPRMC;
                } else if (cmp_prefix(szLine, "$GPGGA")) {
                    enmea_type = NMEA_GPGGA;
                } else {
                    enmea_type = NMEA_INVALID;
                    uccommas = 11;
                }
                j = 0;
                break;

            case 2:
                if (enmea_type == NMEA_GPRMC) { // get fix info
                    if (c == 'A') {
                     ucGps_Status = GPS_FIX;
                   } else if (c == 'V') {
                     ucGps_Status = GPS_NOFIX;
                   }
                }
                break;

            case 3:
            case 4:
                if (enmea_type == NMEA_GPRMC) { // get latitude data
                    parse_coord (&fTemp_Lat, c);
                }
                break;

            case 5:
            case 6:
                if (enmea_type == NMEA_GPRMC) { // get longitude data
                    parse_coord (&fTemp_Lon, c);
                }
                break;

            case 7:
                if (enmea_type == NMEA_GPRMC) { // get speed
                    if (c == ',') {
                        uiGps_Speed = 0;
                    } else if (c != '.') {
                        uiGps_Speed *= 10;
                        uiGps_Speed += (c - '0');
                    }
                }
                break;

            case 8:
                if (enmea_type == NMEA_GPRMC) { // get heading
                    if (c == ',') {
                        uiGps_Heading = 0;
                    } else if (c != '.') {
                        uiGps_Heading *= 10;
                        uiGps_Heading += (c - '0');
                    }
                }
                break;

            case 9:
                switch (enmea_type) {
                    case NMEA_GPRMC:            // end of GPRMC sentence
                        if (ucGps_Status == GPS_FIX) {
                            uiGps_Heading /= 10;
                            fCurr_Lat = fTemp_Lat;
                            fCurr_Lon = fTemp_Lon;
                            uccommas = 11;
                            bcompleted = TRUE;
                        }
                        break;

                    case NMEA_GPGGA:            // get altitude
                        if (c == ',') {
                          uiGps_Alt = 0;
                        } else if ( c != '.' ) {
                          uiGps_Alt *= 10;
                          uiGps_Alt += (c - '0');
                        }
                        break;
                }
                break;

            case 10:
                if (enmea_type == NMEA_GPGGA) { // end of GPGGA sentence
                    uiGps_Alt /= 10;
                    uccommas = 11;
                }
                break;
        }
    }
    return bcompleted;
}

//----------------------------------------------------------------------------
//
/// \brief   USART DMA interrupt handler
/// \param   -
/// \returns -
/// \remarks interrupt is triggered on both "half transfer" event and "transfer
///          completed" event, so it must check which one was the cause.
///          When the former occurs, write index is moved to half buffer size,
///          whereas when the latter occurs, write index is reset to 0.
///
//----------------------------------------------------------------------------
void DMA1_Channel6_IRQHandler( void ) {

    if (DMA_GetITStatus(DMA1_FLAG_TC6)) {    // transfer complete
        DMA_ClearITPendingBit(DMA1_IT_TC6);
        ucWindex = 0;
    } else {                                 // half transfer
        DMA_ClearITPendingBit(DMA1_IT_HT6);
        ucWindex = BUFFER_LENGTH / 2;
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Compare NMEA prefixes
/// \param   -
/// \returns
/// \remarks -
///
//----------------------------------------------------------------------------
static bool cmp_prefix( uint8_t * src , const uint8_t * dest ) {
    uint8_t j = 0;
    bool bmatch = TRUE;

    while ((j < 6) && (*src != 0) && bmatch) {
        bmatch = (*src++ == *dest++);
        j++;
    }
    return bmatch;
}

//----------------------------------------------------------------------------
//
/// \brief   Get total waypoint number
/// \param   -
/// \returns waypoint number
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Nav_Wpt_Number ( void ) {
  return uiWpt_Number;
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
  return uiWpt_Index;
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
  return (uint16_t)Waypoint[uiWpt_Index].Alt;
}

//----------------------------------------------------------------------------
//
/// \brief   Get waypoint data
/// \param   index = waypoint number
/// \returns waypoint data structure
/// \remarks -
///
//----------------------------------------------------------------------------
void Nav_Wpt_Get ( uint16_t index, STRUCT_WPT * wpt ) {
  if (index > uiWpt_Number) {
     index = 0;
  }
  wpt->Lat = Waypoint[index].Lat;
  wpt->Lon = Waypoint[index].Lon;
  wpt->Alt = Waypoint[index].Alt;
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
void Nav_Wpt_Set ( uint16_t index, STRUCT_WPT wpt ) {
}

//----------------------------------------------------------------------------
//
/// \brief   Get computed bearing [°]
/// \param   -
/// \returns bearing angle in degrees, between 0° and 360°
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Bearing_Deg ( void ) {
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
float Nav_Heading_Deg ( void ) {
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
float Nav_Bank_Rad ( void ) {
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
float Nav_Pitch_Rad ( void ) {
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
  return fCurr_Alt;
}

//----------------------------------------------------------------------------
//
/// \brief   Get gps status
/// \param   -
/// \returns 0 = no fix, 3 = 3d fix
/// \remarks -
///
//----------------------------------------------------------------------------
uint8_t Gps_Fix ( void ) {
  return ucGps_Status;
}

//----------------------------------------------------------------------------
//
/// \brief   Get ground speed detected by GPS [kt]
/// \param   -
/// \returns GS in knots
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Gps_Speed_Kt ( void ) {
  return uiGps_Speed;
}

//----------------------------------------------------------------------------
//
/// \brief   Get altitude detected by GPS [m]
/// \param   -
/// \returns Altitude in centimeters
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Gps_Alt_M ( void ) {
  return uiGps_Alt;
}

//----------------------------------------------------------------------------
//
/// \brief   Get GPS heading [°]
/// \param   -
/// \returns heading angle in degrees, between 0° and 360°
/// \remarks -
///
//----------------------------------------------------------------------------
uint16_t Gps_Heading_Deg ( void ) {
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
  return (int32_t)(fCurr_Lat * 10000000.0f);
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
  return (int32_t)(fCurr_Lon * 10000000.0f);
}

