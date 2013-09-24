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
/// Change: corrected parsing of coordinates according higher precision of
///         uBlox GPS: 5 decimal digits instead of 4.
//
//============================================================================*/

#include "FreeRTOS.h"

#include "stm32f10x_usart.h"
#include "stm32f10x_dma.h"
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
#include "log.h"
#include "globals.h"
#include "nav.h"

/*--------------------------------- Definitions ------------------------------*/

#ifndef   VAR_STATIC
#   define VAR_STATIC static
#endif

#define USART2_DR_Base  0x40004404

#define MAX_WAYPOINTS   8       //!< maximum number of waypoints
#define MIN_DISTANCE    100     //!< minimum distance from waypoint [m]

#define LINE_LENGTH     48      //!< length of lines read from file

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/
/*
/// navigation modes
typedef enum {
    NAV_RTL,        //!< return to launch
    NAV_WPT         //!< waypoint following
} ENUM_NAV_MODE;
*/
/// NMEA string types
typedef enum {
    NMEA_GPRMC,     //!< recommended minimum specific GPS/transit data
    NMEA_GPGGA,     //!< global positioning system fix data
    NMEA_INVALID    //!< invalid NMEA string
} ENUM_NMEA_TYPE;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC uint8_t uc_Gps_Buffer[BUFFER_LENGTH];        //!< gps data buffer
VAR_STATIC STRUCT_WPT Waypoint[MAX_WAYPOINTS];         	//!< waypoints array
VAR_STATIC const uint8_t sz_File[16] = "path.txt";      //!< file name
VAR_STATIC uint8_t sz_Line[LINE_LENGTH];                //!< input line
VAR_STATIC UINT w_File_Bytes;                           //!< counter of read bytes
VAR_STATIC float f_Dir_Error;                           //!< direction error [rad]
VAR_STATIC float f_Alt_Error;                           //!< altitude error [m]
VAR_STATIC float f_Pitch;                               //!< pitch setpoint [rad]
VAR_STATIC float f_Dest_Lat;                            //!< destination latitude
VAR_STATIC float f_Dest_Lon;                            //!< destination longitude
VAR_STATIC float f_Dest_Alt;                            //!< destination altitude
VAR_STATIC float f_Curr_Lat;                            //!< current latitude
VAR_STATIC float f_Curr_Lon;                            //!< current longitude
VAR_STATIC float f_Curr_Alt;                            //!< current altitude [m]
VAR_STATIC float f_Temp_Lon;                            //!< temporary longitude during parse
VAR_STATIC float f_Temp_Lat;                            //!< temporary latitude during parse
VAR_STATIC float f_Bearing;                             //!< angle to destination [°]
VAR_STATIC float f_Heading;                             //!< aircraft navigation heading [°]
VAR_STATIC uint32_t ul_Temp_Coord;                      //!< temporary for coordinate parser
VAR_STATIC uint16_t ui_Distance;                        //!< distance to destination [m]
VAR_STATIC uint16_t ui_Wpt_Number;                      //!< total number of waypoints
VAR_STATIC uint16_t ui_Wpt_Index;                       //!< waypoint index
VAR_STATIC uint16_t ui_Gps_Heading;                     //!< aircraft GPS heading [°]
VAR_STATIC uint16_t ui_Gps_Speed;                       //!< speed [kt/10]
VAR_STATIC uint16_t ui_Gps_Alt;                         //!< altitude [m]
VAR_STATIC uint8_t uc_Gps_Status;                       //!< status of GPS
VAR_STATIC uint8_t uc_Windex;                           //!< USART buffer write index
VAR_STATIC uint8_t uc_Rindex;                           //!< USART buffer read index

/*--------------------------------- Prototypes -------------------------------*/

static void load_path( void );
static void gps_init( void );
static bool parse_waypoint ( const uint8_t * psz_line );
static void parse_coord( float * fCoord, uint8_t c );
static bool parse_gps( void );
static bool cmp_prefix( const uint8_t * src , const uint8_t * dest );

//----------------------------------------------------------------------------
//
/// \brief   navigation task
///
/// \remarks Computation must be started only when parse_gps() returns TRUE,
///          otherwise values of lat, lon, heading are undefined.
///
//----------------------------------------------------------------------------
void Navigation_Task( void *pvParameters ) {

    float f_temp, f_dx, f_dy;

    (void)pvParameters;

    f_Dir_Error = 0.0f;                                 // default direction error
    f_Alt_Error = 0.0f;                                 // default altitude error
    f_Bearing = 0.0f;                                   // angle to destination [°]
    f_Pitch = 0.0f;                                     // default pitch angle
    uc_Gps_Status = GPS_NOFIX;                          // init GPS status
    ui_Gps_Heading = 0;                                 // aircraft GPS heading [°]
    ui_Gps_Speed = 0;                                   // aircraft GPS speed [kt]
    ui_Distance = 0;                                    // distance to destination [m]
    ui_Wpt_Number = 0;                                  // no waypoint yet

    load_path();                                        // load path from SD card
    /* WARNING: GPS UART must be initialized after loading mission file !!! */
    gps_init();                                         // initialize USART for GPS

    /* Wait first GPS fix */
    while ((parse_gps() == FALSE) ||                    // NMEA sentence not completed
           (uc_Gps_Status != GPS_FIX)) {                // no satellite fix
        ;                                               // keep parsing NMEA sentences
    }

    /* Save launch position */
    Waypoint[0].Lon = f_Curr_Lon;                       // launch position as first waypoint
    Waypoint[0].Lat = f_Curr_Lat;
    if (ui_Wpt_Number != 0) {                           // waypoint file available
        ui_Wpt_Index = 1;                               // read first waypoint
    } else {                                            // no waypoint file
        ui_Wpt_Index = 0;                               // use launch position
    }
    f_Dest_Lon = Waypoint[ui_Wpt_Index].Lon;            // load destination longitude
    f_Dest_Lat = Waypoint[ui_Wpt_Index].Lat;            // load destination latitude
    f_Dest_Alt = Waypoint[ui_Wpt_Index].Alt;            // load destination altitude

    for (;;) {
        if (parse_gps()) {                                  // NMEA sentence completed
#if (SIMULATOR == SIM_NONE)                                 // normal mode
            f_Curr_Alt = (float)BMP085_Get_Altitude();    // get barometric altitude
//            f_Curr_Alt = (float)ui_Gps_Alt;                 // get GPS altitude
#else                                                       // simulation mode
            f_Curr_Alt = Simulator_Get_Altitude();          // get simulator altitude
#endif
            /* Get X and Y components of bearing */
            f_dy = f_Dest_Lon - f_Curr_Lon;
            f_dx = f_Dest_Lat - f_Curr_Lat;

            /* Compute heading */
            f_Heading = Attitude_Yaw_Rad() / PI;

            /* Compute bearing to waypoint */
            f_Bearing = atan2f(f_dy, f_dx) / PI;

            /* Compute direction error */
            f_temp = f_Heading - f_Bearing;
            if (f_temp < -1.0f) {
                f_temp += 2.0f;
            } else if (f_temp > 1.0f) {
                f_temp = 2.0f - f_temp;
            }
            f_Dir_Error = f_temp;

            /* Compute distance to waypoint */
            f_temp = sqrtf((f_dy * f_dy) + (f_dx * f_dx));
            ui_Distance = (uint16_t)(f_temp * 111113.7f);

            /* Waypoint reached: next waypoint */
            if (ui_Distance < MIN_DISTANCE) {
                if (ui_Wpt_Number != 0) {
                    if (++ui_Wpt_Index == ui_Wpt_Number) {
                        ui_Wpt_Index = 1;
                    }
                }
                f_Dest_Lon = Waypoint[ui_Wpt_Index].Lon;    // new destination longitude
                f_Dest_Lat = Waypoint[ui_Wpt_Index].Lat;    // new destination latitude
                f_Dest_Alt = Waypoint[ui_Wpt_Index].Alt;    // new destination altitude
            }
            f_temp = f_Curr_Alt - f_Dest_Alt;               // altitude error
            f_Alt_Error = f_temp;

            /* Compute throttle and pitch based on altitude error */
/*
            f_temp = f_Curr_Alt - f_Dest_Alt;               // altitude error
            if (f_temp > f_Height_Margin) {                 // above maximum
                f_Throttle = f_Throttle_Min;                // minimum  throttle
                f_Pitch = PITCHATMINTHROTTLE;
            } else if (f_temp < - f_Height_Margin) {        // below minimum
                f_Throttle = f_Throttle_Max;                // max throttle
                f_Pitch = PITCHATMAXTHROTTLE;
            } else {                                        // interpolate
                f_temp = (f_temp - HEIGHT_MARGIN) / (2.0f * HEIGHT_MARGIN);
                f_Throttle = f_temp * (f_Throttle_Min - f_Throttle_Max) + f_Throttle_Min;
                f_Pitch = f_temp * (PITCHATMINTHROTTLE - PITCHATMAXTHROTTLE) + PITCHATMINTHROTTLE;
            }
*/
        }
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Load path from file on SD card
/// \param   -
/// \return  -
/// \remarks uc_Gps_Buffer[] array is used for file reading.
///          USART 2 must be disabled because it uses same array for reception.
///
//----------------------------------------------------------------------------
static void load_path( void ) {

    uint8_t c, uc_counter;
    uint8_t * p_buffer_pointer;
    uint8_t * psz_line_pointer;
    bool b_error = TRUE;

    psz_line_pointer = sz_Line;                     // init line pointer
    uc_counter = LINE_LENGTH - 1;                   // init char counter

    /* Mount file system and open waypoint file */
    if (!b_FS_Ok) {                               // file system not mounted
        ui_Wpt_Number = 0;                          // no waypoint available
    } else if (FR_OK != f_open(&st_File, (const XCHAR *)sz_File,FA_READ)) {
                                                    // error opening file
        ui_Wpt_Number = 0;                          // no waypoint available
    } else {                                        // file system ok and
        b_error = FALSE;                            // file succesfully open
        ui_Wpt_Number = 1;                          // waypoint available
    }

    /* Read waypoint file */
    while ((!b_error) &&
           (FR_OK == f_read(&st_File, uc_Gps_Buffer, BUFFER_LENGTH, &w_File_Bytes))) {
        b_error = (w_File_Bytes == 0);              // force error if end of file
        p_buffer_pointer = uc_Gps_Buffer;           // init buffer pointer
        while ((w_File_Bytes != 0) && (!b_error)) { // buffer not empty and no error
            w_File_Bytes--;                         // decrease overall counter
            c = *p_buffer_pointer++;                // read another char
            if ( c == 10 ) {                        // found new line
                uc_counter = 0;                     // reached end of line
            } else if ( c == 13 ) {                 // found carriage return
                uc_counter--;                       // decrease counter
            } else {                                // alphanumeric character
                *psz_line_pointer++ = c;            // copy character
                uc_counter--;                       // decrease counter
            }
            if (uc_counter == 0) {                  // end of line
                *psz_line_pointer = 0;              // append line delimiter
                psz_line_pointer = sz_Line;         // reset line pointer
                uc_counter = LINE_LENGTH - 1;       // reset char counter
                b_error = parse_waypoint(sz_Line);  // parse line for waypoint
            }
        }
    }
    ( void )f_close( &st_File );                     // close file
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

    uc_Windex = 0;                                   // clear write index
    uc_Rindex = 0;                                   // clear read index
    ul_Temp_Coord = 0UL;                             // clear temporary coordinate

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
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)uc_Gps_Buffer;
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
static bool parse_waypoint ( const uint8_t * psz_line ) {

    uint8_t c, uc_field = 0, uc_counter = LINE_LENGTH;
    float f_temp, fdiv;

    while (( uc_field < 3 ) && ( uc_counter > 0 )) {
        fdiv = 1.0f;                                    // initialize divisor
        f_temp = 0.0f;                                  // initialize temporary
        c = *psz_line++;                                // initialize char
        /* leading spaces */
        while (( c == ' ' ) && ( uc_counter > 0 )) {
            c = *psz_line++;                            // next char
            uc_counter--;                               // count characters
        }
        /* start of integer part */
        if (( c < '0' ) || ( c > '9' )) {               //
            return TRUE;                                // first char not numeric
        }
        /* integer part */
        while (( c >= '0' ) && ( c <= '9' ) && ( uc_counter > 0 )) {
            f_temp = f_temp * 10.0f + (float)(c - '0'); // accumulate
            c = *psz_line++;                            // next char
            uc_counter--;                               // count characters
        }
        /* decimal point */
        if (( c != '.' ) && ( uc_field != 2 )) {        // altitude may lack decimal
            return TRUE;
        } else {
            c = *psz_line++;                            // skip decimal point
            uc_counter--;                               // count characters
        }
        /* fractional part */
        while (( c >= '0' ) && ( c <= '9' ) && ( uc_counter > 0 )) {
            if (fdiv < 1000000.0f) {
                f_temp = f_temp * 10.0f + (float)(c - '0'); // accumulate
                fdiv = fdiv * 10.0f;                    // update divisor
            }
            c = *psz_line++;                            // next char
            uc_counter--;                               // count characters
        }
        /* delimiter */
        if (( c != ',' ) && ( c != 0 )) {
            return TRUE;                                // error
        } else {
            f_temp = f_temp / fdiv;                     // convert
        }
        /* assign */
        switch ( uc_field++ ) {
            case 0: Waypoint[ui_Wpt_Number].Lon = f_temp; break;
            case 1: Waypoint[ui_Wpt_Number].Lat = f_temp; break;
            case 2: Waypoint[ui_Wpt_Number++].Alt = f_temp; break;
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
static void parse_coord( float * f_coord, uint8_t c )
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
            ul_Temp_Coord = ul_Temp_Coord * 10UL + (uint32_t)(c - '0');
            break;
        case '.' :
            *f_coord = (float)(ul_Temp_Coord % 100UL) / 60.0f;  // decimal part
            *f_coord += (float)(ul_Temp_Coord / 100UL);         // integer part
            ul_Temp_Coord = 0UL;
            break;
        case ',' :
            *f_coord += (float)ul_Temp_Coord / 6000000.0f;      // decimal part
            ul_Temp_Coord = 0UL;
            break;
        default :
            ul_Temp_Coord = 0UL;
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
    bool b_completed = FALSE;           //!< true when NMEA sentence completed
    static uint8_t uc_commas;           //!< counter of commas in NMEA sentence
    static ENUM_NMEA_TYPE e_nmea_type;

    while (!b_completed &&                      // NMEA sentence not completed
           (uc_Rindex != uc_Windex)) {          // received another character

        c = uc_Gps_Buffer[uc_Rindex++];         // read character

        if (uc_Rindex >= BUFFER_LENGTH) {       // update read index
            uc_Rindex = 0;
        }

        if (c == '$') uc_commas = 0;            // start of NMEA sentence
        if (c == ',') uc_commas++;              // count commas

        switch (uc_commas) {
            case 0:
                if (j < 6) {
                    sz_Line[j++] = c;           // read prefix
                } else {
                    sz_Line[j] = 0;             // terminate prefix
                }
                break;

            case 1:                             // check prefix
                if (cmp_prefix(sz_Line, (const uint8_t *)"$GPRMC")) {
                    e_nmea_type = NMEA_GPRMC;
                } else if (cmp_prefix(sz_Line, (const uint8_t *)"$GPGGA")) {
                    e_nmea_type = NMEA_GPGGA;
                } else {
                    e_nmea_type = NMEA_INVALID;
                    uc_commas = 11;
                }
                j = 0;
                break;

            case 2:
                if (e_nmea_type == NMEA_GPRMC) { // get fix info
                    if (c == 'A') {
                       uc_Gps_Status = GPS_FIX;
                    } else if (c == 'V') {
                       uc_Gps_Status = GPS_NOFIX;
                    }
                }
                break;

            case 3:
            case 4:
                if (e_nmea_type == NMEA_GPRMC) { // get latitude data
                    parse_coord (&f_Temp_Lat, c);
                }
                break;

            case 5:
            case 6:
                if (e_nmea_type == NMEA_GPRMC) { // get longitude data
                    parse_coord (&f_Temp_Lon, c);
                }
                break;

            case 7:
                if (e_nmea_type == NMEA_GPRMC) { // get speed
                    if (c == ',') {
                        ui_Gps_Speed = 0;
                    } else if (c != '.') {
                        ui_Gps_Speed *= 10;
                        ui_Gps_Speed += (c - '0');
                    }
                }
                break;

            case 8:
                if (e_nmea_type == NMEA_GPRMC) { // get heading
                    if (c == ',') {
                        ui_Gps_Heading = 0;
                    } else if (c != '.') {
                        ui_Gps_Heading *= 10;
                        ui_Gps_Heading += (c - '0');
                    }
                }
                break;

            case 9:
                switch (e_nmea_type) {
                    case NMEA_GPRMC:            // end of GPRMC sentence
                        if (uc_Gps_Status == GPS_FIX) {
                            ui_Gps_Heading /= 10;
                            f_Curr_Lat = f_Temp_Lat;
                            f_Curr_Lon = f_Temp_Lon;
                            uc_commas = 11;
                            b_completed = TRUE;
                        }
                        break;

                    case NMEA_GPGGA:            // get altitude
                        if (c == ',') {
                          ui_Gps_Alt = 0;
                        } else if ( c != '.' ) {
                          ui_Gps_Alt *= 10;
                          ui_Gps_Alt += (c - '0');
                        }
                        break;

                    case NMEA_INVALID:          // invalid
                        break;

                    default:                    // error
                        break;
                }
                break;

            case 10:
                if (e_nmea_type == NMEA_GPGGA) { // end of GPGGA sentence
                    ui_Gps_Alt /= 10;
                    uc_commas = 11;
                }
                break;

            default:
                break;
        }
    }
    return b_completed;
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
        uc_Windex = 0;
    } else {                                 // half transfer
        DMA_ClearITPendingBit(DMA1_IT_HT6);
        uc_Windex = BUFFER_LENGTH / 2;
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
static bool cmp_prefix( const uint8_t * src , const uint8_t * dest ) {
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
  return ui_Wpt_Number;
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
  return ui_Wpt_Index;
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
  return (uint16_t)Waypoint[ui_Wpt_Index].Alt;
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
  if (index > ui_Wpt_Number) {
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
    (void) index;
    (void) wpt;
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
  if (f_Bearing < 0.0f)  {
    return 360.0f + (f_Bearing * 180.0f);
  } else {
    return (f_Bearing * 180.0f);
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
  if (f_Heading < 0.0f)  {
    return 360.0f + (f_Heading * 180.0f);
  } else {
    return (f_Heading * 180.0f);
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
  return ui_Distance;
}

//----------------------------------------------------------------------------
//
/// \brief   Get bank angle [rad]
/// \param   -
/// \returns bank angle
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Dir_Error ( void ) {
  return f_Dir_Error;
}

//----------------------------------------------------------------------------
//
/// \brief   Get throttle
/// \param   -
/// \returns throttle
/// \remarks -
///
//----------------------------------------------------------------------------
float Nav_Alt_Error ( void ) {
  return f_Alt_Error;
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
  return f_Pitch;
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
  return f_Curr_Alt;
}

//----------------------------------------------------------------------------
//
/// \brief   Get gps buffer index
/// \param   -
/// \returns gps buffer write index
/// \remarks -
///
//----------------------------------------------------------------------------
uint8_t Gps_Buffer_Index ( void ) {
  return uc_Windex;
}

//----------------------------------------------------------------------------
//
/// \brief   Get gps buffer pointer
/// \param   -
/// \returns gps buffer pointer
/// \remarks -
///
//----------------------------------------------------------------------------
uint8_t * Gps_Buffer_Pointer ( void ) {
  if (uc_Windex == 0) {
    return (uint8_t *)&uc_Gps_Buffer[BUFFER_LENGTH / 2];
  } else {
    return uc_Gps_Buffer;
  }
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
  return uc_Gps_Status;
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
  return ui_Gps_Speed;
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
  return ui_Gps_Alt;
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
  return ui_Gps_Heading;
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
  return (int32_t)(f_Curr_Lat * 10000000.0f);
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
  return (int32_t)(f_Curr_Lon * 10000000.0f);
}

