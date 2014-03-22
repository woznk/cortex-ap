/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief GPS driver
 *
 * @file
 *
 * Change:
 *
 *============================================================================*/

#include "ch.h"
#include "hal.h"

#include "config.h"
#include "gps.h"

/*--------------------------------- Definitions ------------------------------*/

#define LINE_LENGTH 16      /* length of lines read */

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/* NMEA string types */
typedef enum {
    NMEA_GPRMC,     /* recommended minimum specific GPS/transit data */
    NMEA_GPGGA,     /* global positioning system fix data */
    NMEA_INVALID    /* invalid NMEA string */
} ENUM_NMEA_TYPE;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

static void parse_coord( float * f_coord, uint8_t c );
static bool cmp_prefix( const uint8_t * src , const uint8_t * dest );

/*----------------------------------- Locals ---------------------------------*/

static float f_temp_lat = 0.0f;             /* temporary latitude */
static float f_temp_lon = 0.0f;             /* temporary longitude */
static float f_curr_lat = 0.0f;             /* current latitude */
static float f_curr_lon = 0.0f;             /* current longitude */

static uint8_t sz_line[LINE_LENGTH];        /* input line */

static uint32_t ul_temp_coord = 0UL;        /* temporary for coordinate parser */
static uint16_t ui_gps_cog = 0;             /* aircraft course over ground [°] */
static uint16_t ui_gps_speed = 0;           /* speed [kt/10] */
static uint16_t ui_gps_alt = 0;             /* altitude [m] */
static uint8_t uc_gps_status = GPS_NOFIX;   /* status of GPS */
static uint8_t uc_commas = 11;              /* counter of commas in NMEA sentence */
static uint8_t prefix_index = 0;

static ENUM_NMEA_TYPE e_nmea_type;

static Semaphore sem_gps;

static SerialConfig sdcfg = {
  57600
};

/*---------------------------------- Functions -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   Initialize UART for GPS
 * @param   -
 * @return  -
 * @remarks
 *
 *---------------------------------------------------------------------------*/
void Init_GPS ( void ) {

  /* Activates the Serial driver 2, PA2(TX) and PA3(RX) are routed to USART2. */
  palSetPadMode(GPIOA, 2, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  sdStart(&SD2, &sdcfg);
  chSemInit(&sem_gps, 1);   /* Semaphore initialization */
}

/*----------------------------------------------------------------------------
 *
 * @brief     Parse NMEA sentence for coordinates
 * @param[in] f_coord : pointer to coordinate
 * @param[in] c : character of NMEA sentence
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
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
            ul_temp_coord = ul_temp_coord * 10UL + (uint32_t)(c - '0');
            break;
        case '.' :
            *f_coord = (float)(ul_temp_coord % 100UL) / 60.0f;  /* decimal part */
            *f_coord += (float)(ul_temp_coord / 100UL);         /* integer part */
            ul_temp_coord = 0UL;
            break;
        case ',' :
            *f_coord += (float)ul_temp_coord / 6000000.0f;      /* decimal part */
            ul_temp_coord = 0UL;
            break;
        default :
            ul_temp_coord = 0UL;
            break;
    }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Parse GPS sentences
 * @param   -
 * @returns true if new coordinate data are available, false otherwise
 * @remarks structure of NMEA sentences
 *              1
 *  $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
 *  hhmmss.ss = UTC of position fix
 *  A         = Data status (V = navigation receiver warning)
 *  llll.ll   = Latitude of fix
 *  a         = N or S
 *  yyyy.yy   = Longitude of fix
 *  a         = E or W
 *  x.x       = Speed over ground [kts]
 *  x.x       = Track made good in [deg] True
 *  ddmmyy    = UT date
 *  x.x       = Magnetic variation [deg] (Easterly var. subtracts from true course)
 *  a         = E or W
 *  hh        = Checksum
 *
 *  $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 *  hhmmss.ss = UTC of Position
 *  llll.ll   = Latitude
 *  a         = N or S
 *  yyyy.yy   = Longitude
 *  a         = E or W
 *  x         = GPS quality indicator (0=invalid; 1=GPS fix; 2=Diff. GPS fix)
 *  xx        = Number of satellites in use [not those in view]
 *  x.x       = Horizontal dilution of position
 *  x.x       = Antenna altitude above/below mean sea level (geoid)
 *  M         = Meters (Antenna height unit)
 *  x.x       = Geoidal separation (Diff. between WGS-84 earth ellipsoid and
 *              mean sea level.-= geoid is below WGS-84 ellipsoid)
 *  M         = Meters (Units of geoidal separation)
 *  x.x       = Age in seconds since last update from diff. reference station
 *  xxxx      = Differential reference station ID#
 *  hh        = Checksum
 *
 *---------------------------------------------------------------------------*/
void Parse_GPS( void ) {

  msg_t c;

  do {

    c = chnGetTimeout(&SD2, TIME_IMMEDIATE);

    if (c != Q_TIMEOUT) {
//      chSemWait(&sem_gps);
      if (c == '$') uc_commas = 0;              /* start of NMEA sentence */
      if (c == ',') uc_commas++;                /* count commas */

      switch (uc_commas) {
        case 0:
            if (prefix_index < 6) {
                sz_line[prefix_index++] = c;    /* read prefix */
            } else {
                sz_line[prefix_index] = 0;      /* terminate prefix */
            }
            break;

        case 1:                                 /* check prefix */
            if (cmp_prefix(sz_line, (const uint8_t *)"$GPRMC")) {
                e_nmea_type = NMEA_GPRMC;
            } else if (cmp_prefix(sz_line, (const uint8_t *)"$GPGGA")) {
                e_nmea_type = NMEA_GPGGA;
            } else {
                e_nmea_type = NMEA_INVALID;
                uc_commas = 11;
            }
            prefix_index = 0;
            break;

        case 2:
            if (e_nmea_type == NMEA_GPRMC) {    /* get fix info */
                if (c == 'A') {
                   uc_gps_status = GPS_FIX;
                } else if (c == 'V') {
                   uc_gps_status = GPS_NOFIX;
                }
            }
            break;

        case 3:
        case 4:
            if (e_nmea_type == NMEA_GPRMC) {    /* get latitude data */
                parse_coord (&f_temp_lat, c);
            }
            break;

        case 5:
        case 6:
            if (e_nmea_type == NMEA_GPRMC) {    /* get longitude data */
                parse_coord (&f_temp_lon, c);
            }
            break;

        case 7:
            if (e_nmea_type == NMEA_GPRMC) {    /* get speed */
                if (c == ',') {
                    ui_gps_speed = 0;
                } else if (c != '.') {
                    ui_gps_speed *= 10;
                    ui_gps_speed += (c - '0');
                }
            }
            break;

        case 8:
            if (e_nmea_type == NMEA_GPRMC) {    /* get heading */
                if (c == ',') {
                    ui_gps_cog = 0;
                } else if (c != '.') {
                    ui_gps_cog *= 10;
                    ui_gps_cog += (c - '0');
                }
            }
            break;

        case 9:
            switch (e_nmea_type) {
                case NMEA_GPRMC:                /* end of GPRMC sentence */
                    if (uc_gps_status == GPS_FIX) {
                        ui_gps_cog /= 10;
                        f_curr_lat = f_temp_lat;
                        f_curr_lon = f_temp_lon;
                        uc_commas = 11;
                    }
                    break;

                case NMEA_GPGGA:                /* get altitude */
                    if (c == ',') {
                      ui_gps_alt = 0;
                    } else if (c != '.') {
                      ui_gps_alt *= 10;
                      ui_gps_alt += (c - '0');
                    }
                    break;

                case NMEA_INVALID:              /* invalid */
                    break;

                default:                        /* error */
                    break;
            }
            break;

        case 10:
            if (e_nmea_type == NMEA_GPGGA) {    /* end of GPGGA sentence */
                ui_gps_alt /= 10;
                uc_commas = 11;
            }
            break;

        default:
            break;
      }                     /* end switch */
    }                       /* end if */
//    chSemSignal(&sem_gps);
  } while (c != Q_TIMEOUT); /* end do */
}


/*----------------------------------------------------------------------------
 *
 * @brief   Compare NMEA prefixes
 * @param   -
 * @returns
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
static bool cmp_prefix( const uint8_t * src , const uint8_t * dest ) {
    uint8_t j = 0;
    bool b_match = TRUE;

    while ((j < 6) && (*src != 0) && b_match) {
        b_match = (*src++ == *dest++);
        j++;
    }
    return b_match;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get gps status
 * @param   -
 * @returns 0 = no fix, 3 = 3d fix
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
uint8_t Gps_Status ( void ) {
  uint8_t uc_temp;

//  chSemWait(&sem_gps);
  uc_temp = uc_gps_status;
//  chSemSignal(&sem_gps);

  return uc_temp;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get ground speed detected by GPS [kt]
 * @param   -
 * @returns GS in knots
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
uint16_t Gps_Speed_Kt ( void ) {
  uint16_t ui_temp;

//  chSemWait(&sem_gps);
  ui_temp = ui_gps_speed;
//  chSemSignal(&sem_gps);

  return ui_temp;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get altitude detected by GPS [m]
 * @param   -
 * @returns Altitude in centimeters
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
uint16_t Gps_Alt_M ( void ) {
  uint16_t ui_temp;

//  chSemWait(&sem_gps);
  ui_temp = ui_gps_alt;
//  chSemSignal(&sem_gps);

  return ui_temp;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get GPS course over ground [°]
 * @param   -
 * @returns COG angle in degrees, between 0° and 360°
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
uint16_t Gps_COG_Deg ( void ) {
  uint16_t ui_temp;

//  chSemWait(&sem_gps);
  ui_temp = ui_gps_cog;
//  chSemSignal(&sem_gps);

  return ui_temp;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get latitude [°]
 * @param   -
 * @returns latitude angle degrees
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
float Gps_Latitude ( void ) {
  float f_temp;

//  chSemWait(&sem_gps);
  f_temp = f_curr_lat;
//  chSemSignal(&sem_gps);

  return f_temp;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get longitude [°]
 * @param   -
 * @returns longitude angle degrees
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
float Gps_Longitude ( void ) {
  float f_temp;

//  chSemWait(&sem_gps);
  f_temp = f_curr_lon;
//  chSemSignal(&sem_gps);

  return f_temp;
}

