//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief telemetry interface
///
/// \file
/// Reference systems :                                             \code
///                                   __|__
///                                  /  |  \
///                                 |___|___|
///                                    | |
///                                    / \
///                                   |   |
///                                   |   |
///                      _____________|___|_____________
///                     /                               \
///                    |                                 |
///                    |_________________________________|
///                                   |   |
///                                    \_/
///                                  ---+---
///
///         X-Plane         |      Flightgear        |     UAV DevBoard
/// ------------------------+------------------------+-------------------------
///    PITCH+               |    PITCH+              |               PITCH+
///       - >     -         |       - >     -        | YAW+  -         - >
///     /      Z+   \       |     /      Z+   \      |     /     Z+  /
/// <--| ----(X)     |      | <--| ----(.)     |     |    |    (X)--| ---> X+
/// Y+  \     |     /       | Y+  \     |     /      |     \    |    \
///       -   |  <- YAW+    |       -   |  <- YAW+   |       - >|      -
///           |   ^         |           |   ^        |          |   ^
///       \   |   /         |       \   |   /        |      \   |   /
///         -___-   ROLL+   |         -___-   ROLL+  |        -___-   ROLL+
///           |             |           |            |          |
///           |             |           |            |          |
///           V X+          |           V X+         |          V Y+
///                         |                        |
/// ------------------------+------------------------+-------------------------
///                                                                     \endcode
///
//  Change: Nav_Bearing / Nav_Heading renamed Nav_Bearing_Deg / Nav_Heading_Deg
//
//
//============================================================================*/

// ---- Include Files -------------------------------------------------------

#include "misc.h"

#include "config.h"
#include "nav.h"
#include "DCM.h"
#include "servodriver.h"
#include "usart1driver.h"
#include "telemetry.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#   undef VAR_STATIC
#endif
#define   VAR_STATIC static

#ifdef    VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/// telemetry message types
typedef enum E_TELEMETRY {
    TEL_NULL,
    TEL_GPS_POSITION = 0xF0,
    TEL_SERVO_POS,
    TEL_WAYPOINT,
    TEL_ACCEL,
    TEL_GYRO,
    TEL_DEBUG_I,
    TEL_DEBUG_F,
    TEL_DCM
} wait_code_t;

/// telemetry parser stati
typedef enum E_PARSER {
    PARSE_PREAMBLE = 0, // preamble
    PARSE_TYPE,         // 1: data type ('S'= sensor, 'K'= gains)
    PARSE_SENSORS,      // 2: sensor start
    PARSE_SENSOR_1,     // 3: sensor 1
    PARSE_SENSOR_2,     // 4: sensor 2
    PARSE_SENSOR_3,     // 5: sensor 3
    PARSE_SENSOR_4,     // 6: sensor 4
    PARSE_SENSOR_5,     // 7: sensor 5
    PARSE_SENSOR_6,     // 8: sensor 6
    PARSE_AIRSPEED,     // 9: airspeed
    PARSE_ALTITUDE,     // 10: altitude
    PARSE_GAINS,        // 11: PID gains start
    PARSE_GAIN_1,       // 12: gain 1
    PARSE_GAIN_2,       // 13: gain 2
    PARSE_GAIN_3,       // 14: gain 3
    PARSE_GAIN_4,       // 15: gain 4
    PARSE_GAIN_5,       // 16: gain 5
    PARSE_GAIN_6        // 17: gain 6
} parser_status_t;

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

//VAR_GLOBAL xQueueHandle xTelemetry_Queue;

/*----------------------------------- Locals ---------------------------------*/
/*
"$S,100,200,300,400,500,600,110\n"
"$S,560,112,-12,12345,0,1023,110\n"
"$S,512,512,512,512,512,512,512,110,110\n"
"$S,512,512,512,\n$S,512,512,512,512,512,512\n"
"$GPRMC,194617.04,A,4534.6714,N,01128.8559,E,000.0,287.0,091008,001.9,E,A*31\n"
"$K,8799,1299, 899, 199,4999,4999\r\n"
"$S,560,112,-12,12345,0,1023,110\n"
*/
VAR_STATIC parser_status_t xStatus = PARSE_PREAMBLE;//!< status of parser
VAR_STATIC float fTemp = 0.0f;                      //!< temporary for parser
VAR_STATIC float fTrueAirSpeed = 0.0f;              //!< simulator true air speed
VAR_STATIC float fAltitude;                         //!< simulator altitude
VAR_STATIC float fSensor[8];                        //!< simulator sensor data
VAR_STATIC float fGain[TEL_GAIN_NUMBER] = {
    PITCH_KP,                                       //!< default pitch kp
    PITCH_KI,                                       //!< default pitch ki
    ROLL_KP,                                        //!< default roll kp
    ROLL_KI,                                        //!< default roll ki
    NAV_KP,                                         //!< default direction kp
    NAV_KI,                                         //!< default direction ki
    0.0f,                                           //!< dummy placeholder
    0.0f                                            //!< dummy placeholder
};                                                  //!< gains for PID loops

/*--------------------------------- Prototypes -------------------------------*/

/*---------------------------------- Functions -------------------------------*/

//----------------------------------------------------------------------------
//
/// \brief   parse telemetry data
/// \returns -
/// \remarks telemetry is used to upload different information during flight
///          and during simulation:
///
///          INFORMATION  SIMULATION  FLIGHT  NOTE
///          ---------------------------------------------------------------
///          sensor data      X         -     during flight, sensor data
///                                           comes from actual gyroscopes,
///                                           accelerometers and pressure
///          PID gains        X         X     PID gains are input on ground
///                                           control station
///
///          Telemetry messages start with a preamble (2 characters) used to
///          discriminate the type of message. The preamble format has been
///          chosen to resemble the GPS NMEA sentences.
///          Preamble is followed by useful data. Number of data fields and
///          data field format depends on message type:
///
///          PREAMBLE  MESSAGE CONTENT  DATA FIELDS DATA FORMAT
///          -------------------------------------------------------------
///             $S     gyro,accel,speed     7        16 bit decimal integers
///             $K     PID gains            6        16 bit decimal integers
///
///         Parsing of sensor data and PID gains is a simple string to float
///         conversion. Sensor data and gains have no final checksum.
///
//----------------------------------------------------------------------------
void Telemetry_Parse ( void )
{
    uint8_t c;
    while (USART1_Getch(&c)) {          // received another character
        switch (xStatus) {
            case PARSE_PREAMBLE :   /* ------------- preamble -------------- */
                if (c == '$') { xStatus++; }    // preamble ok
                break;
            case PARSE_TYPE :       /* ------------- data type ------------- */
                switch (c) {
                    case 'S': xStatus = PARSE_SENSORS; break;   // sensor data
                    case 'K': xStatus = PARSE_GAINS; break;     // gain data
                    default : xStatus = PARSE_PREAMBLE; break;  // error
                }
                break;
            case PARSE_SENSORS :    /* -------------- sensors -------------- */
                if (c == ',') { xStatus++; } else { xStatus = PARSE_PREAMBLE; }
                break;
            case PARSE_SENSOR_1 :
            case PARSE_SENSOR_2 :
            case PARSE_SENSOR_3 :
            case PARSE_SENSOR_4 :
            case PARSE_SENSOR_5 :
            case PARSE_SENSOR_6 :
                if (c == ',') {
                    fSensor[xStatus - 3] = 32767.0f - fTemp;    // save sensor
                    fTemp = 0.0f;                               // clear temp
                    xStatus++;                                  // next sensor
                } else if ((c >= '0') && (c <= '9')) {          // numeric char
                    fTemp = fTemp * 10.0f + (float)(c - '0');   // add to temp
                } else if (c != ' ') {                          // error
                    fTemp = 0.0f;                               // clear temp
                    xStatus = PARSE_PREAMBLE;                   // reset parser
                }
                break;
            case PARSE_AIRSPEED :
                if (c == ',') {
                    fTrueAirSpeed = fTemp / 100.0f;             // save airspeed
                    fTemp = 0.0f;                               // clear temp
                    xStatus++;                                  // next data
                } else if ((c >= '0') && (c <= '9')) {          // numeric char
                    fTemp = fTemp * 10.0f + (float)(c - '0');   // add to temp
                } else if (c != ' ') {                          // error
                    fTemp = 0.0f;                               // clear temp
                    xStatus = PARSE_PREAMBLE;                   // reset parser
                } else {
				}
                break;
            case PARSE_ALTITUDE :
                if (c == '\r') {                                // end of data
                    fAltitude = fTemp;                          // save altitude
                    fTemp = 0.0f;                               // clear temp
                    xStatus = PARSE_PREAMBLE;                   // reset parser
                } else if ((c >= '0') && (c <= '9')) {          // numeric char
                    fTemp = fTemp * 10.0f + (float)(c - '0');   // add to temp
                } else if (c != ' ') {                          // error
                    fTemp = 0.0f;                               // clear temp
                    xStatus = PARSE_PREAMBLE;                   // reset parser
                } else {
				}
                break;

            case PARSE_GAINS :  /* --------------- PID gains ------------------ */
                if (c == ',') { xStatus++; } else { xStatus = PARSE_PREAMBLE; }
                break;
            case PARSE_GAIN_1 :
            case PARSE_GAIN_2 :
            case PARSE_GAIN_3 :
            case PARSE_GAIN_4 :
            case PARSE_GAIN_5 :
            case PARSE_GAIN_6 :
                if ((c == '\r') || (c == '\n')) {                   // end of data
                    fGain[xStatus - PARSE_GAIN_1] = fTemp / 2000.0f;// save last gain
                    fTemp = 0.0f;                                   // clear temp
                    xStatus = PARSE_PREAMBLE;                       // reset parser
                } else if (c == ',') {                              // comma
                    fGain[xStatus - PARSE_GAIN_1] = fTemp / 2000.0f;// save gain
                    fTemp = 0.0f;                                   // clear temp
                    xStatus++;                                      // next gain
                } else if ((c >= '0') && (c <= '9')) {              // numeric char
                    fTemp = fTemp * 10.0f + (float)(c - '0');       // add to temp
                } else if (c != ' ') {                              // error
                    fTemp = 0.0f;                                   // clear temp
                    xStatus = PARSE_PREAMBLE;                       // reset parser
                } else {
				}
                break;

            default:                                            // wrong status
                fTemp = 0.0f;                                   // clear temp
                xStatus = PARSE_PREAMBLE;                       // reset parser
                break;
        }
    }
}


//----------------------------------------------------------------------------
//
/// \brief   Downlink controls
/// \returns -
/// \remarks buffer content:
///
///             index   content
///
///               0     TEL_SERVO_POS
///               1     elevator
///               2         "
///               3         "
///               4         "
///               5     ailerons
///               6         "
///               7         "
///               8         "
///               9     rudder
///              10         "
///              11         "
///              12         "
///              13     throttle
///              14         "
///              15         "
///              16         "
///
//----------------------------------------------------------------------------
void Telemetry_Send_Controls(void)
{
    USART1_Putch(TEL_SERVO_POS);                    // telemetry wait code
    USART1_Putf((float)Servo_Get(SERVO_ELEVATOR));  // elevator
    USART1_Putf((float)Servo_Get(SERVO_AILERON));   // ailerons
    USART1_Putf((float)Servo_Get(SERVO_RUDDER));    // rudder
    USART1_Putf((float)Servo_Get(SERVO_THROTTLE));  // throttle

    USART1_Transmit();                              // send data
}

//----------------------------------------------------------------------------
//
/// \brief   Downlink waypoint data
/// \returns -
/// \remarks buffer content:
///
///             index   content
///
///               0     TEL_WAYPOINT
///               1     waypoint index
///               2     bearing
///               3         "
///               4     altitude
///               5         "
///               6     distance
///               7         "
///
//----------------------------------------------------------------------------
void Telemetry_Send_Waypoint(void)
{
    USART1_Putch(TEL_WAYPOINT);                 // telemetry wait code
    USART1_Putch(Nav_Wpt_Index());              // waypoint index
    USART1_Putw((uint16_t)Nav_Bearing_Deg());       // bearing to waypoint
    USART1_Putw((uint16_t)Nav_Wpt_Altitude());  // waypoint altitude
    USART1_Putw(Nav_Distance());                // distance to waypoint

    USART1_Transmit();                          // send data
}

//----------------------------------------------------------------------------
//
/// \brief   Downlink GPS position
/// \returns -
/// \remarks buffer content:
///
///             index   content
///
///               0     TEL_GPS_POSITION
///               1     latitude
///               2         "
///               3         "
///               4         "
///               5     longitude
///               6         "
///               7         "
///               8         "
///               9     altitude
///              10         "
///              10     heading
///              10         "
///
//----------------------------------------------------------------------------
void Telemetry_Send_Position(void)
{
    USART1_Putch(TEL_GPS_POSITION);                     // telemetry wait code
    USART1_Putf((float)Gps_Latitude() / 10000000.0f);   // current latitude
    USART1_Putf((float)Gps_Longitude() / 10000000.0f);  // current longitude
    USART1_Putf(Nav_Altitude());                        // current altitude
    USART1_Putf(Nav_Heading_Deg());                         // heading

    USART1_Transmit();                                  // send data
}

///----------------------------------------------------------------------------
///
/// \brief   sends data via telemetry USART
/// \return  -
/// \param   data, pointer to message
/// \param   num number of elements
/// \remarks message is an array of 'num' words, each word is 16 bit.
///          Each element is converted to an hexadecimal string and sent to UART
///
///----------------------------------------------------------------------------
void Telemetry_Send_Message(uint16_t *data, uint8_t num)
{
    uint32_t l_temp;
    uint8_t digit, i = 0;

    for (i = 0; i < num; i++) {
        l_temp = *data++;
        USART1_Putch(' ');
        digit = ((l_temp >> 12) & 0x0000000F);
        USART1_Putch(((digit < 10) ? (digit + '0') : ((digit - 10) + 'A')));
        digit = ((l_temp >> 8) & 0x0000000F);
        USART1_Putch(((digit < 10) ? (digit + '0') : ((digit - 10) + 'A')));
        digit = ((l_temp >> 4) & 0x0000000F);
        USART1_Putch(((digit < 10) ? (digit + '0') : ((digit - 10) + 'A')));
        digit = (l_temp & 0x0000000F);
        USART1_Putch(((digit < 10) ? (digit + '0') : ((digit - 10) + 'A')));
    }
    USART1_Putch('\n');

    USART1_Transmit();                      // send data
}

///----------------------------------------------------------------------------
///
/// \brief   sends DCM matrix via telemetry USART
/// \return  -
/// \remarks entries of DCM matrix are sent as raw bytes
///
///----------------------------------------------------------------------------
void Telemetry_Send_DCM(void) {

    uint8_t x, y;

    USART1_Putch(TEL_DCM);                  // telemetry wait code
    for (y = 0; y < 3; y++) {               // 3 rows
      for (x = 0; x < 3; x++) {             // 3 columns
          USART1_Putf(DCM_Matrix[y][x]);    // DCM entry
      }
    }
    USART1_Transmit();                      // send data
}

///----------------------------------------------------------------------------
///
/// \brief   interface to simulator data : true air speed
/// \return  true air speed
/// \remarks -
///
///----------------------------------------------------------------------------
float Telemetry_Get_Speed(void) {
    return fTrueAirSpeed;
}

///----------------------------------------------------------------------------
///
/// \brief   interface to simulator data : altitude
/// \return  altitude
/// \remarks -
///
///----------------------------------------------------------------------------
float Telemetry_Get_Altitude(void) {
    return fAltitude;
}

///----------------------------------------------------------------------------
///
/// \brief   Get PID gain
/// \param   gain = gain number
/// \return  gain value
/// \remarks -
///
///----------------------------------------------------------------------------
float Telemetry_Get_Gain(telEnum_Gain gain) {
    if (gain < TEL_GAIN_NUMBER) {
        return fGain[gain];
    } else {
        return 0.0f;
    }
}

///----------------------------------------------------------------------------
///
/// \brief   Get sensor value
/// \param   piSensor = pointer to sensor data array
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void Telemetry_Get_Raw_IMU(int16_t * piSensor) {
    uint8_t j;

    for (j = 0; j < 6; j++) {
        *piSensor++ = (int16_t)fSensor[j];
    }
}

