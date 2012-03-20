//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief Simulator interface
///
/// \file
/// Sistemi di riferimento:                                             \code
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
/// \todo aggiungere parser protocollo ardupilot o mnav
///
//  CHANGES szString[] renamed ucBuffer[] and used also for downlink of servo 
//          positions and waypoint data.
//          Added actual UART transmission to Telemetry_Send_Waypoints() and
//          Telemetry_Send_Servo() functions
//
//============================================================================*/

// ---- Include Files -------------------------------------------------------


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "math.h"
#include "gps.h"
#include "nav.h"
#include "DCM.h"
#include "log.h"
#include "config.h"
#include "servodriver.h"
#include "telemetry.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#   undef VAR_STATIC
#endif
#define VAR_STATIC static

#ifdef VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#define TELEMETRY_DEBUG 0

#if (TELEMETRY_DEBUG == 0)
#   define Telemetry_Get_Char UART0GetChar
#else
#   define Telemetry_Get_Char Debug_GetChar
#endif

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

typedef enum E_TELEMETRY {
    TEL_NULL,
    TEL_GPS_POSITION = 0xf0,
    TEL_SERVO_POS,
    TEL_WAYPOINT,
    TEL_DEBUG_I,
    TEL_DEBUG_F,
} wait_code_t;

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

#if (TELEMETRY_DEBUG == 1)
const char s_pcSentence[] =
"$S,100,200,300,400,500,600,110\n$S,560,112,-12,12345,0,1023,110\n$S,512,512,512,512,512,512,512,110,110\n$S,512,512,512,\n$S,512,512,512,512,512,512\n$GPRMC,194617.04,A,4534.6714,N,01128.8559,E,000.0,287.0,091008,001.9,E,A*31\n$K,8799,1299, 899, 199,4999,4999\r\n$S,560,112,-12,12345,0,1023,110\n";
#endif

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL xQueueHandle xTelemetry_Queue;

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC unsigned char ucBuffer[48];  /// downlink data buffer
VAR_STATIC float fGain[6];              /// gains for PID loops
VAR_STATIC float fSensor[8];            /// simulator sensor data
VAR_STATIC float fTrueAirSpeed = 0.0f;  /// simulator true air speed

/*--------------------------------- Prototypes -------------------------------*/

#if (TELEMETRY_DEBUG == 1)
bool Debug_GetChar ( char *ch );
#endif
static void Telemetry_Init( void );
static void Telemetry_Send_Message(uint16_t *data, uint8_t num);
static void Telemetry_Send_DCM( void );
static bool Telemetry_Parse( void );
static void Telemetry_Send_Controls( void );
static void Telemetry_Send_Waypoint( void );

/*---------------------------------- Functions -------------------------------*/

#ifndef _WINDOWS

///----------------------------------------------------------------------------
///
/// \brief  telemetry task
/// \return  -
/// \remarks waits for a message to be added to telemetry queue and sends it
///          to the UART
///
///----------------------------------------------------------------------------
void Telemetry_Task( void *pvParameters )
{
    xTelemetry_Message message;

    Telemetry_Init();       // Telemetry initialization

    while (1) {
        while (xQueueReceive( xTelemetry_Queue, &message, portMAX_DELAY ) != pdPASS) {
        }
        Telemetry_Send_Message(message.pcData, message.ucLength);
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Initialize telemetry
/// \return  -
/// \remarks configures USART1.
///          See http://www.micromouseonline.com/2009/12/31/stm32-usart-basics/#ixzz1eG1EE8bT
///          for direct register initialization of USART 1
///
//----------------------------------------------------------------------------
static void Telemetry_Init( void ) {

    USART_InitTypeDef USART_InitStructure;

    // Initialize USART1 structure
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    // Configure USART1
    USART_Init(USART1, &USART_InitStructure);

    // Enable USART1 interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //USART_ITConfig(USART1,USART_IT_TXE,ENABLE);

    // Enable the USART1
    USART_Cmd(USART1, ENABLE);
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
static void Telemetry_Send_Message(uint16_t *data, uint8_t num)
{
    long l_temp;
    uint8_t digit, i, j = 0;

    for (i = 0; i < num; i++) {
        l_temp = *data++;
        ucBuffer[j++] = ' ';
        digit = ((l_temp >> 12) & 0x0000000F);
        ucBuffer[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = ((l_temp >> 8) & 0x0000000F);
        ucBuffer[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = ((l_temp >> 4) & 0x0000000F);
        ucBuffer[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = (l_temp & 0x0000000F);
        ucBuffer[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
    }
    ucBuffer[j++] = '\n';

    for (j = 0; j < (i * 5) + 1; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, ucBuffer[j]);
    }
}

///----------------------------------------------------------------------------
///
/// \brief   sends DCM matrix via telemetry USART
/// \return  -
/// \remarks entries of DCM matrix are converted to hexadecimal string and sent
///          to UART
///
///----------------------------------------------------------------------------
static void Telemetry_Send_DCM(void) {

    uint8_t x, y, j = 0;
    unsigned char ucDigit;
    long lTemp;

    for (y = 0; y < 3; y++) {
      for (x = 0; x < 3; x++) {
          lTemp = (long)ceil(DCM_Matrix[y][x] * 32767.0f);
          ucBuffer[j++] = ' ';
          ucDigit = ((lTemp >> 12) & 0x0000000F);
          ucBuffer[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
          ucDigit = ((lTemp >> 8) & 0x0000000F);
          ucBuffer[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
          ucDigit = ((lTemp >> 4) & 0x0000000F);
          ucBuffer[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
          ucDigit = (lTemp & 0x0000000F);
          ucBuffer[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
      }
    }
    ucBuffer[j++] = '\n';
    ucBuffer[j] = '\r';
    for (j = 0; j < 47; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, ucBuffer[j]);
    }
}

//----------------------------------------------------------------------------
//
/// \brief   parse telemetry data
/// \returns TRUE when new GPS data are available
/// \remarks telemetry is used to upload different information during flight
///          and during simulation:
///
///          INFORMATION  SIMULATION  FLIGHT  NOTE
///          ---------------------------------------------------------------
///          GPS data         X         -     during flight, GPS data comes
///                                           from actual GPS receiver
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
///             $G     GPS data          see NMEA    NMEA sentence
///             $S     gyro,accel,speed     7        16 bit decimal integers
///             $K     PID gains            6        16 bit decimal integers
///
///         GPS data is forwarded to navigation task, forcing characters in
///         the GPS UART buffer, as they had been received from actual GPS.
///         Parsing of sensor data and PID gains is a simple string to float
///         conversion. Sensor data and gains have no final checksum.
///
//----------------------------------------------------------------------------
static bool Telemetry_Parse ( void )
{
    char c;
    bool bResult;
    static float fTemp = 0.0f;
    static unsigned char ucStatus = 0;

    bResult = FALSE;
    while (FALSE/*Telemetry_Get_Char(&c)*/) {

        switch (ucStatus) {
            case 0:             // ----------------- preamble -----------------
                if (c == '$') { ucStatus++; }
                break;
            case 1:
                switch (c) {
                    case 'S': ucStatus++; break;    // Sensor data
                    case 'G': ucStatus = 10; break; // GPS data
                    case 'K': ucStatus = 19; break; // Gain data
                    default : ucStatus = 0; break;  //
                }
                break;
            case 2:             // ------------------ sensors ------------------
                if (c == ',') { ucStatus++; } else { ucStatus = 0; }
                break;
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:
            case 8:
                if (c == ',') {
                    fSensor[ucStatus - 3] = fTemp;
                    fTemp = 0.0f;
                    ucStatus++;
                } else if ((c >= '0') && (c <= '9')) {
                    fTemp = fTemp * 10.0f + (float)(c - '0');
                } else if (c != ' ') {
                    fTemp = 0.0f;
                    ucStatus = 0;
                }
                break;
            case 9:
                if (c == '\r') {
                    fTrueAirSpeed = fTemp / 100.0f;
                    fTemp = 0.0f;
                    ucStatus = 0;
                } else if ((c >= '0') && (c <= '9')) {
                    fTemp = fTemp * 10.0f + (float)(c - '0');
                } else if (c != ' ') {
                    fTemp = 0.0f;
                    ucStatus = 0;
                }
                break;

            case 10:            // ------------------- GPS --------------------
                Nav_Gps_Putc('$');      // force initial characters into GPS buffer
                Nav_Gps_Putc('G');
                ucStatus++;
            case 11:
                if (c == '$') {         // preamble received
                   ucStatus = 1;        // go check preamble type
                } else {                // still inside NMEA sentence
                   Nav_Gps_Putc(c);     // force character into GPS buffer
                }
                break;

            case 12:            // --------------- PID gains ------------------
                if (c == ',') { ucStatus++; } else { ucStatus = 0; }
                break;
            case 13:
            case 14:
            case 15:
            case 16:
            case 17:
            case 18:
                if ((c == '\r') || (c == '\n')) {
                    fGain[ucStatus - 20] = fTemp;
                    fTemp = 0.0f;
                    ucStatus = 26;
                } else if (c == ',') {
                    fGain[ucStatus - 20] = fTemp;
                    fTemp = 0.0f;
                    ucStatus++;
                } else if ((c >= '0') && (c <= '9')) {
                    fTemp = fTemp * 10.0f + (float)(c - '0');
                } else if (c != ' ') {
                    fTemp = 0.0f;
                    ucStatus = 0;
                }
                break;
            case 19:
                ucStatus = 0;
                Gyro_Gain    = fGain[0] / 9999.0f;
                Accel_Gain   = fGain[1] / 9999.0f;
                PitchRoll_Kp = fGain[2] / 9999000.0f;
                PitchRoll_Ki = fGain[3] / 99990000.0f;
                Yaw_Kp       = fGain[4] / 9999.0f;
                Yaw_Ki       = fGain[5] / 9999000.0f;
                break;

            default:
                fTemp = 0.0f;
                ucStatus = 0;
                break;
        }
    }
    return bResult;
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
static void Telemetry_Send_Controls(void)
{
    uint8_t j;
    float *pfBuff;

    ucBuffer[0] = TEL_SERVO_POS;                // telemetry wait code

    pfBuff = (float *)&ucBuffer[1];             // elevator
    *pfBuff = (float)Servo_Get(SERVO_ELEVATOR);

    pfBuff = (float *)&ucBuffer[5];             // ailerons
    *pfBuff = (float)Servo_Get(SERVO_AILERON);

    pfBuff = (float *)&ucBuffer[9];             // rudder
    *pfBuff = (float)Servo_Get(SERVO_RUDDER);

    pfBuff = (float *)&ucBuffer[13];            // throttle
    *pfBuff = (float)Servo_Get(SERVO_THROTTLE);

    for (j = 0; j < 17; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, ucBuffer[j]);
    }
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
///               2     bearing LSB
///               3     bearing MSB
///               4     altitude LSB
///               5     altitude MSB
///               6     distance LSB
///               7     distance MSB
///
//----------------------------------------------------------------------------
static void Telemetry_Send_Waypoint(void)
{
    uint8_t j;
    uint16_t *puiBuff;

    ucBuffer[0] = TEL_WAYPOINT;             // telemetry wait code

    ucBuffer[1] = Nav_WaypointIndex();      // waypoint index

    puiBuff = (uint16_t *)&ucBuffer[2];     // bearing to waypoint
    *puiBuff = (uint16_t)Nav_Bearing();

    puiBuff = (uint16_t *)&ucBuffer[4];     // waypoint altitude
    *puiBuff = (uint16_t)Nav_Altitude();

    puiBuff = (uint16_t *)&ucBuffer[6];     // distance to waypoint
    *puiBuff = Nav_Distance();

    for (j = 0; j < 8; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, ucBuffer[j]);
    }
}

///----------------------------------------------------------------------------
///
/// \brief   interface to simulator data : true air speed
/// \return  true air speed
/// \remarks -
///
///----------------------------------------------------------------------------
float Sim_Speed(void) {
    return fTrueAirSpeed;
}
#endif


#if (TELEMETRY_DEBUG == 1)
//----------------------------------------------------------------------------
//
/// \brief Get a character from preloaded string
///
/// \returns
/// \remarks
///
//----------------------------------------------------------------------------
bool Debug_GetChar ( char *ch )
{
    char c;
    VAR_STATIC unsigned char j = 0;

    c = s_pcSentence[j];
    if (c == 0) { j = 0; } else { j++; }
    *ch = c;
    return TRUE;
}
#endif

