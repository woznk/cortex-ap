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
//  CHANGES Telemetry_Init() called inside telemetry task,
//          added static prefix to all static functions
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
#include "aileronctrl.h"
#include "elevatorctrl.h"
#include "rudderctrl.h"
#include "throttlectrl.h"
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

enum E_TELEMETRY {
    TEL_NULL,
    TEL_GPS_POSITION = 0xf0,
    TEL_SERVO_POS,
    TEL_WAYPOINT,
    TEL_DEBUG_I,
    TEL_DEBUG_F,
} wait_code;

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

#if (TELEMETRY_DEBUG == 1)
const char s_pcSentence[] =
"$S,100,200,300,400,500,600,110\n$S,560,112,-12,12345,0,1023,110\n$S,512,512,512,512,512,512,512,110,110\n$S,512,512,512,\n$S,512,512,512,512,512,512\n$GPRMC,194617.04,A,4534.6714,N,01128.8559,E,000.0,287.0,091008,001.9,E,A*31\n$K,8799,1299, 899, 199,4999,4999\r\n$S,560,112,-12,12345,0,1023,110\n";
#endif

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL xQueueHandle xTelemetry_Queue;

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC unsigned char szString[48];
VAR_STATIC float pfGains[6];              /// gains
VAR_STATIC float pfSimSensorData[8];      /// Simulator sensor data
VAR_STATIC float pfSimSensorOffset[8];    /// Simulator sensor offsets
VAR_STATIC float fSimTAS = 0.0f;          /// Simulator TRUE air speed
VAR_STATIC float fSimCOG = 0.0f;          /// Simulator course over ground (GPS)
VAR_STATIC bool bSimSettled = FALSE;

//
// Used to change the polarity of the sensors
//
VAR_STATIC float pfSensorSign[8] = {
    1.0f, // 0, aircraft accel X
    1.0f, // 1, aircraft accel Y
    1.0f, // 2, aircraft accel Z
    1.0f, // 3, aircraft roll
    1.0f, // 4, aircraft pitch
    1.0f, // 5, aircraft yaw
    1.0f, // 6
    1.0f  // 7
};

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
/// \remarks -
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
///
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
/// \brief   sends data via USART 1
/// \remarks
///
///
///----------------------------------------------------------------------------
static void Telemetry_Send_Message(uint16_t *data, uint8_t num)
{
    long l_temp;
    uint8_t digit, i, j = 0;

    for (i = 0; i < num; i++) {
        l_temp = *data++;
        szString[j++] = ' ';
        digit = ((l_temp >> 12) & 0x0000000F);
        szString[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = ((l_temp >> 8) & 0x0000000F);
        szString[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = ((l_temp >> 4) & 0x0000000F);
        szString[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = (l_temp & 0x0000000F);
        szString[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
    }
    szString[j++] = '\n';

    for (j = 0; j < (i * 5) + 1; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, szString[j]);
    }
}

///----------------------------------------------------------------------------
///
///  DESCRIPTION Outputs DCM matrix
/// \RETURN      -
/// \REMARKS
///
///
///----------------------------------------------------------------------------
static void Telemetry_Send_DCM(void) {

    uint8_t x, y, j = 0;
    unsigned char ucDigit;
    long lTemp;

    for (y = 0; y < 3; y++) {
      for (x = 0; x < 3; x++) {
          lTemp = (long)ceil(DCM_Matrix[y][x] * 32767.0f);
          szString[j++] = ' ';
          ucDigit = ((lTemp >> 12) & 0x0000000F);
          szString[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
          ucDigit = ((lTemp >> 8) & 0x0000000F);
          szString[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
          ucDigit = ((lTemp >> 4) & 0x0000000F);
          szString[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
          ucDigit = (lTemp & 0x0000000F);
          szString[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
      }
    }
    szString[j++] = '\n';
    szString[j] = '\r';
    for (j = 0; j < 47; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, szString[j]);
    }
}

//----------------------------------------------------------------------------
//
/// \brief   parse telemetry data
///
/// \returns TRUE when new GPS data are available
/// \remarks
///
///
//----------------------------------------------------------------------------
static bool Telemetry_Parse ( void )
{
    char c;
    bool bResult;
    static float fTemp = 0.0f;
    static unsigned char ucStatus = 0;
    static unsigned long ulChCounter = 1L, ulTemp = 0L;

    bResult = FALSE;
    while (FALSE/*Telemetry_Get_Char(&c)*/) {

        if (ulChCounter != 0L) {                    // Count chars
            ulChCounter++;
        }
        if ((ucStatus > 9) && (ucStatus < 19)) {    // Log GPS data
            Log_PutChar(c);
        }

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
                    pfSimSensorData[ucStatus - 3] = fTemp;
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
                    fSimTAS = fTemp / 100.0f;
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
            case 11:
                if (c == ',') { ucStatus++; }
                break;
            case 12:
                if (c == 'A') {
                    Gps_Status |= GPS_STATUS_FIX;
                } else if (c == 'V') {
                    Gps_Status &= ~GPS_STATUS_FIX;
                } else if (c == ',') {
                    ucStatus++;
                }
                break;
            case 13:
                if ( c == ',' ) {
                    fCurrLat = (ulTemp % 1000000L) / 600000.0f; // convert ' to �
                    fCurrLat += (float)(ulTemp / 1000000L);     // add �
                    ulTemp = 0L;
                    ucStatus++;
                } else if ( c != '.' ) {
                    ulTemp = ulTemp * 10L + (unsigned long)(c - '0');
                }
                break;
            case 14:    // TO DO : change sign of Lat based on N / S
                if (c == ',') { ucStatus++; }
                break;
            case 15:
                if ( c == ',' ) {
                    fCurrLon = (ulTemp % 1000000L) / 600000.0f; // convert ' to �
                    fCurrLon += (float)(ulTemp / 1000000L);     // add �
                    ulTemp = 0L;
                    ucStatus++;
                } else if ( c != '.' ) {
                    ulTemp = ulTemp * 10L + (unsigned long)(c - '0');
                }
                break;
            case 16:    // TO DO : change sign of Lon based on E / W
                if (c == ',') { ucStatus++; }
                break;
            case 17:    //
                if ( c == ',' ) {
                    fSimCOG = fTemp / 10.0f;
                    fTemp = 0.0f;
                    ucStatus++;
                } else if ( c != '.' ) {
                    fTemp = (fTemp * 10.0f) + (float)(c - '0');
                }
                break;
            case 18:
                if ( c == ',' ) {
                    Heading = (int)(ulTemp / 10L);
//                    North = 360 - Heading;
                    ulTemp = 0L;
                    ucStatus = 0;
                    bResult = TRUE;
                    Log_PutChar('\r'); // Terminate GPS sentence
                } else if ( c != '.' ) {
                    ulTemp = (ulTemp * 10) + (unsigned long)(c - '0');
                }
                break;

            case 19:            // ------------------ gains ------------------
                if (c == ',') { ucStatus++; } else { ucStatus = 0; }
                break;
            case 20:
            case 21:
            case 22:
            case 23:
            case 24:
            case 25:
                if ((c == '\r') || (c == '\n')) {
                    pfGains[ucStatus - 20] = fTemp;
                    fTemp = 0.0f;
                    ucStatus = 26;
                } else if (c == ',') {
                    pfGains[ucStatus - 20] = fTemp;
                    fTemp = 0.0f;
                    ucStatus++;
                } else if ((c >= '0') && (c <= '9')) {
                    fTemp = fTemp * 10.0f + (float)(c - '0');
                } else if (c != ' ') {
                    fTemp = 0.0f;
                    ucStatus = 0;
                }
                break;
            case 26:
                ucStatus = 0;
                Gyro_Gain    = pfGains[0] / 9999.0f;
                Accel_Gain   = pfGains[1] / 9999.0f;
                PitchRoll_Kp = pfGains[2] / 9999000.0f;
                PitchRoll_Ki = pfGains[3] / 99990000.0f;
                Yaw_Kp       = pfGains[4] / 9999.0f;
                Yaw_Ki       = pfGains[5] / 9999000.0f;
                break;

            default:
                fTemp = 0.0f;
                ulTemp = 0L;
                ucStatus = 0;
                break;
        }
    }
    if (ulChCounter > 1000) {
        Sim_SaveOffsets();
        ulChCounter = 0;
        bSimSettled = TRUE;
    }

    return bResult;
}


//----------------------------------------------------------------------------
//
/// \brief   Downlink controls
///
/// \returns
/// \remarks
///
///
//----------------------------------------------------------------------------
static void Telemetry_Send_Controls(void)
{
    float *pfBuff;
    unsigned char cData[20];

    cData[0] = TEL_SERVO_POS;       // wait code

    pfBuff = (float *)&cData[1];    // elevator
//    *pfBuff = Elevator();

    pfBuff = (float *)&cData[5];    // ailerons
//    *pfBuff = Ailerons();

    pfBuff = (float *)&cData[9];    // rudder
//    *pfBuff = Rudder();

    pfBuff = (float *)&cData[13];   // throttle
//    *pfBuff = Throttle();

//    UART0Send(cData, 17);
}

//----------------------------------------------------------------------------
//
/// \brief   Downlink waypoint
///
/// \returns
/// \remarks
///
///
//----------------------------------------------------------------------------
static void Telemetry_Send_Waypoint(void)
{
    unsigned int *puiBuff;
    unsigned char cData[8];

    cData[0] = TEL_WAYPOINT;                // wait code

    cData[1] = Nav_WaypointIndex();         // waypoint index

    puiBuff = (unsigned int *)&cData[2];    // bearing
    *puiBuff = (unsigned int)Nav_Bearing();

    puiBuff = (unsigned int *)&cData[4];    // altitude
    *puiBuff = 0;

    puiBuff = (unsigned int *)&cData[6];    // distance
    *puiBuff = Nav_Distance();

//    UART0Send(cData, 8);
}

///----------------------------------------------------------------------------
///
/// \brief Interface to simulator data : data settled
/// \return
/// \remarks
///
///----------------------------------------------------------------------------
bool Sim_Settled ( void ) {
    return bSimSettled;
}
#endif

///----------------------------------------------------------------------------
///
/// \brief Interface to simulator data : output.
/// \return
/// \remarks
///
///----------------------------------------------------------------------------
float Sim_GetData(int n) {

    float fData;
    switch (n) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
            fData = (pfSimSensorData[n] - pfSimSensorOffset[n]) * pfSensorSign[n];
            break;
        default:
            fData = 0.0f;
            break;
    }
    return fData;
}

///----------------------------------------------------------------------------
///
/// \brief Interface to simulator data : TRUE air speed
/// \return
/// \remarks
///
///----------------------------------------------------------------------------
float Sim_Speed(void) {
//    return fSimTAS;
    return fSimCOG;
}

///----------------------------------------------------------------------------
///
/// \brief Interface to simulator data : input.
/// \return
/// \remarks Used to force simulator data from extern
///
///----------------------------------------------------------------------------
void Sim_SetData(int iIndex, float fVal) {

    switch (iIndex) {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
            pfSimSensorData[iIndex] = fVal;
            break;
        default:
            break;
    }
}

///----------------------------------------------------------------------------
///
/// \brief Interface to simulator data : offset.
/// \return
/// \remarks Used to save offsets
///
///----------------------------------------------------------------------------
void Sim_SaveOffsets(void) {

    int j;
    for (j = 0; j < 6 ; j++) {
        pfSimSensorOffset[j] = pfSimSensorData[j];
    }
    pfSimSensorOffset[2] -= GRAVITY;
}


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

