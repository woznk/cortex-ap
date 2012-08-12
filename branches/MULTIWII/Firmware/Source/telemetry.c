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
/// \todo add mavlink protocol, see:
///  http://www.qgroundcontrol.org/mavlink/start
///  http://www.qgroundcontrol.org/dev/mavlink_onboard_integration_tutorial
///
//  CHANGES removed unneeded #inclusions, DCM update rate to 4 Hz
//
//============================================================================*/

// ---- Include Files -------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "misc.h"

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "math.h"
#include "nav.h"
#include "DCM.h"
#include "log.h"
#include "config.h"
#include "servodriver.h"
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

#define TELEMETRY_FREQUENCY 50
#define TELEMETRY_DELAY     (configTICK_RATE_HZ / TELEMETRY_FREQUENCY)
#define RX_BUFFER_LENGTH    48
#define TX_BUFFER_LENGTH    48
#define TEL_DCM_LENGTH      (1 + (9 * 4))

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

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
VAR_STATIC uint8_t ucRxBuffer[RX_BUFFER_LENGTH];    // uplink data buffer
VAR_STATIC uint8_t ucTxBuffer[TX_BUFFER_LENGTH];    // downlink data buffer
VAR_STATIC uint8_t ucWindex;                        // uplink write index
VAR_STATIC uint8_t ucRindex;                        // uplink read index
VAR_STATIC parser_status_t xStatus;                 // status of parser
VAR_STATIC float fTemp;                             // temporary for parser
VAR_STATIC float fTrueAirSpeed;                     // simulator true air speed
VAR_STATIC float fAltitude;                         // simulator altitude
VAR_STATIC float fSensor[8];                        // simulator sensor data
VAR_STATIC float fGain[TEL_GAIN_NUMBER] = {         // gains for PID loops
    PITCH_KP,                                       // default pitch kp
    PITCH_KI,                                       // default pitch ki
    ROLL_KP,                                        // default roll kp
    ROLL_KI,                                        // default roll ki
    NAV_KP,                                         // default direction kp
    NAV_KI,                                         // default direction ki
    0.0f,                                           // dummy placeholder
    0.0f                                            // dummy placeholder
};

/*--------------------------------- Prototypes -------------------------------*/

static void Telemetry_Init( void );
static void Telemetry_Send_Message(uint16_t *data, uint8_t num);
static void Telemetry_Send_DCM( void );
static void Telemetry_Parse( void );
static void Telemetry_Send_Waypoint( void );

/*---------------------------------- Functions -------------------------------*/

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
    uint8_t ucCycles = 0;
    portTickType Last_Wake_Time;

    Last_Wake_Time = xTaskGetTickCount();       //
    Telemetry_Init();                           // telemetry initialization

    while (TRUE) {
        vTaskDelayUntil(&Last_Wake_Time, TELEMETRY_DELAY);
#if 1
        Telemetry_Send_Controls();              // update simulator controls
#endif
        Telemetry_Parse();                      // parse uplink data
        if (++ucCycles >= (TELEMETRY_FREQUENCY / 4)) {// every .125 second
            ucCycles = 0;                       // reset cycle counter
            Telemetry_Send_Waypoint();          // send waypoint information
#if 1
            Telemetry_Send_DCM();
#endif
        }
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
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Initialize USART1 structure */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);       // configure USART1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // enable USART1 interrupt
    //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

    /* Configure NVIC for USART1 interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    USART_Cmd(USART1, ENABLE);                      // enable the USART1

    ucWindex = 0;                                   // clear write index
    ucRindex = 0;                                   // clear read index
    xStatus = PARSE_PREAMBLE;                       // reset parser status
    fTemp = 0.0f;                                   // clear parser temporary
    fTrueAirSpeed = 0.0f;                           // clear true air speed
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
    uint32_t l_temp;
    uint8_t digit, i, j = 0;

    for (i = 0; i < num; i++) {
        l_temp = *data++;
        ucTxBuffer[j++] = ' ';
        digit = ((l_temp >> 12) & 0x0000000F);
        ucTxBuffer[j++] = ((digit < 10) ? (digit + '0') : ((digit - 10) + 'A'));
        digit = ((l_temp >> 8) & 0x0000000F);
        ucTxBuffer[j++] = ((digit < 10) ? (digit + '0') : ((digit - 10) + 'A'));
        digit = ((l_temp >> 4) & 0x0000000F);
        ucTxBuffer[j++] = ((digit < 10) ? (digit + '0') : ((digit - 10) + 'A'));
        digit = (l_temp & 0x0000000F);
        ucTxBuffer[j++] = ((digit < 10) ? (digit + '0') : ((digit - 10) + 'A'));
    }
    ucTxBuffer[j++] = '\n';

    for (j = 0; j < (i * 5) + 1; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, ucTxBuffer[j]);
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

    uint8_t x, y, j;
    float * pfTemp;

    ucTxBuffer[0] = TEL_DCM;
    pfTemp = (float *)&(ucTxBuffer[1]);
    for (y = 0; y < 3; y++) {
      for (x = 0; x < 3; x++) {
          *pfTemp++ = DCM_Matrix[y][x];
      }
    }
    for (j = 0; j < TEL_DCM_LENGTH; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, ucTxBuffer[j]);
    }
}

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
static void Telemetry_Parse ( void )
{
    uint8_t c;
    while (ucRindex != ucWindex) {          // received another character
        c = ucRxBuffer[ucRindex++];         // read character

        if (ucRindex >= RX_BUFFER_LENGTH) { // update read index
            ucRindex = 0;
        }

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
    uint8_t j;
    float *pfBuff;

    ucTxBuffer[0] = TEL_SERVO_POS;                  // telemetry wait code

    pfBuff = (float *)&ucTxBuffer[1];               // starting of data field

    *pfBuff++ = (float)Servo_Get(SERVO_ELEVATOR);   // elevator
    *pfBuff++ = (float)Servo_Get(SERVO_AILERON);    // ailerons
    *pfBuff++ = (float)Servo_Get(SERVO_RUDDER);     // rudder
    *pfBuff   = (float)Servo_Get(SERVO_THROTTLE);   // throttle

    for (j = 0; j < 17; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, ucTxBuffer[j]);
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

    ucTxBuffer[0] = TEL_WAYPOINT;               // telemetry wait code

    ucTxBuffer[1] = Nav_Wpt_Index();            // waypoint index

    puiBuff = (uint16_t *)&ucTxBuffer[2];       // starting of data field

    *puiBuff++ = (uint16_t)Nav_Bearing();       // bearing to waypoint
    *puiBuff++ = (uint16_t)Nav_Wpt_Altitude();  // waypoint altitude
    *puiBuff++ = Nav_Distance();                // distance to waypoint
    *puiBuff   = (uint16_t)Nav_Heading();       // heading

    for (j = 0; j < 10; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, ucTxBuffer[j]);
    }
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
/// \param   gain, gain number
/// \return  gain value
/// \remarks -
///
///----------------------------------------------------------------------------
float Telemetry_Get_Gain(telEnum_Gain gain) {
    if (gain < TEL_GAIN_NUMBER) {
        return fGain[gain];
    } else {
        return 1.0f;
    }
}

///----------------------------------------------------------------------------
///
/// \brief   Get sensor value
/// \param   puiSensor, pointer to sensor data array
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void Telemetry_Get_Sensors(int16_t * piSensor) {
    uint8_t j;

    for (j = 0; j < 6; j++) {
        *piSensor++ = (int16_t)fSensor[j];
    }
}

//----------------------------------------------------------------------------
//
/// \brief   telemetry USART interrupt handler
/// \param   -
/// \returns -
/// \remarks -
///
//----------------------------------------------------------------------------
void USART1_IRQHandler( void ) {
//  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//  portCHAR cChar;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET) {
//		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
		ucRxBuffer[ucWindex++] = USART_ReceiveData( USART1 );
        if (ucWindex >= RX_BUFFER_LENGTH) {
            ucWindex = 0;
        }
    }
//	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

