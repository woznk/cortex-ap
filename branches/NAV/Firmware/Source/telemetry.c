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
//  CHANGES added functions Telemetry_Get_Gain() and Telemetry_Get_Sensors().
//          function Sim_Speed() renamed Telemetry_Sim_Speed().
//          removed transmission of simulator controls from telemetry task:
//          simulator controls are updated inside attitude task when simulator 
//          option is active.
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

#define TELEMETRY_FREQUENCY 50
#define TELEMETRY_DELAY     (configTICK_RATE_HZ / TELEMETRY_FREQUENCY)
#define RX_BUFFER_LENGTH    48
#define TX_BUFFER_LENGTH    48

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

typedef enum E_TELEMETRY {
    TEL_NULL,
    TEL_GPS_POSITION = 0xF0,
    TEL_SERVO_POS,
    TEL_WAYPOINT,
    TEL_DEBUG_I,
    TEL_DEBUG_F,
} wait_code_t;

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL xQueueHandle xTelemetry_Queue;

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
VAR_STATIC uint8_t ucRxBuffer[RX_BUFFER_LENGTH];    /// uplink data buffer
VAR_STATIC uint8_t ucTxBuffer[TX_BUFFER_LENGTH];    /// downlink data buffer
VAR_STATIC uint8_t ucWindex;                        /// upling write index
VAR_STATIC uint8_t ucRindex;                        /// uplink read index
VAR_STATIC uint8_t ucStatus;                        /// status of parser
VAR_STATIC float fTemp;                             /// temporary for parser
VAR_STATIC float fGain[TEL_GAIN_NUMBER];            /// gains for PID loops
VAR_STATIC float fSensor[8];                        /// simulator sensor data
VAR_STATIC float fTrueAirSpeed;                     /// simulator true air speed

/*--------------------------------- Prototypes -------------------------------*/

static void Telemetry_Init( void );
static void Telemetry_Send_Message(uint16_t *data, uint8_t num);
static void Telemetry_Send_DCM( void );
static void Telemetry_Parse( void );
static void Telemetry_Send_Controls( void );
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

    while (1) {
        vTaskDelayUntil(&Last_Wake_Time, TELEMETRY_DELAY);
        Telemetry_Parse();                      // parse uplink data
        if (++ucCycles >= TELEMETRY_FREQUENCY) {// every second
            ucCycles = 0;                       // reset cycle counter
            Telemetry_Send_Waypoint();          // send waypoint information
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
    ucStatus = 0;                                   // reset parser status
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
    long l_temp;
    uint8_t digit, i, j = 0;

    for (i = 0; i < num; i++) {
        l_temp = *data++;
        ucTxBuffer[j++] = ' ';
        digit = ((l_temp >> 12) & 0x0000000F);
        ucTxBuffer[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = ((l_temp >> 8) & 0x0000000F);
        ucTxBuffer[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = ((l_temp >> 4) & 0x0000000F);
        ucTxBuffer[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
        digit = (l_temp & 0x0000000F);
        ucTxBuffer[j++] = ((digit < 10) ? (digit + '0') : (digit - 10 + 'A'));
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

    uint8_t x, y, j = 0;
    unsigned char ucDigit;
    long lTemp;

    for (y = 0; y < 3; y++) {
      for (x = 0; x < 3; x++) {
          lTemp = (long)ceil(DCM_Matrix[y][x] * 32767.0f);
          ucTxBuffer[j++] = ' ';
          ucDigit = ((lTemp >> 12) & 0x0000000F);
          ucTxBuffer[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
          ucDigit = ((lTemp >> 8) & 0x0000000F);
          ucTxBuffer[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
          ucDigit = ((lTemp >> 4) & 0x0000000F);
          ucTxBuffer[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
          ucDigit = (lTemp & 0x0000000F);
          ucTxBuffer[j++] = ((ucDigit < 10) ? (ucDigit + '0') : (ucDigit - 10 + 'A'));
      }
    }
    ucTxBuffer[j++] = '\n';
    ucTxBuffer[j] = '\r';
    for (j = 0; j < 47; j++) {
      while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
      }
      USART_SendData(USART1, ucTxBuffer[j]);
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
static void Telemetry_Parse ( void )
{
    char c;
    while (ucRindex != ucWindex) {          // received another character
        c = ucRxBuffer[ucRindex++];         // read character

        if (ucRindex >= RX_BUFFER_LENGTH) { // update read index
            ucRindex = 0;
        }

        switch (ucStatus) {
            case 0:             /* ----------------- preamble ----------------- */
                if (c == '$') { ucStatus++; }
                break;
            case 1:
                switch (c) {
                    case 'S': ucStatus++; break;    // sensor data
                    case 'G': ucStatus = 10; break; // GPS data
                    case 'K': ucStatus = 19; break; // gain data
                    default : ucStatus = 0; break;  //
                }
                break;
            case 2:             /* ------------------ sensors ------------------ */
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

            case 10:            /* ------------------- GPS -------------------- */
                Nav_Gps_Putc('$');      // force initial characters into GPS buffer
                Nav_Gps_Putc('G');
                ucStatus++;
            case 11:
                if (c == '$') {         // next preamble received
                   ucStatus = 1;        // go check preamble type
                } else {                // still inside NMEA sentence
                   Nav_Gps_Putc(c);     // force character into GPS buffer
                }
                break;

            case 12:            /* --------------- PID gains ------------------ */
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
                break;

            default:
                fTemp = 0.0f;
                ucStatus = 0;
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
static void Telemetry_Send_Controls(void)
{
    uint8_t j;
    float *pfBuff;

    ucTxBuffer[0] = TEL_SERVO_POS;                // telemetry wait code

    pfBuff = (float *)&ucTxBuffer[1];             // elevator
    *pfBuff = (float)Servo_Get(SERVO_ELEVATOR);

    pfBuff = (float *)&ucTxBuffer[5];             // ailerons
    *pfBuff = (float)Servo_Get(SERVO_AILERON);

    pfBuff = (float *)&ucTxBuffer[9];             // rudder
    *pfBuff = (float)Servo_Get(SERVO_RUDDER);

    pfBuff = (float *)&ucTxBuffer[13];            // throttle
    *pfBuff = (float)Servo_Get(SERVO_THROTTLE);

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

    ucTxBuffer[0] = TEL_WAYPOINT;             // telemetry wait code

    ucTxBuffer[1] = Nav_WaypointIndex();      // waypoint index

    puiBuff = (uint16_t *)&ucTxBuffer[2];     // bearing to waypoint
    *puiBuff = (uint16_t)Nav_Bearing();

    puiBuff = (uint16_t *)&ucTxBuffer[4];     // waypoint altitude
    *puiBuff = (uint16_t)Nav_Altitude();

    puiBuff = (uint16_t *)&ucTxBuffer[6];     // distance to waypoint
    *puiBuff = Nav_Distance();

    for (j = 0; j < 8; j++) {
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
float Telemetry_Sim_Speed(void) {
    return fTrueAirSpeed;
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
/// \param   puiSensor, pointer to sensro data array
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
