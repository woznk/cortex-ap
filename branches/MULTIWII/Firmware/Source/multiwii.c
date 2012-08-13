//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief multiwii telemetry interface
///
/// \file
///  see: https://github.com/wertarbyte/multiwii-firmware/blob/master/Serial.ino
///
//  Change  removed unused #definitions
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

#define TELEMETRY_FREQUENCY 50  //!< frequency of telemetry task
#define TELEMETRY_DELAY     (configTICK_RATE_HZ / TELEMETRY_FREQUENCY) //!< delay for telemetry task
#define RX_BUFFER_LENGTH    48  //!< length of receive buffer
#define TX_BUFFER_LENGTH    48  //!< length of transmit buffer
#define PAYLOAD_SIZE        16  //!< maximum size of payload

#define VERSION				0	//!< append multiwii version
#define MULTITYPE           0	//!< append type of multicopter
#define MSP_VERSION         0   //!< Multiwii Serial Protocol 0

/* outgoing messages */
#define MSP_IDENT           100 //!< multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS          101 //!< cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU         102 //!< raw IMU data, 9 DOF
#define MSP_SERVO           103 //!< 8 servos
#define MSP_MOTOR           104 //!< 8 motors
#define MSP_RC              105 //!< 8 rc channels
#define MSP_RAW_GPS         106 //!< fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS        107 //!< distance home, direction home
#define MSP_ATTITUDE        108 //!< roll, pitch, heading
#define MSP_ALTITUDE        109 //!< 1 altitude
#define MSP_BAT             110 //!< vbat, powermetersum
#define MSP_RC_TUNING       111 //!< rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID             112 //!< up to 16 PID
#define MSP_BOX             113 //!< up to 16 checkbox
#define MSP_MISC            114 //!< powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS      115 //!< which pins are in use for motors & servos, for GUI
#define MSP_BOXNAMES        116 //!< the aux switch names
#define MSP_PIDNAMES        117 //!< the PID names
#define MSP_WP              118 //!< get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
#define MSP_HEADING         125 //!< headings and MAG configuration
#define MSP_DEBUGMSG        253 //!< debug string buffer
#define MSP_DEBUG           254 //!< debug1,debug2,debug3,debug4

/* incoming messages */
#define MSP_SET_RAW_RC      200 //!< 8 rc chan
#define MSP_SET_RAW_GPS     201 //!< fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID         202 //!< set up to 16 PID
#define MSP_SET_BOX         203 //!< set up to 16 checkbox
#define MSP_SET_RC_TUNING   204 //!< rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION 205 //!< calibrate accelerometers
#define MSP_MAG_CALIBRATION 206 //!< calibrate magnetometers
#define MSP_SET_MISC        207 //!< powermeter trig + 8 free for future use
#define MSP_RESET_CONF      208 //!< reset configuration
#define MSP_WP_SET          209 //!< sets a given WP (WP#, lat, lon, alt, flags)

#define MSP_EEPROM_WRITE    250 //!< no param

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/// Status of multiwii header parser
typedef enum E_STATE {
    IDLE,           //!< waiting
    HEADER_START,   //!< start of header, $ received
    HEADER_M,       //!< multiwii protocol, M received
    HEADER_ARROW,   //!< direction of message, > or < received
    HEADER_SIZE,    //!< size of payload
    HEADER_CMD,     //!< command identifier
  } enum_status_t;

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/// Names of multiwii PID
const uint8_t pidnames[] =
  "ROLL;"
  "PITCH;"
  "YAW;"
  "ALT;"
  "Pos;"
  "PosR;"
  "NavR;"
  "LEVEL;"
  "MAG;"
  "VEL;"
;
/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC enum_status_t xStatus;                   //!< status of communication
VAR_STATIC uint8_t MSP_Checksum;                    //!< message checksum
VAR_STATIC uint8_t MSP_Command;                     //!< command identifier
VAR_STATIC uint8_t MSP_Size;                        //!< payload size
VAR_STATIC uint8_t MSP_Offset;                      //!< payload offset
VAR_STATIC uint8_t ucRxWindex;                      //!< uplink write index
VAR_STATIC uint8_t ucRxRindex;                      //!< uplink read index
VAR_STATIC uint8_t ucTxWindex;                      //!< downlink write index

VAR_STATIC uint8_t ucRxBuffer[RX_BUFFER_LENGTH];    //!< uplink data buffer
VAR_STATIC uint8_t ucTxBuffer[TX_BUFFER_LENGTH];    //!< downlink data buffer
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

static void Telemetry_Init( void );

/*---------------------------------- Functions -------------------------------*/


///----------------------------------------------------------------------------
///
/// \brief   reads a byte from message payload
/// \return  byte read
/// \param   -
/// \remarks
///
///----------------------------------------------------------------------------
uint8_t read8(void) {
  uint8_t t;
  t = ucRxBuffer[MSP_Offset++] & 0xFF;
  if (MSP_Offset >= RX_BUFFER_LENGTH) { // update offset
      MSP_Offset = 0;                   // wrap around
  }
  return t;
}


///----------------------------------------------------------------------------
///
/// \brief   reads a word from message payload
/// \return  word read
/// \param   -
/// \remarks
///
///----------------------------------------------------------------------------
uint16_t read16(void) {
  uint16_t t = read8();
  t += (uint16_t)read8() << 8;
  return t;
}


///----------------------------------------------------------------------------
///
/// \brief   reads a long word from message payload
/// \return  long word read
/// \param   -
/// \remarks
///
///----------------------------------------------------------------------------
uint32_t read32(void) {
  uint32_t t = read16();
  t += (uint32_t)read16() << 16;
  return t;
}

///----------------------------------------------------------------------------
///
/// \brief   Initialize multi Wii response message
/// \return  -
/// \param   length = message length
/// \remarks
///
///----------------------------------------------------------------------------
void __inline MSP_Init_Response(uint8_t length) {
  ucTxBuffer[0] = '$';              // header
  ucTxBuffer[1] = 'M';              //
  ucTxBuffer[2] = '>';              // outgoing message
  ucTxBuffer[3] = length;           // message length
  ucTxBuffer[4] = MSP_Command;      // command
  MSP_Checksum = 0;                 // clear checksum
  ucTxWindex = 5;                   // initialize write index
}

///----------------------------------------------------------------------------
///
/// \brief   Initialize multi Wii error message
/// \return  -
/// \param   length = message length
/// \remarks
///
///----------------------------------------------------------------------------
void __inline MSP_Init_Error(uint8_t length) {
  ucTxBuffer[0] = '$';              // header
  ucTxBuffer[1] = 'M';              //
  ucTxBuffer[2] = '!';              // error message
  ucTxBuffer[3] = length;           // message length
  ucTxBuffer[4] = MSP_Command;      // command
  MSP_Checksum = 0;                 // clear checksum
  ucTxWindex = 5;                   // initialize write index
}

///----------------------------------------------------------------------------
///
/// \brief   Append a byte to outgoing message
/// \return  -
/// \param   a = message byte
/// \remarks
///
///----------------------------------------------------------------------------
void MSP_Append_8(uint8_t a) {
  ucTxBuffer[ucTxWindex++] = a;
  MSP_Checksum ^= a;
}


///----------------------------------------------------------------------------
///
/// \brief   Append a word to outgoing message
/// \return  -
/// \param   a = message word
/// \remarks
///
///----------------------------------------------------------------------------
void MSP_Append_16(int16_t a) {
  MSP_Append_8((a     ) & 0xFF);
  MSP_Append_8((a >> 8) & 0xFF);
}


///----------------------------------------------------------------------------
///
/// \brief   Append a long word to outgoing message
/// \return  -
/// \param   a = message long word
/// \remarks -
///
///----------------------------------------------------------------------------
void MSP_Append_32(uint32_t a) {
  MSP_Append_8((a      ) & 0xFF);
  MSP_Append_8((a >>  8) & 0xFF);
  MSP_Append_8((a >> 16) & 0xFF);
  MSP_Append_8((a >> 24) & 0xFF);
}


///----------------------------------------------------------------------------
///
/// \brief   Append a null terminated string to outgoing message
/// \return  -
/// \param   s = pointer to null terminated string
/// \remarks -
///
///----------------------------------------------------------------------------
void MSP_Append_Name(const uint8_t* s) {
  while (*s != 0) {
  	MSP_Append_8(*s++);
  }
}


///----------------------------------------------------------------------------
///
/// \brief   parses multiwii commands
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void MSP_Parse_Command( void ) {
  uint8_t i;

  switch (MSP_Command) {

    case MSP_SET_PID:					// set PID values
     for (i = 0; i < TEL_GAIN_NUMBER; i++) { // PITCH_KP, PITCH_KI
	 	fGain[i] = read8();             // ROLL_KP, ROLL_KI,
     }                                  // NAV_KP, NAV_KI,
     MSP_Init_Response(0);				// initialize response
     break;

    case MSP_IDENT:                     // requested identification
     MSP_Init_Response(7);              // initialize response
     MSP_Append_8(VERSION);             // append multiwii version
     MSP_Append_8(MULTITYPE);           // append type of multicopter
     MSP_Append_8(MSP_VERSION);         // MultiWii Serial Protocol Version
     MSP_Append_32(0);                  // append "capability"
     break;

    case MSP_STATUS:                    // requested status
     MSP_Init_Response(10);             //
     MSP_Append_16(0);                  // cycleTime, time needed to complete a loop
     MSP_Append_16(0);                  // i2c_errors_count, I2C errors
     MSP_Append_16(ACC | BARO << 1 |    // available sensors
                   MAG << 2 | GPS << 3 |
                   SONAR << 4 );
     MSP_Append_32(0                    // flags and options
                                        // f.ACC_MODE      << BOXACC |
                                        // f.BARO_MODE     << BOXBARO |
                                        // f.MAG_MODE      << BOXMAG |
                                        // f.ARMED         << BOXARM |
                                        // f.GPS_HOME_MODE << BOXGPSHOME |
                                        // f.GPS_HOLD_MODE << BOXGPSHOLD |
                                        // f.HEADFREE_MODE << BOXHEADFREE |
                                        // f.PASSTHRU_MODE << BOXPASSTHRU |
                                        // rcOptions[BOXCAMSTAB]  << BOXCAMSTAB |
                                        // rcOptions[BOXCAMTRIG]  << BOXCAMTRIG |
                                        // rcOptions[BOXBEEPERON] << BOXBEEPERON |
                                        // rcOptions[BOXLEDMAX]   << BOXLEDMAX |
                                        // rcOptions[BOXLLIGHTS]  << BOXLLIGHTS |
                                        // rcOptions[BOXHEADADJ]  << BOXHEADADJ
				  );
     break;

    case MSP_ATTITUDE:                  // requested attitude
     MSP_Init_Response(8);              // initialize response
     for (i = 0; i < 2; i++)			// append presumably roll, pitch, yaw
         MSP_Append_16(0);              // angle[i], roll pitch
     MSP_Append_16(0);                  // heading, heading
     MSP_Append_16(0);	                // headFreeModeHold, ?
     break;

    case MSP_HEADING:                   // requested heading
     MSP_Init_Response(7);              // initialize response
     MSP_Append_8(0                     // f.MAG_MODE      << 0 | mag stabilization mode
                                        // f.HEADFREE_MODE << 1   headfree mode
				  );
     MSP_Append_16(0);                  // heading, heading
     MSP_Append_16(0);                  // magHold, magnetic orientation to hold
     MSP_Append_16(0);                  // headFreeModeHold, ?
     break;

    case MSP_ALTITUDE:                  // requested altitude
     MSP_Init_Response(4);              // initialize response
     MSP_Append_32(0);                  // EstAlt, estimated barometric altitude
     break;

    case MSP_PID:                       // requested PID values
     MSP_Init_Response(TEL_GAIN_NUMBER);// initialize response
     for (i = 0; i < TEL_GAIN_NUMBER; i++) { //
       MSP_Append_8(fGain[i]);	        // append gain
     }
     break;

    case MSP_PIDNAMES:                  // requested PID names
     MSP_Init_Response(sizeof(pidnames)); // initialize response
     MSP_Append_Name(pidnames);         // append PID names
     break;

#if defined(USE_MSP_WP)
   case MSP_WP:                         // requested waypoint
    i = read8();                        // get the number of required wp
    MSP_Init_Response(12);              // initialize response
    if (i == 0) {                       // home position (waypoint #0)
      MSP_Append_8(0);                  // append waypoint number
      MSP_Append_32(GPS_home[LAT]);     // append home latitude
      MSP_Append_32(GPS_home[LON]);     // append home longitude
      MSP_Append_16(0);                 // altitude will come here
      MSP_Append_8(0);                  // nav flag will come here
    } else if (i == 16) {               // hold position (waypoint #16)
      MSP_Append_8(16);                 // append waypoint number
      MSP_Append_32(GPS_hold[LAT]);     // append hold latitude
      MSP_Append_32(GPS_hold[LON]);     // append hold longitude
      MSP_Append_16(0);                 // altitude will come here
      MSP_Append_8(0);                  // nav flag will come here
    }
    break;
#endif

   case MSP_RESET_CONF:                 // reset configuration
     MSP_Init_Response(0);              // initialize response
     break;

   case MSP_EEPROM_WRITE:               // write parameters to eeprom
     MSP_Init_Response(0);              // initialize response
     break;

   case MSP_DEBUG:                      // debug message
     MSP_Init_Response(0);              // initialize response
     break;

   default :                           // indicate error "$M!"
     MSP_Init_Error(0);                // initialize error message
     break;
  }
  MSP_Append_8(MSP_Checksum);
  for (i = 0; i < ucTxBuffer[3] + 3; i++) {
    while (0/*USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET*/) {
    }
//    USART_SendData(USART1, ucTxBuffer[i]);
  }
}


///----------------------------------------------------------------------------
///
/// \brief   manages multi wii serial protocol reception
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void MSP_Receive( void ) {
  uint8_t c;
  static uint8_t counter;

  while (ucRxRindex != ucRxWindex) {      // receive buffer not empty
    c = ucRxBuffer[ucRxRindex++];         // read character
    if (ucRxRindex >= RX_BUFFER_LENGTH) { // update read index
        ucRxRindex = 0;                   // wrap around
    }
    switch (xStatus) {
        case IDLE:                        // idle
            if (c == '$') {               // message start
                xStatus = HEADER_START;   //
            } else {                      //
                xStatus = IDLE;           //
            }
            break;

        case HEADER_START:                // message started
            if (c == 'M') {               // Multiwii protocol
                xStatus = HEADER_M;       //
            } else {                      //
                xStatus = IDLE;           //
            }
            break;

        case HEADER_M:                    //
            if (c == '<') {               // incoming message
                xStatus = HEADER_ARROW;   //
            } else {                      //
                xStatus = IDLE;           //
            }
            break;

        case HEADER_ARROW:                //
            if (c > PAYLOAD_SIZE) {       // payload size too big
                xStatus = IDLE;           // ignore rest of message
            } else {                      // payload size fits
                counter = 0;              // clear payload bytes counter
                MSP_Size = c;             // save payload size
                MSP_Checksum = 0;         // clear message checksum
                MSP_Checksum ^= c;        // start computing checksum
                xStatus = HEADER_SIZE;    //
            }
            break;

        case HEADER_SIZE:                 //
            MSP_Command = c;              // save command byte
            MSP_Offset = ucRxRindex;      // save offset of message start
            MSP_Checksum ^= c;            // keep computing checksum
            xStatus = HEADER_CMD;         //
            break;                        //

        case HEADER_CMD:                  //
            if (counter < MSP_Size) {     // not all expected bytes have been received
                MSP_Checksum ^= c;        // compute checksum
                counter++;
            } else if (MSP_Checksum == c) { // calculated and transferred checksums match
                MSP_Parse_Command();      // valid packet, evaluate it
                xStatus = IDLE;           //
            }
            break;

        default :
            xStatus = IDLE;               //
            break;
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
/*
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Initialize USART1 structure
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);       // configure USART1

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  // enable USART1 interrupt
    //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

    // Configure NVIC for USART1 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_KERNEL_INTERRUPT_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init( &NVIC_InitStructure );

    USART_Cmd(USART1, ENABLE);                      // enable the USART1
*/
    ucRxWindex = 0;                                 // clear uplink write index
    ucRxRindex = 0;                                 // clear uplink read index
    ucTxWindex = 0;                                 // clear downlink write index
    xStatus = IDLE;                                 // reset reception status
}

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
/*
    uint8_t ucCycles = 0;                       //
    portTickType Last_Wake_Time;                //
    Last_Wake_Time = xTaskGetTickCount();       //
*/
    Telemetry_Init();                           // telemetry initialization

    while (TRUE) {
//        vTaskDelayUntil(&Last_Wake_Time, TELEMETRY_DELAY);
        MSP_Receive();          				//
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
    return 0.0f;//fTrueAirSpeed;
}

///----------------------------------------------------------------------------
///
/// \brief   interface to simulator data : altitude
/// \return  altitude
/// \remarks -
///
///----------------------------------------------------------------------------
float Telemetry_Get_Altitude(void) {
    return 0.0f;//fAltitude;
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
/// \param   piSensor = pointer to sensor data array
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
void Telemetry_Get_Sensors(int16_t * piSensor) {
/*
    uint8_t j;
    for (j = 0; j < 6; j++) {
        *piSensor++ = (int16_t)fSensor[j];
    }
*/
}
