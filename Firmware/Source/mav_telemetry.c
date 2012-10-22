//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief
/// Tentative Mavlink protocol implementation
///
/// \file
/// -------------------- Mavlink command structure --------------------
/// \code
/// Byte # Address Data type Function
///   0    0x00    byte      Command ID
///   1    0x01    byte      Options
///   2    0x02    byte      Parameter 1
///   3    0x03    long      Parameter 2
///   4    0x04    ..
///   5    0x05    ..
///   6    0x06    ..
///   7    0x07    long      Parameter 3
///   8    0x08    ..
///   9    0x09    ..
///   10   0x0A    ..
///   11   0x0B    long      Parameter 4
///   12   0x0C    ..
///   13   0x0D    ..
///   14   0x0E    ..
///   15   0x0F    ?         CRC ?
///   16   0x10    ?         CRC ?
/// \endcode
/// APM 2.0 has adopted a subset of the MAVLink protocol command set.
/// -------------------- NAVigation Commands --------------------
/// Highest priority, all have a lat and lon component.
/// For commands of higher ID than the NAV commands, unexecuted commands are
/// dropped when ready for the next NAV command so plan/queue commands accordingly!
/// For example, if you had a string of CMD_MAV_CONDITION commands following a 0x10
/// command that had not finished when the waypoint was reached, the unexecuted
/// CMD_MAV_CONDITION and CMD_MAV_DO commands would be skipped and the next NAV
/// command would be loaded.
/// \code
/// Cmd ID   Name                        Parameter 1    Altitude Lat Lon Notes
/// hex/dec
/// 0x10/16 MAV_CMD_NAV_WAYPOINT         -              altitude lat lon
/// 0x11/17 MAV_CMD_NAV_LOITER_UNLIM     (indefinitely) altitude lat lon
/// 0x12/18 MAV_CMD_NAV_LOITER_TURNS     turns          altitude lat lon
/// 0x13/19 MAV_CMD_NAV_LOITER_TIME      time (sec*10)  altitude lat lon
/// 0x14/20 MAV_CMD_NAV_RETURN_TO_LAUNCH -              altitude lat lon
/// 0x15/21 MAV_CMD_NAV_LAND             -              altitude lat lon
/// 0x16/22 MAV_CMD_NAV_TAKEOFF          takeoff pitch  altitude -   -   takeoff pitch specifies the minimum
///                                                                      pitch for the case with airspeed
///                                                                      sensor and the target pitch for the
///                                                                      case without.
/// 0x17/23 MAV_CMD_NAV_TARGET            -             altitude lat lon
/// \endcode
/// -------------------- May Commands --------------------
/// These commands are optional to finish and have a end criteria, eg
/// "reached waypoint" or "reached altitude".
/// \code
/// Cmd ID   Name                         Parameter 1 Parameter 2 Parameter 3 Parameter 4 Notes
/// 0x70/112 MAV_CMD_CONDITION_DELAY      -           -           time(sec)   -
/// 0x71/113 MAV_CMD_CONDITION_CHANGE_ALT rate(cm/s)  alt(finish) -           -           rate must be > 10 cm/sec
///                                                                                       due to integer math
/// 0x72/114 MAV_CMD_CONDITION_DISTANCE   -           -           distance(m) -
/// \endcode
/// -------------------- Now Commands --------------------
/// These commands are executed once until no more new now commands are available
/// \code
/// Cmd ID   Name                     Parameter 1     Parameter 2  Parameter 3     Parameter 4 Notes
/// 0xB1/177 MAV_CMD_DO_JUMP          index           -            repeat count    -           The repeat count must be greater than 1 for the command to execute.
///                                                                                            Use a repeat count of 1 if you intend a single use.
/// 0xB2/178 MAV_CMD_DO_CHANGE_SPEED  Speed type      Speed (m/s)  Throttle (%)    -           Speed type: 0 = Airspeed, 1 = Ground Speed
///                                                                                            Speed: -1 indicates no change
///                                                                                            Throttle: -1 indicates no change
/// 0xB3/179 MAV_CMD_DO_SET_HOME      Use current     altitude     lat             lon         Use current : 1 = use current location, 0 = use specified location
/// 0xB4/180 MAV_CMD_DO_SET_PARAMETER Param num       Param value                              CURRENTLY NOT IMPLEMENTED IN APM
/// 0xB5/181 MAV_CMD_DO_SET_RELAY     Relay num       On/off(1/0)  -               -
/// 0xB6/182 MAV_CMD_DO_REPEAT_RELAY  Relay num       Cycle count  Cycle time(sec) -           Max cycle time = 60 sec, A repeat relay or repeat servo command
///                                                                                            will cancel any current repeating event
/// 0xB7/183 MAV_CMD_DO_SET_SERVO     Servo num (5-8) On/off(1/0)  -               -
/// 0xB6/184 MAV_CMD_DO_REPEAT_SERVO  Servo num (5-8) Cycle count  Cycle time(sec) -           Max cycle time = 60 sec, A repeat relay or repeat servo command
///                                                                                            will cancel any current repeating event
/// \endcode
/// List of ArduPilot Mega parameters modifiable by MAVLink:
///     http://code.google.com/p/ardupilot-mega/wiki/MAVParam
///
/// MAVLink protocol specifications:
///     http://qgroundcontrol.org/mavlink/start
///     http://qgroundcontrol.org/dev/mavlink_arduino_integration_tutorial
///     http://qgroundcontrol.org/dev/mavlink_onboard_integration_tutorial
///
//  CHANGES disabled references to global_data structure
//
//============================================================================*/

// ---- Include Files -------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "misc.h"
#include "mavlink.h"

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "math.h"
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

#define TELEMETRY_FREQUENCY 10
#define TELEMETRY_DELAY     (configTICK_RATE_HZ / TELEMETRY_FREQUENCY)
#define RX_BUFFER_LENGTH    48
#define TX_BUFFER_LENGTH    48
#define TEL_DCM_LENGTH      (1 + (9 * 4))

#define ONBOARD_PARAM_COUNT 5
#define ONBOARD_PARAM_NAME_LENGTH 8
#define PARAM_SYSTEM_ID     1
#define PARAM_COMPONENT_ID  2

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/
/*
struct global_struct
{
    float param[ONBOARD_PARAM_COUNT];
    char param_name[ONBOARD_PARAM_COUNT][MAVLINK_MSG_PARAM_SET_FIELD_PARAM_ID_LEN];
};
*/
/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

//VAR_GLOBAL xQueueHandle xTelemetry_Queue;
//VAR_GLOBAL struct global_struct global_data;

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC uint8_t system_type;
VAR_STATIC uint8_t system_mode;
VAR_STATIC uint8_t system_state;
VAR_STATIC uint8_t autopilot_type;

VAR_STATIC uint16_t len;
VAR_STATIC uint16_t packet_drops = 0;
//VAR_STATIC uint16_t mode = MAV_MODE_UNINIT;         // Defined in mavlink_types.h, which is included by mavlink.h
VAR_STATIC uint16_t m_parameter_i = 0;
VAR_STATIC uint32_t custom_mode;

VAR_STATIC mavlink_system_t mavlink_system;
VAR_STATIC mavlink_param_set_t set;
VAR_STATIC mavlink_status_t status;
VAR_STATIC mavlink_message_t msg;                   // Initialize the required buffers

VAR_STATIC uint8_t buf[MAVLINK_MAX_PACKET_LEN];
VAR_STATIC uint8_t ucRxBuffer[RX_BUFFER_LENGTH];    // uplink data buffer
//VAR_STATIC uint8_t ucTxBuffer[TX_BUFFER_LENGTH];    // downlink data buffer
VAR_STATIC uint8_t ucWindex;                        // uplink write index
VAR_STATIC uint8_t ucRindex;                        // uplink read index

/*--------------------------------- Prototypes -------------------------------*/

static void Telemetry_Init( void );
static void Telemetry_Downlink (uint8_t * buf, uint16_t len);
static void communication_queued_send(void);
static void communication_receive(void);

/*---------------------------------- Functions -------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief   reset all parameters to default
/// \return  -
/// \warning DO NOT USE THIS IN FLIGHT!
/// \remarks -
///
///
///----------------------------------------------------------------------------
/*
static __inline void global_data_reset_param_defaults( void )
{
    global_data.param[PARAM_SYSTEM_ID] = 42;
    strcpy(global_data.param_name[PARAM_SYSTEM_ID], "SYS_ID");
}
*/

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
    portTickType Last_Wake_Time;

    Last_Wake_Time = xTaskGetTickCount();       //
    Telemetry_Init();                           // telemetry initialization
//    global_data_reset_param_defaults();         // Load default parameters as fallback

    mavlink_system.sysid = 20;                  // ID 20 for this airplane
//    mavlink_system.compid = MAV_COMP_ID_IMU;    // The component sending the message is the IMU
    mavlink_system.type = MAV_TYPE_FIXED_WING;  // This system is an airplane / fixed wing
    system_type = MAV_TYPE_FIXED_WING;          // Define the system type, in this case an airplane
    system_state = MAV_STATE_STANDBY;           // System ready for flight
//    system_mode = MAV_MODE_PREFLIGHT;           // Booting up
    autopilot_type = MAV_AUTOPILOT_GENERIC;
    custom_mode = 0;                            // Custom mode, can be defined by user/adopter

    while (TRUE)
    {
        vTaskDelayUntil(&Last_Wake_Time, TELEMETRY_DELAY);  // Use any wait function you want, better not use sleep
        mavlink_msg_heartbeat_pack( mavlink_system.sysid,   // Pack the message
                                    mavlink_system.compid,
                                    &msg,
                                    system_type,
                                    autopilot_type,
                                    system_mode,
                                    custom_mode,
                                    system_state);
        len = mavlink_msg_to_send_buffer(buf, &msg);        // Copy the message to the send buffer
        Telemetry_Downlink(buf, len);                       // Send the message
        communication_receive();                            // Process parameter request, if occured
        communication_queued_send();                        // Send parameters at 10 Hz, if previously requested
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
static void Telemetry_Init( void )
{
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
//        xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );
        ucRxBuffer[ucWindex++] = USART_ReceiveData( USART1 );
        if (ucWindex >= RX_BUFFER_LENGTH) {
            ucWindex = 0;
        }
    }
//    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}


//----------------------------------------------------------------------------
//
/// \brief
/// \param   -
/// \returns -
/// \remarks -
///
//----------------------------------------------------------------------------
static void handle_mavlink_message(mavlink_channel_t chan,
                                   mavlink_message_t* msg)
{
    uint16_t i, j;
    char * key;
    bool match;

    switch (chan)
	{
        case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
            m_parameter_i = 0;        // Start sending parameters
            break;

        case MAVLINK_MSG_ID_PARAM_SET:
            mavlink_msg_param_set_decode(msg, &set);
/*
            if (((uint8_t) set.target_system == (uint8_t) global_data.param[PARAM_SYSTEM_ID]) &&    // Check if this message is for this system
                ((uint8_t) set.target_component == (uint8_t) global_data.param[PARAM_COMPONENT_ID]))
            {
                key = (char *) set.param_id;
                for (i = 0; i < ONBOARD_PARAM_COUNT; i++)
                {
                    match = TRUE;
                    for (j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++)
                    {
                        if ((char)(key[j]) !=
                            (char)(global_data.param_name[i][j]))          // Compare
                        {
                            match = FALSE;
                        }
                        if (((char) global_data.param_name[i][j]) == '\0')  // End comparison if null termination is reached
                        {
                            break;
                        }
                    }
*/
                    if (match)                                              // Check if matched
                    {
/*
                        if ((global_data.param[i] != set.param_value) &&    // Write and emit changes if there is a difference
                            !isnan(set.param_value) &&                      // AND if new value is NOT "not-a-number"
                            !isinf(set.param_value) &&                      // AND is NOT infinity
                            (set.param_type == MAVLINK_TYPE_FLOAT))
                        {
                            global_data.param[i] = set.param_value;
*/
                            mavlink_msg_param_value_send(MAVLINK_COMM_0,    // Report back new value
                                                        0, //(int8_t*) global_data.param_name[i],
                                                        0, //global_data.param[i], MAVLINK_TYPE_FLOAT,
                                                        ONBOARD_PARAM_COUNT,
														m_parameter_i);
//                        }
                    }
//                }
//            }
            break;
        default:
            break;
    }
}


//----------------------------------------------------------------------------
//
/// \brief   Send low-priority messages at a maximum rate of xx Hertz
/// \param   -
/// \returns -
/// \remarks This function sends messages at a lower rate to not exceed the
///          wireless bandwidth. It sends one message each time it is called
///          until the buffer is empty.
///          Call this function with xx Hertz to increase/decrease the bandwidth.
///
//----------------------------------------------------------------------------

static void communication_queued_send(void)
{
    if (m_parameter_i < ONBOARD_PARAM_COUNT)     //send parameters one by one
    {
        mavlink_msg_param_value_send(MAVLINK_COMM_0,
                                    0, //(int8_t*) global_data.param_name[m_parameter_i],
                                    0, //global_data.param[m_parameter_i],
                                    MAVLINK_TYPE_FLOAT,
                                    ONBOARD_PARAM_COUNT,
                                    m_parameter_i);
        m_parameter_i++;
    }
}


//----------------------------------------------------------------------------
//
/// \brief   Receive communication packets and handle them
/// \param   -
/// \returns -
/// \remarks This function decodes packets on the protocol level and also
///          handles their value by calling the appropriate functions.
///
//----------------------------------------------------------------------------

static void communication_receive(void)
{
    uint8_t c;

    while (ucRindex != ucWindex)                // received another character
    {
        c = ucRxBuffer[ucRindex++];             // read character
        if (ucRindex >= RX_BUFFER_LENGTH)       // update read index
        {
            ucRindex = 0;
        }
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
        {
			switch (msg.msgid)			        // Handle message
			{
                case MAVLINK_MSG_ID_HEARTBEAT:
			    {
                    // E.g. read GCS heartbeat and go into comm lost mode if timer times out
                    // Handle message the same way like in for UART0
                    // you can also consider to write a handle function like
                    // handle_mavlink(mavlink_channel_t chan, mavlink_message_t* msg)
                    // Which handles the messages for both or more UARTS
                }
			    break;
			case MAVLINK_MSG_ID_COMMAND_LONG:	// EXECUTE ACTION
				break;
			default:				            // Do nothing
				break;
			}
		}		                                // And get the next one
	}

	packet_drops += status.packet_rx_drop_count;    // Update global packet drops counter
}


//----------------------------------------------------------------------------
//
/// \brief   Send the message with the standard UART send function
/// \param   -
/// \returns -
/// \remarks
///
///
//----------------------------------------------------------------------------
static void Telemetry_Downlink (uint8_t * buf, uint16_t len)
{
    uint16_t j;

    for (j = 0; j < len; j++)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
        {
        }
        USART_SendData(USART1, buf[j]);
    }
}

//----------------------------------------------------------------------------
//
/// \brief
/// \param   -
/// \returns -
/// \remarks
///
//----------------------------------------------------------------------------
void Telemetry_Get_Sensors(int16_t * piSensors)
{}
//----------------------------------------------------------------------------
//
/// \brief
/// \param   -
/// \returns -
/// \remarks
///
//----------------------------------------------------------------------------
float Telemetry_Get_Gain(telEnum_Gain gain)
{}
//----------------------------------------------------------------------------
//
/// \brief
/// \param   -
/// \returns -
/// \remarks
///
//----------------------------------------------------------------------------
void Telemetry_Send_Controls(void)
{}
//----------------------------------------------------------------------------
//
/// \brief
/// \param   -
/// \returns -
/// \remarks
///
//----------------------------------------------------------------------------
float Telemetry_Get_Speed(void)
{}
//----------------------------------------------------------------------------
//
/// \brief
/// \param   -
/// \returns -
/// \remarks
///
//----------------------------------------------------------------------------
float Telemetry_Get_Altitude(void)
{}


