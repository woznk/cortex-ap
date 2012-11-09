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
/// -------------------- Mavlink message structure --------------------
/// \code
///  Byte        Name        Content              Value
/// ------------------------------------------------------
///   1          MAVLINK_STX Start Transmission   0xFE
///   2          len         Length               0 - 255
///   3          seq         Sequence             0 - 255
///   4          SYSID       System identifier    0 - 255
///   5          COMPID      Component identifier 0 - 255
///   6          MSGID       Message identifier   0 - 255
///   7          Payload     Payload
///   7 + len    CRC 1
///   8 + len    CRC 2
/// \endcode
/// -------------------- Mavlink payload structure --------------------
/// \code
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
/// \endcode
/// ------------- Mavlink identifiers and message lengths -------------
/// \code
/// Message identifier (MSGID)           Value  Length
/// ---------------------------------------------------
/// MAVLINK_MSG_ID_HEARTBEAT                0      9
/// MAVLINK_MSG_ID_SYS_STATUS               1     31
/// MAVLINK_MSG_ID_GPS_RAW_INT             24     30
/// MAVLINK_MSG_ID_VFR_HUD                 74     20
/// MAVLINK_MSG_ID_ATTITUDE                30     28
/// MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT   62     26
/// MAVLINK_MSG_ID_MISSION_CURRENT         42      2
/// MAVLINK_MSG_ID_RC_CHANNELS_RAW         35     22
/// MAVLINK_MSG_ID_WIND                   168     12
/// \endcode
///
/// --------------------- Mavlink message contents --------------------
/// \code
/// Identifier (MSGID)                   Content           Offset  Type
/// -----------------------------------------------------------------------
/// MAVLINK_MSG_ID_HEARTBEAT             Heartbeat type       4   uint8_t
///                                      OSD mode             0   uint32_t
///                                      Base mode            6   uint8_t
///
/// MAVLINK_MSG_ID_SYS_STATUS            Battery voltage     14   uint16_t
///                                      Battery current     16   int16_t
///                                      Battery remaining   30   int8_t
///
/// MAVLINK_MSG_ID_GPS_RAW_INT           GPS latitude         8   int32_t
///                                      GPS longitude       12   int32_t
///                                      GPS fix             28   unit8_t
///                                      Visible satellites  29   uint8_t
///
/// MAVLINK_MSG_ID_VFR_HUD               Airspeed             0   float
///                                      Ground speed         4   float
///                                      Heading             16   int16_t
///                                      Throttle            18   uint16_t
///                                      Altitude             8   float
///                                      Climb rate          12   float
///
/// MAVLINK_MSG_ID_ATTITUDE              Pitch                8   float
///                                      Roll                12   float
///                                      Yaw                 16   float
///
/// MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT Nav roll             0   float
///                                      Nav pitch            4   float
///                                      Nav bearing         20   int16_t
///                                      Target bearing      22   int16_t
///                                      Waypoint distance   24   uint16_t
///                                      Altitude error       8   float
///                                      Airspeed error      12   float
///                                      Crosstrack error    16   float
///
/// MAVLINK_MSG_ID_MISSION_CURRENT       Waypoint number      0   uint16_t
///
/// MAVLINK_MSG_ID_RC_CHANNELS_RAW       Channel 1            4   uint16_t
///                                      Channel 2            6   uint16_t
///                                      Channel 5           12   uint16_t
///                                      Channel 6           14   uint16_t
///                                      Channel 7           16   uint16_t
///                                      Channel 8           18   uint16_t
///                                      RSSI                21   uint8_t
///
/// MAVLINK_MSG_ID_WIND                  Wind direction       0   float
///                                      Wind speed           4   float
///                                      Wind speed Z         8   float
///
/// \endcode
///
/// ------------------------- Mavlink commands ------------------------
///
/// APM 2.0 has adopted a subset of the MAVLink protocol command set.
///
/// ----------------------- Mavlink NAV commands ----------------------
///
/// Navigation commands have highest priority, all have a lat and lon component.
/// For commands of higher ID than the NAV commands, unexecuted commands are
/// dropped when ready for the next NAV command so plan/queue commands accordingly!
/// For example, if you had a string of CMD_MAV_CONDITION commands following a
/// 0x10 command that had not finished when the waypoint was reached, the
/// unexecuted CMD_MAV_CONDITION and CMD_MAV_DO commands would be skipped
/// and the next NAV command would be loaded.
///
/// \code
/// Command name                 ID  Parameters
/// --------------------------------------------------------------
/// MAV_CMD_NAV_WAYPOINT         16  -
///                                  altitude
///                                  lat
///                                  lon
/// MAV_CMD_NAV_LOITER_UNLIM     17  (indefinitely)
///                                  altitude
///                                  lat
///                                  lon
/// MAV_CMD_NAV_LOITER_TURNS     18  turns
///                                  altitude
///                                  lat
///                                  lon
/// MAV_CMD_NAV_LOITER_TIME      19  time (sec*10)
///                                  altitude
///                                  lat
///                                  lon
/// MAV_CMD_NAV_RETURN_TO_LAUNCH 20  -
///                                  altitude
///                                  lat
///                                  lon
/// MAV_CMD_NAV_LAND             21 -
///                                  altitude
///                                  lat
///                                  lon
/// MAV_CMD_NAV_TAKEOFF          22  takeoff pitch
///                                  altitude
///                                  -
///                                  -
/// Takeoff pitch specifies the minimum pitch for the case with
/// airspeed sensor and the target pitch for the case without.
/// MAV_CMD_NAV_TARGET           23  -
///                                  altitude
///                                  lat
///                                  lon
/// \endcode
///
/// ---------------------- Mavlink MAY commands -----------------------
///
/// These commands are optional to finish and have a end criteria,
/// eg "reached waypoint" or "reached altitude".
///
/// \code
/// Name                         ID  Parameters
/// --------------------------------------------------
/// MAV_CMD_CONDITION_DELAY      112 -
///                                  -
///                                  time(sec)
///                                  -
/// MAV_CMD_CONDITION_CHANGE_ALT 113
///                                  rate(cm/s) (rate must be > 10 cm/sec due to integer math)
///                                  alt(finish)
///                                  -
///                                  -
/// MAV_CMD_CONDITION_DISTANCE   114 -
///                                  -
///                                  distance(m)
///                                  -
/// \endcode
///
/// -------------------- Mavlink NOW Commands --------------------
///
/// These commands are executed once until no more new now commands
/// are available.
///
/// \code
/// Name                     ID  Parameters
/// ---------------------------------------------------------------------------
/// MAV_CMD_DO_JUMP          177 index
///                              -
///                              repeat count (1 = sinlge use, > 1 multiple use)
///                              -
/// MAV_CMD_DO_CHANGE_SPEED  178 Speed type (0 = Airspeed, 1 = Ground Speed)
///                              Speed (m/s) (-1 indicates no change)
///                              Throttle (%) (-1 indicates no change)
///                              -
/// MAV_CMD_DO_SET_HOME      179 Use current (1 = use current location, 0 = use specified location)
///                              altitude
///                              lat
///                              lon
/// MAV_CMD_DO_SET_PARAMETER 180 Param num
///                              Param value
///                              CURRENTLY NOT IMPLEMENTED IN APM
/// MAV_CMD_DO_SET_RELAY     181 Relay num
///                              On / off (1 / 0)
///                              -
///                              -
/// MAV_CMD_DO_REPEAT_RELAY  182 Relay num
///                              Cycle count
///                              Cycle time(sec)
///                              -
///                              Max cycle time = 60 sec.
///                              A repeat relay or repeat servo command
///                              will cancel any current repeating event
/// MAV_CMD_DO_SET_SERVO     183 Servo num (5-8)
///                              On / off (1 / 0)
///                              -
///                              -
/// MAV_CMD_DO_REPEAT_SERVO  184 Servo num (5-8)
///                              Cycle count
///                              Cycle time(sec)
///                              -
///                              Max cycle time = 60 sec.
///                              A repeat relay or repeat servo command
///                              will cancel any current repeating event
/// \endcode
///
/// ------------------------------ Links ------------------------------
///
/// ArduPilot Mega parameters modifiable by MAVLink
/// http://code.google.com/p/ardupilot-mega/wiki/MAVParam
///
/// MAVLink protocol specifications
/// http://qgroundcontrol.org/mavlink/start
/// http://qgroundcontrol.org/dev/mavlink_arduino_integration_tutorial
/// http://qgroundcontrol.org/dev/mavlink_onboard_integration_tutorial
/// https://pixhawk.ethz.ch/mavlink/
///
/// Changes: updated protocol description
///
//============================================================================*/

// ---- Include Files -------------------------------------------------------

#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "usart1driver.h"
#include "math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "misc.h"
#include "mavlink.h"
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
    USART1_Init();
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
        USART1_Transmit();                                  // Send the message
        communication_receive();                            // Process parameter request, if occured
        communication_queued_send();                        // Send parameters at 10 Hz, if previously requested
    }
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


