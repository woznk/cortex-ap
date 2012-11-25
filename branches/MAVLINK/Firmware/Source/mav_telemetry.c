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
///   0          MAVLINK_STX Start Transmission   0xFE
///   1          len         Length               0 - 255
///   2          seq         Sequence             0 - 255
///   3          SYSID       System identifier    0 - 255
///   4          COMPID      Component identifier 0 - 255
///   5          MSGID       Message identifier   0 - 255
///   6          Payload     Payload
///   7 + len    CRC 1
///   8 + len    CRC 2
/// \endcode
/// ------------- Mavlink identifiers and message lengths -------------
///
/// All identifiers are prefixed with MAVLINK_MSG_ID_ in the code, e.g.
/// ID of HEARTBEAT message will be MAVLINK_MSG_ID_HEARTBEAT
///
/// \code
/// Identifier (MSGID)    Value  Length Status
/// ---------------------------------------------------
/// HEARTBEAT                0      9   Verified
/// SYS_STATUS               1     31
/// PARAM_REQUEST_LIST      21      2   Implemented
/// PARAM_VALUE             22     25   Implemented
/// PARAM_SET               23     23
/// GPS_RAW_INT             24     30   Verified
/// ATTITUDE                30     28   Verified
/// RC_CHANNELS_RAW         35     22
/// MISSION_CURRENT         42      2
/// MISSION_REQUEST_LIST    43      2
/// MISSION_COUNT           44      4
/// MISSION_CLEAR_ALL       45      2
/// NAV_CONTROLLER_OUTPUT   62     26
/// REQUEST_DATA_STREAM     66      6
/// VFR_HUD                 74     20   Verified
/// COMMAND_LONG            76     21?
/// WIND                   168     12
/// \endcode
///
/// --------------------- Mavlink message contents --------------------
///
/// All identifiers are prefixed with MAVLINK_MSG_ID_ in the code, e.g.
/// ID of HEARTBEAT message will be MAVLINK_MSG_ID_HEARTBEAT
///
/// \code
/// Identifier (MSGID)    Content           Offset  Type    Note
/// -----------------------------------------------------------------------
/// HEARTBEAT             see code
///
/// SYS_STATUS            Battery voltage     14   uint16_t
///                       Battery current     16   int16_t
///                       Battery remaining   30   int8_t
///
/// PARAM_REQUEST_LIST    Target System        ?   uint8_t
///                       System ID            ?            Request all parameters of this component.
///                       Target Component     ?   uint8_t  Component ID
///                                                         After his request, all parameters are emitted.
///
/// PARAM_SET             param_value          0   float    Onboard parameter value
///                       target_system        4   uint8_t  System ID
///                       target_component     5   uint8_t  Component ID
///                       param_id, 16         6   array    Onboard parameter name, null terminated if length < 16 chars
///                                                         and WITHOUT null termination if length = 16 chars
///                                                         applications have to provide 16+1 bytes storage if the name
///                                                         is stored as string
///                       param_type          22   uint8_t  Onboard parameter type: see MAVLINK_TYPE enum
///
/// PARAM_VALUE           see code
///
/// GPS_RAW_INT           see code
///
/// VFR_HUD               see code
///
/// ATTITUDE              see code
///
/// NAV_CONTROLLER_OUTPUT Nav roll             0   float
///                       Nav pitch            4   float
///                       Altitude error       8   float
///                       Airspeed error      12   float
///                       Crosstrack error    16   float
///                       Nav bearing         20   int16_t
///                       Target bearing      22   int16_t
///                       Waypoint distance   24   uint16_t
///
/// MISSION_CURRENT       Waypoint number      0   uint16_t
///
/// RC_CHANNELS_RAW       Channel 1            4   uint16_t
///                       Channel 2            6   uint16_t
///                       Channel 5           12   uint16_t
///                       Channel 6           14   uint16_t
///                       Channel 7           16   uint16_t
///                       Channel 8           18   uint16_t
///                       RSSI                21   uint8_t
///
/// MISSION_CLEAR_ALL     target_system        ?   uint8_t   System ID
///                       target_component	   ?   uint8_t   Component ID
///
/// MISSION_REQUEST_LIST  target_system	       ?   uint8_t  System ID
///                       target_component     ?   uint8_t  Component ID
///
/// MISSION_COUNT         target_system        ?   uint8_t  System ID
///                       target_component     ?   uint8_t  Component ID
///                       count                ?   uint16_t Number of mission items in the sequence
/// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
/// The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
///
/// REQUEST_DATA_STREAM   target_system        ?   uint8_t  The target requested to send the message stream.
///                       target_component     ?   uint8_t  The target requested to send the message stream.
///                       req_stream_id        ?   uint8_t  The ID of the requested data stream
///                       req_message_rate     ?   uint16_t The requested interval between two messages of this type
///                       start_stop           ?   uint8_t  1 to start sending, 0 to stop sending.
///
/// WIND                  Wind direction       0   float
///                       Wind speed           4   float
///                       Wind speed Z         8   float
///
/// COMMAND_LONG          target_system        ?   uint8_t  System which should execute the command
///                       target_component     ?   uint8_t  Component which should execute the command, 0 for all components
///                       command              ?   uint16_t Command ID, as defined by MAV_CMD enum.
///                       confirmation         ?   uint8_t  0=First transmission. 1-255=Confirmation transmissions (e.g. for kill command)
///                       param1               ?   float    Parameter 1, as defined by MAV_CMD enum.
///                       param2               ?   float    Parameter 2, as defined by MAV_CMD enum.
///                       param3               ?   float    Parameter 3, as defined by MAV_CMD enum.
///                       param4               ?   float    Parameter 4, as defined by MAV_CMD enum.
///                       param5               ?   float    Parameter 5, as defined by MAV_CMD enum.
///                       param6               ?   float    Parameter 6, as defined by MAV_CMD enum.
///                       param7               ?   float    Parameter 7, as defined by MAV_CMD enum.
/// Send a command with up to four parameters to the MAV
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
///--------------------- AqGCS messages taxonomy ------------------------
///
/// All menues result in the transmission of one or more Mavlink messages.
///
/// Menu            Message              ID Len
/// --------------------------------------------
/// Connect         heartbeat            00  9
/// Pid             param request list   15  2
/// Pid exit        request data stream  42  6
/// All param       param request list   15  2
/// All param exit  request data stream  42  6
/// Hud             request data stream  42  6
///                 request data stream  42  6
/// Hud exit        request data stream  42  6
/// Gps             request data stream  42  6
/// Gps exit        request data stream  42  6
/// Save mission    mission count        2C  4
/// Mission exit    request data stream  42  6
/// Tx exit         request data stream  42  6
/// Disconnect      request data stream  42  6
///
///--------------------- aq-gcs2 messages taxonomy ------------------------
///
/// All menues result in the transmission of one or more Mavlink messages.
///
/// Menu            Message              ID Len
/// --------------------------------------------
/// Read from rom   command long         4C  21
/// Save to rom     command long         4C  21
/// Pid             param request list   15  2
/// Save mission    mission clear all    2D  2
/// Retrieve miss.  mission request list 2B  2
///
/// ------------------------------ Links ------------------------------
///
/// ArduPilot Mega parameters modifiable by MAVLink
/// http://code.google.com/p/ardupilot-mega/wiki/MAVParam
///
/// ArduPilot Mega MAVLink commands
/// http://code.google.com/p/ardupilot-mega/wiki/MAVLink
///
/// MAVLink protocol specifications
/// http://qgroundcontrol.org/mavlink/start
/// http://qgroundcontrol.org/dev/mavlink_arduino_integration_tutorial
/// http://qgroundcontrol.org/dev/mavlink_onboard_integration_tutorial
///
/// List of commands
/// https://pixhawk.ethz.ch/mavlink/
///
/// Changes: function Mavlink_Receive() renamed Mavlink_Parse(), 
///          added Mavlink_Receive()Mavlink_Param_Send(), Mavlink_Queued_Send(), 
///          implemented PARAM REQUEST LIST
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

#include "config.h"
#include "misc.h"
#include "nav.h"
#include "attitude.h"
#include "mavlink.h"
#include "telemetry.h"
#include "mav_telemetry.h"

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

#define ONBOARD_PARAM_COUNT         TEL_GAIN_NUMBER
#define ONBOARD_PARAM_NAME_LENGTH   8

#define PARAM_SYSTEM_ID     1
#define PARAM_COMPONENT_ID  2

//#define X25_INIT_CRC        0xFFFF
//#define MAVLINK_STX         0xFE

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

VAR_STATIC const Mavlink_Crc[] = MAVLINK_MESSAGE_CRCS ;
/*
#define MAVLINK_MESSAGE_CRCS {
50, 124, 137, 0, 237, 217, 104, 119,
0, 0, 0, 89, 0, 0, 0, 0,
0, 0, 0, 0, 214, 159, 220, 168,
24, 23, 170, 144, 67, 115, 39, 246,
185, 104, 237, 244, 222, 212, 9, 254,
230, 28, 28, 132, 221, 232, 11, 153,
41, 39, 214, 223, 141, 33, 15, 3,
100, 24, 239, 238, 30, 240, 183, 130,
130, 0, 148, 21, 0, 52, 124, 0,
0, 0, 20, 0, 152, 143, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 231, 183, 63, 54, 0, 0, 0,
0, 0, 0, 0, 175, 102, 158, 208,
56, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, 0, 0, 0, 0,
0, 204, 49, 170, 44, 83, 46, 0}
*/

/// Names of parameters
VAR_STATIC const uint8_t Parameter_Name[] =
    "ROLL KP\0\0\0\0\0\0\0\0\0"     ///<
    "ROLL KI\0\0\0\0\0\0\0\0\0"     ///<
    "PITCH KP\0\0\0\0\0\0\0\0"      ///<
    "PITCH KI\0\0\0\0\0\0\0\0"      ///<
    "ALT KP\0\0\0\0\0\0\0\0\0\0"    ///<
    "ALT KI\0\0\0\0\0\0\0\0\0\0"    ///<
    "NAV KP\0\0\0\0\0\0\0\0\0\0"    ///<
    "NAV KI\0\0\0\0\0\0\0\0\0\0"    ///<
;

VAR_STATIC const uint8_t Autopilot_Type = MAV_AUTOPILOT_GENERIC;
VAR_STATIC const uint8_t System_Type = MAV_TYPE_FIXED_WING;       // Define system type, airplane
VAR_STATIC const uint8_t System_ID = 20;
VAR_STATIC const uint8_t Component_ID = MAV_COMP_ID_IMU;

/*---------------------------------- Globals ---------------------------------*/

//VAR_GLOBAL xQueueHandle xTelemetry_Queue;
//VAR_GLOBAL struct global_struct global_data;

/*----------------------------------- Locals ---------------------------------*/

//VAR_STATIC uint8_t System_Mode = MAV_MODE_PREFLIGHT;  // Booting up
//VAR_STATIC uint8_t System_State = MAV_STATE_STANDBY;  // System ready for flight
//VAR_STATIC uint16_t mode = MAV_MODE_UNINIT;           // Defined in mavlink_types.h, which is included by mavlink.h
//VAR_STATIC uint32_t custom_mode;
//VAR_STATIC mavlink_system_t mavlink_system;
//VAR_STATIC mavlink_param_set_t set;
//VAR_STATIC mavlink_status_t status;
//VAR_STATIC mavlink_message_t msg;

VAR_STATIC uint8_t current_tx_seq = 0;
VAR_STATIC uint16_t packet_drops = 0;
VAR_STATIC uint8_t packet_rx_drop_count;
VAR_STATIC uint16_t m_parameter_i = ONBOARD_PARAM_COUNT;
VAR_STATIC uint8_t msgid;

VAR_STATIC uint8_t buf[MAVLINK_MAX_PACKET_LEN];
//VAR_STATIC uint8_t ucWindex;                        // uplink write index
//VAR_STATIC uint8_t ucRindex;                        // uplink read index

VAR_STATIC uint16_t Crc;

VAR_STATIC float fPIDGain[TEL_GAIN_NUMBER] = {
    ROLL_KP,                                        //!< default roll kp
    ROLL_KI,                                        //!< default roll ki
    PITCH_KP,                                       //!< default pitch kp
    PITCH_KI,                                       //!< default pitch ki
    0.0f,                                           //!< dummy placeholder
    0.0f,                                           //!< dummy placeholder
    NAV_KP,                                         //!< default direction kp
    NAV_KI                                          //!< default direction ki
};                                                  //!< gains for PID loops

/*--------------------------------- Prototypes -------------------------------*/

//static void communication_queued_send(void);
//static void communication_receive(void);

/*---------------------------------- Functions -------------------------------*/

///----------------------------------------------------------------------------
///
/// \brief   Initialize CRC
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
static __inline void Checksum_Init(void)
{
   Crc = X25_INIT_CRC;
}

///----------------------------------------------------------------------------
///
/// \brief   Accumulate one byte of data into the CRC
/// \return  -
/// \remarks -
///
///----------------------------------------------------------------------------
static __inline void Checksum_Accumulate(uint8_t data)
{
    uint8_t tmp;

    tmp = data ^ (uint8_t)(Crc & 0xFF);
    tmp ^= (tmp << 4);
    Crc = (Crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
}


//----------------------------------------------------------------------------
//
/// \brief   Send communication packets
/// \param   -
/// \returns -
/// \remarks
///   0          MAVLINK_STX Start Transmission   0xFE
///   1          len         Length               0 - 255
///   2          seq         Sequence             0 - 255
///   3          SYSID       System identifier    0 - 255
///   4          COMPID      Component identifier 0 - 255
///   5          MSGID       Message identifier   0 - 255
///
//----------------------------------------------------------------------------
static void Mavlink_Send( uint8_t crc_extra ) {
	uint16_t j;
    uint8_t length = buf[1];

    Checksum_Init();
    buf[0] = MAVLINK_STX;
	buf[2] = current_tx_seq++;          // One sequence number per component
	buf[3] = System_ID;
	buf[4] = Component_ID;

    for (j = 1; j < length + 6; j++) {
        Checksum_Accumulate(buf[j]);
    }
	Checksum_Accumulate(crc_extra);

    buf[j++] = (uint8_t)(Crc & 0xFF);
	buf[j] = (uint8_t)(Crc >> 8);
    for (j = 0; j < length + 8; j++) {
        USART1_Putch(buf[j]);
    }
    USART1_Transmit();
}

//----------------------------------------------------------------------------
//
/// \brief   Send heartbeat packet
/// \param   -
/// \returns -
/// \remarks
/// The heartbeat message shows that a system is present and responding.
///	The type of the MAV and Autopilot hardware allow the receiving system
/// to treat further messages from this system appropriate (e.g. by laying
/// out the user interface based on the autopilot).
/// MAVLINK_MSG_ID_HEARTBEAT
/// Field         Offset  Type    Meaning
/// ----------------------------------------------------------------------------
/// OSD mode         0   uint32_t Bitfield for use for autopilot-specific flags.
/// Heartbeat type   4   uint8_t  Type of the MAV (quadrotor, heli, see MAV_TYPE ENUM)
/// Base mode        6   uint8_t  System mode bitfield, see MAV_MODE_FLAGS ENUM
/// autopilot        ?   uint8_t  Autopilot type / class. see MAV_AUTOPILOT ENUM
/// system_status    ?   uint8_t  System status flag, see MAV_STATE ENUM
/// mavlink_version  ?   uint8_t_mavlink_version  MAVLink version, read only,
///                               gets added by protocol because of magic data type
///
//----------------------------------------------------------------------------
void Mavlink_Heartbeat( void ) {
	uint16_t j;

    for (j = 0; j < MAVLINK_MAX_PACKET_LEN; j++) {
        buf[j] = 0;
    }

	buf[1] = 9;                         // Payload length
	buf[5] = MAVLINK_MSG_ID_HEARTBEAT;  // Heartbeat message ID
	buf[10] = MAV_TYPE_FIXED_WING;      // Type of the MAV, defined in MAV_TYPE ENUM
    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_HEARTBEAT]);
}

//----------------------------------------------------------------------------
//
/// \brief   Send VFR HUD data
/// \param   -
/// \returns -
/// \remarks
/// Name = MAVLINK_MSG_ID_VFR_HUD, ID = 74, Length = 20
///
/// Field        Offset Type    Meaning
/// -------------------------------------
/// Airspeed       0    float
/// Ground speed   4    float
/// Altitude       8    float
/// Climb rate    12    float
/// Heading       16    int16_t
/// Throttle      18    uint16_t
///
//----------------------------------------------------------------------------
void Mavlink_Hud( void ) {

	buf[1] = 20;                                        // Payload length
	buf[5] = MAVLINK_MSG_ID_VFR_HUD;                    // VFR HUD message ID
    *((float *)(&buf[6])) = 0.0f;                       // Airspeed
    *((float *)(&buf[10])) = (float)Gps_Speed();        // GPS speed
    *((float *)(&buf[14])) = Nav_Altitude();            // Altitude
    *((float *)(&buf[18])) = 0.0f;                      // Climb rate
    *((int16_t *)(&buf[22])) = (int16_t)Nav_Heading();  // Heading
    *((uint16_t *)(&buf[24])) = Nav_Throttle();         // Throttle

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_VFR_HUD]);
}


//----------------------------------------------------------------------------
//
/// \brief   Send attitude data
/// \param   -
/// \returns -
/// \remarks
/// Name = MAVLINK_MSG_ID_ATTITUDE, ID = 30, Length = 28
///
/// Field        Offset Type     Meaning
/// -------------------------------------
/// time_boot_ms  0     uint32_t
/// roll          4     float
/// pitch         8     float
/// yaw          12     float
/// rollspeed    16     float
/// pitchspeed   20     float
/// yawspeed     24     float
///
//----------------------------------------------------------------------------
void Mavlink_Attitude( void ) {

	buf[1] = 28;                                    // Payload length
	buf[5] = MAVLINK_MSG_ID_ATTITUDE;               // Attitude message ID
    *((uint32_t *)(&buf[6])) = 0;                   // time from boot [ms]
    *((float *)(&buf[10])) = Attitude_Roll_Rad();   // roll
    *((float *)(&buf[14])) = Attitude_Pitch_Rad();  // pitch
    *((float *)(&buf[18])) = Attitude_Yaw_Rad();    // yaw
    *((float *)(&buf[22])) = 0.0f;                  // roll rate
    *((float *)(&buf[26])) = 0.0f;                  // pitch rate
    *((float *)(&buf[30])) = 0.0f;                  // yaw rate

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_ATTITUDE]);
}

//----------------------------------------------------------------------------
//
/// \brief   Send GPS data
/// \param   -
/// \returns -
/// \remarks
/// Name = MAVLINK_MSG_ID_GPS_RAW_INT, ID = 24, Length = 30
///
/// Field             Offset Type     Meaning
/// ------------------------------------------
/// time_usec            0   uint64_t
/// lat                  8   int32_t
/// lon                 12   int32_t
/// alt                 16   int32_t
/// eph                 20   uint16_t
/// epv                 22   uint16_t
/// vel                 24   uint16_t
/// cog                 26   uint16_t
/// fix_type            28   uint8_t
/// satellites_visible  29   uint8_t
///
//----------------------------------------------------------------------------
void Mavlink_Gps( void ) {

    buf[1] = 30;                                            // Payload length
    buf[5] = MAVLINK_MSG_ID_GPS_RAW_INT;                    // GPS message ID
    *((uint64_t *)(&buf[6])) = 0;                           // time from boot [us]
    *((int32_t *)(&buf[14])) = Gps_Latitude();              // latitude
    *((int32_t *)(&buf[18])) = Gps_Longitude();             // longitude
    *((uint16_t *)(&buf[26])) = (uint16_t)Nav_Altitude();   // altitude
    *((uint16_t *)(&buf[28])) = 65535;                      // epv
    *((uint16_t *)(&buf[30])) = Gps_Speed();                // velocity
    *((uint16_t *)(&buf[32])) = Gps_Heading();              // course over ground
    buf[34] = 3;                                            // fix type
    buf[35] = 255;                                          // satellites

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_GPS_RAW_INT]);
}

//----------------------------------------------------------------------------
//
/// \brief   Send parameter value
/// \param   -
/// \returns -
/// \remarks
/// Name = MAVLINK_MSG_ID_PARAM_VALUE, ID = 22, Length = 25
///
/// Field       Offset Type    Meaning
/// -------------------------------------
/// param_value   0    float     Onboard parameter value
/// param_count   4    uint16_t  Total number of onboard parameters
/// param_index   6    uint16_t  Index of this onboard parameter
/// param_id      8    array     Onboard parameter name, null terminated ...
/// param_type    24   uint8_t   Onboard parameter type: see MAVLINK_TYPE enum
///
//----------------------------------------------------------------------------
void Mavlink_Param_Value( uint16_t param_index ) {

    uint8_t j;

	buf[1] = 25;                                    // Payload length
	buf[5] = MAVLINK_MSG_ID_PARAM_VALUE;            // Parameter value message ID
    *((float *)(&buf[6])) = fPIDGain[param_index];  // Parameter value
    *((uint16_t *)(&buf[10])) = ONBOARD_PARAM_COUNT;// Total number of parameters
    *((uint16_t *)(&buf[12])) = param_index;        // Parameter index
    for (j = 0; j < 16; j++) {
        buf[j + 14] = Parameter_Name[j + param_index * 16]; // Parameter name
    }
    buf[30] = MAVLINK_TYPE_FLOAT;                   // Parameter type

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_PARAM_VALUE]);
}


//----------------------------------------------------------------------------
//
/// \brief   Parse communication packets
/// \param   -
/// \returns -
/// \remarks This function decodes packets on the protocol level.
///
//----------------------------------------------------------------------------
static bool Mavlink_Parse(void) {

    uint8_t c;
    static uint8_t magic;
    static uint8_t sysid;
    static uint8_t compid;
    static mavlink_parse_state_t parse_state = MAVLINK_PARSE_STATE_UNINIT;
    static uint8_t len;
    static uint8_t buffer_overrun;
    static uint8_t parse_error;
    static uint8_t packet_idx;
    static uint8_t seq;
    static uint8_t current_rx_seq;
    static uint8_t packet_rx_success_count;

    bool msg_received = FALSE;

    while (USART1_Getch (&c)) {               // received another character
	switch (parse_state) {
        case MAVLINK_PARSE_STATE_UNINIT:
        case MAVLINK_PARSE_STATE_IDLE:
            if (c == MAVLINK_STX) {
                parse_state = MAVLINK_PARSE_STATE_GOT_STX;
                len = 0;
                magic = c;
                Checksum_Init();
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_STX:
            if (msg_received || (c > MAVLINK_MAX_PAYLOAD_LEN)) {
                buffer_overrun++;
                parse_error++;
                msg_received = FALSE;
                parse_state = MAVLINK_PARSE_STATE_IDLE;
            } else {
                len = c; // NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
                packet_idx = 0;
                Checksum_Accumulate(c);
                parse_state = MAVLINK_PARSE_STATE_GOT_LENGTH;
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_LENGTH:
            seq = c;
            Checksum_Accumulate(c);
            parse_state = MAVLINK_PARSE_STATE_GOT_SEQ;
            break;

        case MAVLINK_PARSE_STATE_GOT_SEQ:
            sysid = c;
            Checksum_Accumulate(c);
            parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
            break;

        case MAVLINK_PARSE_STATE_GOT_SYSID:
            compid = c;
            Checksum_Accumulate(c);
            parse_state = MAVLINK_PARSE_STATE_GOT_COMPID;
            break;

        case MAVLINK_PARSE_STATE_GOT_COMPID:
            msgid = c;
            Checksum_Accumulate(c);
            if (len == 0) {
                parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
            } else {
                parse_state = MAVLINK_PARSE_STATE_GOT_MSGID;
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_MSGID:
    //		_MAV_PAYLOAD_NON_CONST(rxmsg)[packet_idx++] = (char)c;
	        packet_idx++;
            Checksum_Accumulate(c);
            if (packet_idx == len) {
                parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
            Checksum_Accumulate(Mavlink_Crc[msgid]);
            if (c != (Crc & 0xFF)) { // Check first checksum byte
                parse_error++;
                msg_received = FALSE;
                parse_state = MAVLINK_PARSE_STATE_IDLE;
                if (c == MAVLINK_STX) {
                    parse_state = MAVLINK_PARSE_STATE_GOT_STX;
                    len = 0;
                    Checksum_Init();
                }
            } else {
                parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
    //			_MAV_PAYLOAD_NON_CONST(rxmsg)[packet_idx] = (char)c;
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_CRC1:
            if (c != (Crc >> 8)) {	// Check second checksum byte
                parse_error++;
                msg_received = FALSE;
                parse_state = MAVLINK_PARSE_STATE_IDLE;
                if (c == MAVLINK_STX) {
                    parse_state = MAVLINK_PARSE_STATE_GOT_STX;
                    len = 0;
                    Checksum_Init();
                }
            } else {		        // Successfully got message
                msg_received = TRUE;
                parse_state = MAVLINK_PARSE_STATE_IDLE;
    //			_MAV_PAYLOAD_NON_CONST(rxmsg)[packet_idx+1] = (char)c;
    //			memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
            }
            break;
        }

        // If a message has been sucessfully decoded, check index
        if (msg_received == 1) {
            current_rx_seq = seq;
            // Initial condition: If no packet has been received so far, drop count is undefined
            if (packet_rx_success_count == 0) packet_rx_drop_count = 0;
            // Count this packet as received
            packet_rx_success_count++;
        }

        current_rx_seq = current_rx_seq + 1;
        packet_rx_drop_count = parse_error;
        parse_error = 0;
	}
    return (msg_received);
}

//----------------------------------------------------------------------------
//
/// \brief   Handles packets
/// \param   -
/// \returns -
/// \remarks Handles packet value by calling the appropriate functions.
///
//----------------------------------------------------------------------------
void Mavlink_Receive(void)
{
    if (Mavlink_Parse()) {                          // Received a correct packet
        switch (msgid) {                            // Handle message
            case MAVLINK_MSG_ID_HEARTBEAT:          // E.g. read GCS heartbeat and go into comm lost mode if timer times out
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:	    //
                break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //
                m_parameter_i = 0;
                break;
            case MAVLINK_MSG_ID_PARAM_SET:          //
                break;
            case MAVLINK_MSG_ID_PARAM_VALUE:        //
                break;
            default:				                // Do nothing
                break;
        }
    }		                                        //
	packet_drops += packet_rx_drop_count;           // Update global packet drops counter
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
void Mavlink_Queued_Send(void)
{
    if (m_parameter_i < ONBOARD_PARAM_COUNT) {  //send parameters one by one
        Mavlink_Param_Value(m_parameter_i++);
    } else {
        Mavlink_Heartbeat();
    }
}


/*
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
                    if (match)                                              // Check if matched
                    {
                        if ((global_data.param[i] != set.param_value) &&    // Write and emit changes if there is a difference
                            !isnan(set.param_value) &&                      // AND if new value is NOT "not-a-number"
                            !isinf(set.param_value) &&                      // AND is NOT infinity
                            (set.param_type == MAVLINK_TYPE_FLOAT))
                        {
                            global_data.param[i] = set.param_value;
                            mavlink_msg_param_value_send(MAVLINK_COMM_0,    // Report back new value
                                                        (int8_t*) global_data.param_name[i],
                                                        global_data.param[i], MAVLINK_TYPE_FLOAT,
                                                        ONBOARD_PARAM_COUNT,
														m_parameter_i);
                        }
                    }
                }
            }
            break;
        default:
            break;
    }
}
*/

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


