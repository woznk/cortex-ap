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
/// \code
/// Message identifier (MSGID)           Value  Length Implemented
/// ---------------------------------------------------------------
/// MAVLINK_MSG_ID_HEARTBEAT                0      9        Y
/// MAVLINK_MSG_ID_SYS_STATUS               1     31
/// MAVLINK_MSG_PARAM_REQUEST_LIST         21      2
/// MAVLINK_MSG_ID_GPS_RAW_INT             24     30
/// MAVLINK_MSG_ID_VFR_HUD                 74     20        Y
/// MAVLINK_MSG_ID_ATTITUDE                30     28        Y
/// MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT   62     26
/// MAVLINK_MSG_ID_MISSION_CURRENT         42      2
/// MAVLINK_MSG_ID_RC_CHANNELS_RAW         35     22
/// MISSION_COUNT                          44      4
/// MAVLINK_MSG_REQUEST_DATA_STREAM        66      6
/// MAVLINK_MSG_ID_WIND                   168     12
/// \endcode
///
/// --------------------- Mavlink message contents --------------------
/// \code
/// Identifier (MSGID)                   Content           Offset  Type    Note
/// -----------------------------------------------------------------------
/// MAVLINK_MSG_ID_HEARTBEAT             Heartbeat type       4   uint8_t  Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
///                                      OSD mode             0   uint32_t Bitfield for use for autopilot-specific flags.
///                                      Base mode            6   uint8_t  System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
///                                      autopilot            ?   uint8_t  Autopilot type / class. defined in MAV_AUTOPILOT ENUM
///                                      system_status        ?   uint8_t  System status flag, see MAV_STATE ENUM
///                                      mavlink_version      ?   uint8_t_mavlink_version  MAVLink version, not writable by user, gets added by protocol because of magic data type
/// The heartbeat message shows that a system is present and responding.
///	The type of the MAV and Autopilot hardware allow the receiving system
/// to treat further messages from this system appropriate (e.g. by laying
/// out the user interface based on the autopilot).
///
/// MAVLINK_MSG_ID_SYS_STATUS            Battery voltage     14   uint16_t
///                                      Battery current     16   int16_t
///                                      Battery remaining   30   int8_t
///
/// MAVLINK_MSG_ID_PARAM_REQUEST_LIST    Target System        ?   uint8_t
///                                      System ID            ?            Request all parameters of this component.
///                                      Target Component     ?   uint8_t  Component ID
/// After his request, all parameters are emitted.
///
/// MAVLINK_MSG_ID_GPS_RAW_INT           time_usec            0   uint64_t
///                                      lat                  8   int32_t
///                                      lon                 12   int32_t
///                                      alt                 16   int32_t
///                                      eph                 20   uint16_t
///                                      epv                 22   uint16_t
///                                      vel                 24   uint16_t
///                                      cog                 26   uint16_t
///                                      fix_type            28   uint8_t
///                                      satellites_visible  29   uint8_t
///
/// MAVLINK_MSG_ID_VFR_HUD               Airspeed             0   float
///                                      Ground speed         4   float
///                                      Altitude             8   float
///                                      Climb rate          12   float
///                                      Heading             16   int16_t
///                                      Throttle            18   uint16_t
///
/// MAVLINK_MSG_ID_ATTITUDE              Timestamp            0   uint32_t Time since system boot [ms]
///                                      Roll                 4   float    Roll angle [rad]
///                                      Pitch                8   float    Pitch angle [rad]
///                                      Yaw                 12   float    Yaw angle [rad]
///                                      Roll speed          16   float    Roll angular speed [rad/s]
///                                      Pitch speed         20   float    Pitch angular speed [rad/s]
///                                      Yaw speed           24   float    Yaw angular speed [rad/s]
///
/// MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT Nav roll             0   float
///                                      Nav pitch            4   float
///                                      Altitude error       8   float
///                                      Airspeed error      12   float
///                                      Crosstrack error    16   float
///                                      Nav bearing         20   int16_t
///                                      Target bearing      22   int16_t
///                                      Waypoint distance   24   uint16_t
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
/// MAVLINK_MSG_ID_MISSION_COUNT         target_system        ?   uint8_t  System ID
///                                      target_component     ?   uint8_t  Component ID
///                                      count                ?   uint16_t Number of mission items in the sequence
/// This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction.
/// The GCS can then request the individual mission item based on the knowledge of the total number of MISSIONs.
///
/// MAVLINK_MSG_ID_REQUEST_DATA_STREAM   target_system        ?   uint8_t  The target requested to send the message stream.
///                                      target_component     ?   uint8_t  The target requested to send the message stream.
///                                      req_stream_id        ?   uint8_t  The ID of the requested data stream
///                                      req_message_rate     ?   uint16_t The requested interval between two messages of this type
///                                      start_stop           ?   uint8_t  1 to start sending, 0 to stop sending.
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
///--------------------- AqGCS messages taxonomy ------------------------
///
/// CONNECT      PID          PID EXIT     ALL PARAM    ALL PARAM EXIT
/// ----------------------------------------------------------------------
/// FE STX       FE STX       FE STX       FE STX       FE STX
/// 09 Length    02 Length    06 Length    02 Length    06 Length
/// 07 Sequence  08 Sequence  09 Sequence  0A Sequence  0B Sequence
/// FF SYSID     FF SYSID     FF SYSID     FF SYSID     FF SYSID
/// 00 COMPID    00 COMPID    00 COMPID    00 COMPID    00 COMPID
/// 00 MSGID     15 MSGID     42 MSGID     15 MSGID     42 MSGID
/// 00 Payload   00 Payload   00 Payload   00 Payload   00 Payload
/// 00 Payload   00 Payload   00 Payload   00 Payload   00 Payload
/// 00 Payload   AD CRC 1     00 Payload   16 CRC 1     00 Payload
/// 00 Payload   95 CRC 2     00 Payload   A2 CRC 2     00 Payload
/// 06 Payload                00 Payload                00 Payload
/// 00 Payload                00 Payload                00 Payload
/// 00 Payload                52 CRC 1                  70 CRC 1
/// 00 Payload                ED CRC 2                  46 CRC 2
/// 03 Payload
/// 06 CRC 1
/// F5 CRC 2
///
/// HUD             HUD EXIT     GPS          GPS EXIT     SAVE MISSION
/// ----------------------------------------------------------------------
/// FE FE STX       FE STX       FE STX       FE STX       FE STX
/// 06 06 Length    06 Length    06 Length    06 Length    04 Length
/// 0C 0D Sequence  03 Sequence  04 Sequence  05 Sequence  06 Sequence
/// FF FF SYSID     FF SYSID     FF SYSID     FF SYSID     FF SYSID
/// 00 00 COMPID    00 COMPID    00 COMPID    00 COMPID    00 COMPID
/// 42 42 MSGID     42 MSGID     42 MSGID     42 MSGID     2C MSGID
/// 14 01 Payload   00 Payload   01 Payload   00 Payload   04 Payload
/// 00 00 Payload   00 Payload   00 Payload   00 Payload   00 Payload
/// 00 00 Payload   00 Payload   00 Payload   00 Payload   00 Payload
/// 00 00 Payload   00 Payload   00 Payload   00 Payload   BE Payload
/// 0A 02 Payload   00 Payload   02 Payload   00 Payload   BF CRC 1
/// 01 01 Payload   00 Payload   01 Payload   00 Payload   4E CRC 2
/// 8B B2 CRC 1     DA CRC 1     89 CRC 1     AD CRC 1
/// 56 80 CRC 2     FA CRC 2     69 CRC 2     0F CRC 2
///
/// MISSION EXIT TX EXIT      DISCONNECT
/// -----------------------------------------
/// FE STX       FE STX       FE STX
/// 06 Length    06 Length    06 Length
/// 07 Sequence  08 Sequence  06 Sequence
/// FF SYSID     FF SYSID     FF SYSID
/// 00 COMPID    00 COMPID    00 COMPID
/// 42 MSGID     42 MSGID     42 MSGID
/// 00 Payload   00 Payload   00 Payload
/// 00 Payload   00 Payload   00 Payload
/// 00 Payload   00 Payload   00 Payload
/// 00 Payload   00 Payload   00 Payload
/// 00 Payload   00 Payload   00 Payload
/// 00 Payload   00 Payload   00 Payload
/// 8F CRC 1     C3 CRC 1     1E CRC 1
/// A4 CRC 2     B8 CRC 2     F1 CRC 2
///
///--------------------- aq-gcs2 messages taxonomy ------------------------
///
/// READ FROM ROM SAVE TO ROM  PID          SAVE MISSION RETRIEVE MISSION
/// FE STX        FE STX       FE STX       FE STX       FE STX
/// 21 Length     21 Length    02 Length    02 Length    02 Length
/// 01 Sequence   02 Sequence  00 Sequence  03 Sequence  00 Sequence
/// FF SYSID      FF SYSID     FF SYSID     FF SYSID     FF SYSID
/// 00 COMPID     00 COMPID    00 COMPID    00 COMPID    00 COMPID
/// 4C MSGID      4C MSGID     15 MSGID     2D MSGID     2B MSGID
/// 00 Payload    00 Payload   00 Payload   00 Payload   00 Payload
/// 00 Payload    00 Payload   00 Payload   BE Payload   00 Payload
/// 00 Payload    80 Payload   41 CRC 1     43 CRC 1     A3 CRC 1
/// 00 Payload    3F Payload   4B CRC 2     ED CRC 2     07 CRC 2
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// F5 Payload    F5 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 00 Payload    00 Payload
/// 01 Payload    01 Payload
/// 8D CRC 1      A2 CRC 1
/// 8E CRC 2      1D CRC 2
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
/// Changes: added Mavlink_Gps function, corrected Mavlink_Attitude
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

#define ONBOARD_PARAM_COUNT 5
#define ONBOARD_PARAM_NAME_LENGTH 8
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

/*---------------------------------- Globals ---------------------------------*/

//VAR_GLOBAL xQueueHandle xTelemetry_Queue;
//VAR_GLOBAL struct global_struct global_data;

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC uint8_t System_Mode = MAV_MODE_PREFLIGHT;        // Booting up
VAR_STATIC uint8_t System_State = MAV_STATE_STANDBY;        // System ready for flight
VAR_STATIC uint8_t Autopilot_Type = MAV_AUTOPILOT_GENERIC;
VAR_STATIC uint8_t System_Type = MAV_TYPE_FIXED_WING;       // Define system type, airplane
VAR_STATIC uint8_t System_ID = 20;
VAR_STATIC uint8_t Component_ID = MAV_COMP_ID_IMU;

VAR_STATIC uint8_t Current_Tx_Seq = 0;
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

VAR_STATIC uint16_t Crc;

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
/// \brief   Receive communication packets and handle them
/// \param   -
/// \returns -
/// \remarks This function decodes packets on the protocol level and also
///          handles their value by calling the appropriate functions.
///
//----------------------------------------------------------------------------
void Mavlink_Receive(void) {
    uint8_t c;
    static mavlink_parse_state_t parse_state = MAVLINK_PARSE_STATE_UNINIT;
    static uint8_t len;
    static uint8_t magic;
    static uint8_t buffer_overrun;
    static uint8_t parse_error;
    static uint8_t msg_received;
    static uint8_t packet_idx;
    static uint8_t seq;
    static uint8_t sysid;
    static uint8_t compid;
    static uint8_t msgid;
    static uint8_t current_rx_seq;
    static uint8_t packet_rx_success_count;
    static uint8_t packet_rx_drop_count;

	msg_received = 0;

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
                msg_received = 0;
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
                msg_received = 0;
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
                msg_received = 0;
                parse_state = MAVLINK_PARSE_STATE_IDLE;
                if (c == MAVLINK_STX) {
                    parse_state = MAVLINK_PARSE_STATE_GOT_STX;
                    len = 0;
                    Checksum_Init();
                }
            } else {		// Successfully got message
                msg_received = 1;
                parse_state = MAVLINK_PARSE_STATE_IDLE;
    //			_MAV_PAYLOAD_NON_CONST(rxmsg)[packet_idx+1] = (char)c;
    //			memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
            }
            break;
        }

	// If a message has been sucessfully decoded, check index
	if (msg_received == 1) {
        Mavlink_Heartbeat();
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
	buf[2] = Current_Tx_Seq++;          // One sequence number per component
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
/// \remarks MAVLINK_MSG_ID_HEARTBEAT
///     Field         Offset  Type    Meaning
///     OSD mode         0   uint32_t Bitfield for use for autopilot-specific flags.
///     Heartbeat type   4   uint8_t  Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
///     Base mode        6   uint8_t  System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
///     autopilot        ?   uint8_t  Autopilot type / class. defined in MAV_AUTOPILOT ENUM
///     system_status    ?   uint8_t  System status flag, see MAV_STATE ENUM
///     mavlink_version  ?   uint8_t_mavlink_version  MAVLink version, not writable by user, gets added by protocol because of magic data type
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
/// \remarks Name = MAVLINK_MSG_ID_VFR_HUD, ID = 74, Length = 20
///     Field        Offset Type    Meaning
///     Airspeed       0    float
///     Ground speed   4    float
///     Altitude       8    float
///     Climb rate    12    float
///     Heading       16    int16_t
///     Throttle      18    uint16_t
///
//----------------------------------------------------------------------------
void Mavlink_Hud( void ) {
	uint16_t j;

    for (j = 0; j < MAVLINK_MAX_PACKET_LEN; j++) {
        buf[j] = 0;
    }

	buf[1] = 20;                                 // Payload length
	buf[5] = MAVLINK_MSG_ID_VFR_HUD;             // VFR HUD message ID
//    *((float *)(&buf[6])) = 0.0f;                // Airspeed
    *((float *)(&buf[10])) = (float)Gps_Speed(); // GPS speed
    *((float *)(&buf[14])) = Nav_Altitude();     // Altitude
//    *((float *)(&buf[18])) = 0.0f;               // Climb rate
    *((int16_t *)(&buf[22])) = Gps_Heading();    // Heading
    *((uint16_t *)(&buf[24])) = Nav_Throttle();  // Throttle

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_VFR_HUD]);
}


//----------------------------------------------------------------------------
//
/// \brief   Send attitude data
/// \param   -
/// \returns -
/// \remarks Name = MAVLINK_MSG_ID_ATTITUDE, ID = 30, Length = 28
///     Field        Offset Type     Meaning
///     time_boot_ms  0     uint32_t
///     roll          4     float
///     pitch         8     float
///     yaw          12     float
///     rollspeed    16     float
///     pitchspeed   20     float
///     yawspeed     24     float
///
//----------------------------------------------------------------------------
void Mavlink_Attitude( void ) {
	uint16_t j;

    for (j = 0; j < MAVLINK_MAX_PACKET_LEN; j++) {
        buf[j] = 0;
    }

	buf[1] = 28;                                        // Payload length
	buf[5] = MAVLINK_MSG_ID_ATTITUDE;                   // Attitude message ID
//    *((uint32_t *)(&buf[6])) = time;                    // time from boot [ms]
    *((float *)(&buf[10])) = ToRad(Attitude_Roll());    //
    *((float *)(&buf[14])) = ToRad(Attitude_Pitch());   //
    *((float *)(&buf[18])) = ToRad(Attitude_Yaw());     //

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_ATTITUDE]);
}

//----------------------------------------------------------------------------
//
/// \brief   Send GPS data
/// \param   -
/// \returns -
/// \remarks Name = MAVLINK_MSG_ID_GPS_RAW_INT, ID = 24, Length = 30
///     Field             Offset Type     Meaning
///     time_usec            0   uint64_t
///     lat                  8   int32_t
///     lon                 12   int32_t
///     alt                 16   int32_t
///     eph                 20   uint16_t
///     epv                 22   uint16_t
///     vel                 24   uint16_t
///     cog                 26   uint16_t
///     fix_type            28   uint8_t
///     satellites_visible  29   uint8_t
///
//----------------------------------------------------------------------------
void Mavlink_Gps( void ) {
    uint16_t j;

    for (j = 0; j < MAVLINK_MAX_PACKET_LEN; j++) {
        buf[j] = 0;
    }

    buf[1] = 30;                                // Payload length
    buf[5] = MAVLINK_MSG_ID_GPS_RAW_INT;        // GPS message ID
//    *((uint64_t *)(&buf[6])) = time;          // time from boot [us]
    *((int32_t *)(&buf[14])) = Gps_Latitude();  //
    *((int32_t *)(&buf[18])) = Gps_Longitude(); //
    *((uint16_t *)(&buf[26])) = 65535;          //
    *((uint16_t *)(&buf[28])) = 65535;          //
    *((uint16_t *)(&buf[30])) = Gps_Speed();    //
    *((uint16_t *)(&buf[32])) = 65535;          //
    buf[34] = 3;                                // fix type
    buf[35] = 255;                              // satellites

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_GPS_RAW_INT]);
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
                                    (int8_t*) global_data.param_name[m_parameter_i],
                                    global_data.param[m_parameter_i],
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

    while (USART1_Getch (&c)) {               // received another character
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			switch (msg.msgid) {              // Handle message
                case MAVLINK_MSG_ID_HEARTBEAT: {
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


