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
/// PARAM_VALUE             22     25   Verified
/// PARAM_SET               23     23   Verified
/// GPS_RAW_INT             24     30   Verified
/// ATTITUDE                30     28   Verified
/// GLOBAL_POSITION_INT     33     28   Verified
/// RC_CHANNELS_RAW         35     22
/// MISSION_CURRENT         42      2
/// MISSION_REQUEST_LIST    43      2   Verified
/// MISSION_COUNT           44      4   Verified
/// MISSION_CLEAR_ALL       45      2
/// NAV_CONTROLLER_OUTPUT   62     26
/// REQUEST_DATA_STREAM     66      6
/// VFR_HUD                 74     20   Verified
/// COMMAND_LONG            76     21?
/// HIL_STATE               90     56
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
/// PARAM_REQUEST_LIST    see code
///
/// PARAM_SET             see code
///
/// PARAM_VALUE           see code
///
/// GPS_RAW_INT           see code
///
/// VFR_HUD               see code
///
/// ATTITUDE              see code
///
/// REQUEST_DATA_STREAM   see code
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
/// MISSION_CLEAR_ALL     target_system        ?   uint8_t  System ID
///                       target_component	   ?   uint8_t  Component ID
///
/// MISSION_REQUEST_LIST  target_system	       ?   uint8_t  System ID
///                       target_component     ?   uint8_t  Component ID
///
/// MISSION_COUNT         target_system        ?   uint8_t  System ID
///                       target_component     ?   uint8_t  Component ID
///                       count                ?   uint16_t Number of mission items in the sequence
///                                                 This message is emitted as response to MISSION_REQUEST_LIST
///                                                 by the MAV and to initiate a write transaction.
///                                                 The GCS can then request the individual mission item
///                                                 based on the knowledge of the total number of MISSIONs.
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
///                                                         Send a command with up to four parameters to the MAV
///
/// HIL_STATE             time_usec            0   uint64_t Timestamp [microseconds]
///                       roll                 8   float    Roll angle [rad]
///                       pitch               12   float    Pitch angle [rad]
///                       yaw                 16   float    Yaw angle [rad]
///                       rollspeed           20   float    Roll angular speed [rad/s]
///                       pitchspeed          24   float    Pitch angular speed [rad/s]
///                       yawspeed            28   float    Yaw angular speed [rad/s]
///                       lat                 32   int32_t  Latitude [deg * 1E7]
///                       lon                 36   int32_t  Longitude [deg * 1E7]
///                       alt                 40   int32_t  Altitude [mm]
///                       vx                  44   int16_t  Ground X Speed (Latitude) [m/s * 100]
///                       vy                  46   int16_t  Ground Y Speed (Longitude) [m/s * 100]
///                       vz                  48   int16_t  Ground Z Speed (Altitude) [m/s * 100]
///                       xacc                50   int16_t  X acceleration [mg]
///                       yacc                52   int16_t  Y acceleration [mg]
///                       zacc                54   int16_t  Z acceleration [mg]
///
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
///--------------------- Copter GCS messages taxonomy ------------------------
///
/// Menu            Message              ID Len  Payload
/// -----------------------------------------------------------------------
///                                             rate sys comp stream cmd   note
/// Setup           param request list   15  2
/// PID save        param set            17  23
/// Setup exit      request data stream  66  6   0   20   0     0    stop   all
/// HUD             request data stream  66  6   14  20   0     10   start  extra 1
///                 request data stream  66  6   1   20   0     2    start  ext status
/// HUD exit        application terminates
/// Status          request data stream  66  6   1   20   0     2    start  ext status
/// Status exit     request data stream  66  6   0   20   0     0    stop   all
/// GPS position    request data stream  66  6   1   20   0     2    start  ext status
/// GPS exit        request data stream  66  6   0   20   0     0    stop   all
/// Mode            param request list   15  2       20   0
///                 request data stream  66  6   5   20   0     3    start  RC channels
/// Mode exit       request data stream  66  6   0   20   0     0    stop   all
/// Readouts        request data stream  66  6   14  20   0     1    start  sensors
/// Readouts exit   application terminates
/// Mission         request data stream  66  6   1   20   0     2    start  ext status
/// Mission exit    request data stream  66  6   0   20   0     0    stop   all
/// Mav params      param request list   15  2       20   0
/// Mav params exit request data stream  66  6   0   20   0     0    stop   all
///
///
///--------------------- APM messages taxonomy ------------------------
///
/// Menu            Message              ID Len
/// --------------------------------------------
/// Refresh param   param request list   15  2
/// (arduplane PIDs)
///
///--------------------- ArduCAM OSD messages taxonomy ------------------------
///
/// Message              ID Len         Payload
/// ----------------------------------------------------------
/// request data stream  42  6
///                             rate  sys  comp stream   cmd    note
///          "                    2    14   C8     1    start   sensors
///          "                    2    14   C8     2    start   ext status
///          "                    5    14   C8     3    start   RC channels
///          "                    2    14   C8     6    start   position
///          "                    5    14   C8    10    start   extra 1 (attiude)
///          "                    2    14   C8    11    start   extra 2 (VFR HUD)
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
/// Change: system ID and component ID set to 1 (still some issue with 
///         droidplanner; will be fixed in future releases, according user forum),
///         corrected offsets of System_ID and Component_ID in the received
//          buffer for message of MAVLINK_MSG_ID_MISSION_ITEM message.
///
//============================================================================*/

#include "stm32f10x.h"
#include "usart1driver.h"
#include "math.h"

#include "config.h"
#include "nav.h"
#include "attitude.h"
#include "mav_telemetry.h"

/*--------------------------------- Definitions ------------------------------*/

#ifndef VAR_STATIC
#define VAR_STATIC static
#endif

#define MAX_STREAM_RATE             50

#define ONBOARD_PARAM_COUNT         ((uint16_t)TEL_GAIN_NUMBER)
#define ONBOARD_PARAM_NAME_LENGTH   16

//#define PACKET_LEN                MAVLINK_MAX_PACKET_LEN  // original length
#define PACKET_LEN                  64                      // reduced because of RAM constraints
#define PAYLOAD_LEN                 64

                                                // Origin
#define MAVLINK_MSG_ID_HEARTBEAT            0   // mavlink\common\mavlink_msg_heartbeat.h
#define MAVLINK_MSG_ID_VFR_HUD              74  // mavlink\common\mavlink_msg_vfr_hud.h
#define MAVLINK_MSG_ID_ATTITUDE             30  // mavlink\common\mavlink_msg_attitude.h
#define MAVLINK_MSG_ID_GPS_RAW_INT          24  // mavlink\common\mavlink_msg_gps_raw_int.h
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT  33  // mavlink\common\mavlink_msg_global_position_int.h
#define MAVLINK_MSG_ID_PARAM_VALUE          22  // mavlink\common\mavlink_msg_param_value.h
#define MAVLINK_MSG_ID_MISSION_COUNT        44  // mavlink\common\mavlink_msg_mission_count.h
#define MAVLINK_MSG_ID_MISSION_ITEM         39  // mavlink\common\mavlink_msg_mission_item.h
#define MAVLINK_MSG_ID_REQUEST_DATA_STREAM  66  // mavlink\common\mavlink_msg_request_data_stream.h
#define MAVLINK_MSG_ID_COMMAND_LONG         76  // mavlink\common\mavlink_msg_command_long.h
#define MAVLINK_MSG_ID_PARAM_REQUEST_LIST   21  // mavlink\common\mavlink_msg_param_request_list.h
#define MAVLINK_MSG_ID_PARAM_SET            23  // mavlink\common\mavlink_msg_param_set.h
#define MAVLINK_MSG_ID_HIL_STATE            90  // mavlink\common\mavlink_msg_hil_state.h
#define MAVLINK_MSG_ID_MISSION_REQUEST_LIST 43  // mavlink\common\mavlink_msg_mission_request_list.h
#define MAVLINK_MSG_ID_MISSION_REQUEST      40  // mavlink\common\mavlink_msg_mission_request.h

#define X25_INIT_CRC        0xFFFF              // mavlink\checksum.h
//#define X25_VALIDATE_CRC  0xF0B8              // mavlink\matrixpilot\mavlink.h

#define MAVLINK_STX         0xFE                // mavlink\matrixpilot\mavlink.h
                                                // mavlink\matrixpilot\matrixpilot.h
#define MAVLINK_MESSAGE_CRCS {          \
 50, 124, 137,   0, 237, 217, 104, 119, \
  0,   0,   0,  89,   0,   0,   0,   0, \
  0,   0,   0,   0, 214, 159, 220, 168, \
 24,  23, 170, 144,  67, 115,  39, 246, \
185, 104, 237, 244, 222, 212,   9, 254, \
230,  28,  28, 132, 221, 232,  11, 153, \
 41,  39, 214, 223, 141,  33,  15,   3, \
100,  24, 239, 238,  30, 240, 183, 130, \
130,   0, 148,  21,   0,  52, 124,   0, \
  0,   0,  20,   0, 152, 143,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0, 231, 183,  63,  54,   0,   0,   0, \
  0,   0,   0,   0, 175, 102, 158, 208, \
 56,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0,   0,   0,   0,   0,   0,   0,   0, \
  0, 204,  49, 170,  44,  83,  46,   0 }

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*lint -e753 -e749 -e751 */
enum MAV_AUTOPILOT {
	MAV_AUTOPILOT_GENERIC=0,        /* Generic autopilot, full support for everything */
	MAV_AUTOPILOT_PIXHAWK=1,        /* PIXHAWK autopilot, http://pixhawk.ethz.ch */
	MAV_AUTOPILOT_SLUGS=2,          /* SLUGS autopilot, http://slugsuav.soe.ucsc.edu */
	MAV_AUTOPILOT_ARDUPILOTMEGA=3,  /* ArduPilotMega / ArduCopter, http://diydrones.com */
	MAV_AUTOPILOT_OPENPILOT=4,      /* OpenPilot, http://openpilot.org */
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY=5, /* Generic autopilot only supporting simple waypoints */
	MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY=6, /* Generic autopilot supporting waypoints and other simple navigation commands */
	MAV_AUTOPILOT_GENERIC_MISSION_FULL=7, /* Generic autopilot supporting the full mission command set */
	MAV_AUTOPILOT_INVALID=8,        /* No valid autopilot, e.g. a GCS or other MAVLink component */
	MAV_AUTOPILOT_PPZ=9,            /* PPZ UAV - http://nongnu.org/paparazzi */
	MAV_AUTOPILOT_UDB=10,           /* UAV Dev Board */
	MAV_AUTOPILOT_FP=11,            /* FlexiPilot */
	MAV_AUTOPILOT_ENUM_END=12       /*  */
};

enum MAV_TYPE {
	MAV_TYPE_GENERIC=0,             /* Generic micro air vehicle. */
	MAV_TYPE_FIXED_WING=1,          /* Fixed wing aircraft. */
	MAV_TYPE_QUADROTOR=2,           /* Quadrotor */
	MAV_TYPE_COAXIAL=3,             /* Coaxial helicopter */
	MAV_TYPE_HELICOPTER=4,          /* Normal helicopter with tail rotor. */
	MAV_TYPE_ANTENNA_TRACKER=5,     /* Ground installation */
	MAV_TYPE_GCS=6,                 /* Operator control unit / ground control station */
	MAV_TYPE_AIRSHIP=7,             /* Airship, controlled */
	MAV_TYPE_FREE_BALLOON=8,        /* Free balloon, uncontrolled */
	MAV_TYPE_ROCKET=9,              /* Rocket */
	MAV_TYPE_GROUND_ROVER=10,       /* Ground rover */
	MAV_TYPE_SURFACE_BOAT=11,       /* Surface vessel, boat, ship */
	MAV_TYPE_SUBMARINE=12,          /* Submarine */
	MAV_TYPE_HEXAROTOR=13,          /* Hexarotor */
	MAV_TYPE_OCTOROTOR=14,          /* Octorotor */
	MAV_TYPE_TRICOPTER=15,          /* Octorotor */
	MAV_TYPE_FLAPPING_WING=16,      /* Flapping wing */
	MAV_TYPE_ENUM_END=17            /*  */
};

enum MAV_COMPONENT {
	MAV_COMP_ID_ALL=0,              /*  */
	MAV_COMP_ID_CAMERA=100,         /*  */
	MAV_COMP_ID_SERVO1=140,         /*  */
	MAV_COMP_ID_SERVO2=141,         /*  */
	MAV_COMP_ID_SERVO3=142,         /*  */
	MAV_COMP_ID_SERVO4=143,         /*  */
	MAV_COMP_ID_SERVO5=144,         /*  */
	MAV_COMP_ID_SERVO6=145,         /*  */
	MAV_COMP_ID_SERVO7=146,         /*  */
	MAV_COMP_ID_SERVO8=147,         /*  */
	MAV_COMP_ID_SERVO9=148,         /*  */
	MAV_COMP_ID_SERVO10=149,        /*  */
	MAV_COMP_ID_SERVO11=150,        /*  */
	MAV_COMP_ID_SERVO12=151,        /*  */
	MAV_COMP_ID_SERVO13=152,        /*  */
	MAV_COMP_ID_SERVO14=153,        /*  */
	MAV_COMP_ID_MAPPER=180,         /*  */
	MAV_COMP_ID_MISSIONPLANNER=190, /*  */
	MAV_COMP_ID_PATHPLANNER=195,    /*  */
	MAV_COMP_ID_IMU=200,            /*  */
	MAV_COMP_ID_IMU_2=201,          /*  */
	MAV_COMP_ID_IMU_3=202,          /*  */
	MAV_COMP_ID_GPS=220,            /*  */
	MAV_COMP_ID_UDP_BRIDGE=240,     /*  */
	MAV_COMP_ID_UART_BRIDGE=241,    /*  */
	MAV_COMP_ID_SYSTEM_CONTROL=250, /*  */
	MAV_COMPONENT_ENUM_END=251      /*  */
};

/// Data stream IDs.
/// A data stream is not a fixed set of messages, but rather a recommendation to the autopilot software.
/// Individual autopilots may or may not obey the recommended messages.
enum MAV_DATA_STREAM {                  // Origin: mavlink\common\common.h
	MAV_DATA_STREAM_ALL=0,              /* Enable all data streams */
	MAV_DATA_STREAM_RAW_SENSORS=1,      /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. */
	MAV_DATA_STREAM_EXTENDED_STATUS=2,  /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS */
	MAV_DATA_STREAM_RC_CHANNELS=3,      /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW */
	MAV_DATA_STREAM_RAW_CONTROLLER=4,   /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. */
                                        /* */
	MAV_DATA_STREAM_POSITION=6,         /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. */
                                        /* */
                                        /* */
                                        /* */
	MAV_DATA_STREAM_EXTRA1=10,          /* Dependent on the autopilot */
	MAV_DATA_STREAM_EXTRA2=11,          /* Dependent on the autopilot */
	MAV_DATA_STREAM_EXTRA3=12,          /* Dependent on the autopilot */
	MAV_DATA_STREAM_ENUM_END=13         /* */
};

enum MAVLINK_DATA_STREAM_TYPE {         // Origin: mavlink\common\common.h
	MAVLINK_DATA_STREAM_IMG_JPEG=1,     /*  */
	MAVLINK_DATA_STREAM_IMG_BMP=2,      /*  */
	MAVLINK_DATA_STREAM_IMG_RAW8U=3,    /*  */
	MAVLINK_DATA_STREAM_IMG_RAW32U=4,   /*  */
	MAVLINK_DATA_STREAM_IMG_PGM=5,      /*  */
	MAVLINK_DATA_STREAM_IMG_PNG=6,      /*  */
	MAVLINK_DATA_STREAM_TYPE_ENUM_END=7 /*  */
};

enum MAV_FRAME {                    // Origin: mavlink\common\common.h
	MAV_FRAME_GLOBAL=0,             /* Global coordinate frame, WGS84. First value x: latitude, second value y: longitude, third value z: positive altitude (MSL) */
	MAV_FRAME_LOCAL_NED=1,          /* Local coordinate frame, Z-up (x: north, y: east, z: down). */
	MAV_FRAME_MISSION=2,            /* NOT a coordinate frame, indicates a mission command. */
	MAV_FRAME_GLOBAL_RELATIVE_ALT=3,/* Global coordinate frame, WGS84, relative altitude QFE. First value x: latitude, second value y: longitude, third value z: positive altitude QFE. */
	MAV_FRAME_LOCAL_ENU=4,          /* Local coordinate frame, Z-down (x: east, y: north, z: up) */
	MAV_FRAME_ENUM_END=5            /*  */
};

enum MAV_CMD {                      // Origin: mavlink\common\common.h
	MAV_CMD_NAV_WAYPOINT=16,        /* Navigate to MISSION. |Hold time in seconds/10. (ignored by fixed wing)| Acceptance radius in meters | 0 to pass through the WP, if > 0 radius in meters to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| Desired yaw angle at MISSION (rotary wing)| Latitude| Longitude| Altitude| */
	MAV_CMD_NAV_LOITER_UNLIM=17,    /* Loiter around this MISSION an unlimited amount of time |Empty| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_LOITER_TURNS=18,    /* Loiter around this MISSION for X turns |Turns| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_LOITER_TIME=19,     /* Loiter around this MISSION for X seconds |Seconds (decimal)| Empty| Radius around MISSION, in meters. If positive loiter clockwise, else counter-clockwise| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_NAV_LAND=21,            /* Land at location |Empty| Empty| Empty| Desired yaw angle.| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_TAKEOFF=22,         /* Takeoff from ground / hand |Minimum pitch (if airspeed sensor present), desired pitch without sensor| Empty| Empty| Yaw angle (if magnetometer present), ignored without magnetometer| Latitude| Longitude| Altitude|  */
	MAV_CMD_NAV_ROI=80,             /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicles control system to control the vehicle attitude and the attitude of various sensors such as cameras. |Region of intereset mode. (see MAV_ROI enum)| MISSION index/ target ID. (see MAV_ROI enum)| ROI index (allows a vehicle to manage multiple ROI's)| Empty| x the location of the fixed ROI (see MAV_FRAME)| y| z|  */
	MAV_CMD_NAV_PATHPLANNING=81,    /* Control autonomous path planning on the MAV. |0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| Empty| Yaw angle at goal, in compass degrees, [0..360]| Latitude/X of goal| Longitude/Y of goal| Altitude/Z of goal|  */
	MAV_CMD_NAV_LAST=95,            /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_DELAY=112,    /* Delay mission state machine. |Delay in seconds (decimal)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_CHANGE_ALT=113, /* Ascend/descend at rate.  Delay mission state machine until desired altitude reached. |Descent / Ascend rate (m/s)| Empty| Empty| Empty| Empty| Empty| Finish Altitude|  */
	MAV_CMD_CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |Distance (meters)| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_YAW=115,      /* Reach a certain target angle. |target angle: [0-360], 0 is north| speed during yaw change:[deg per second]| direction: negative: counter clockwise, positive: clockwise [-1,1]| relative offset or absolute angle: [ 1,0]| Empty| Empty| Empty|  */
	MAV_CMD_CONDITION_LAST=159,     /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_MODE=176,        /* Set system mode. |Mode, as defined by ENUM MAV_MODE| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_JUMP=177,            /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |Sequence number| Repeat count| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_CHANGE_SPEED=178,    /* Change speed and/or throttle set points. |Speed type (0=Airspeed, 1=Ground Speed)| Speed  (m/s, -1 indicates no change)| Throttle  ( Percent, -1 indicates no change)| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_HOME=179,        /* Changes the home location either to the current location or a specified location. |Use current (1=use current location, 0=use specified location)| Empty| Empty| Empty| Latitude| Longitude| Altitude|  */
	MAV_CMD_DO_SET_PARAMETER=180,   /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |Parameter number| Parameter value| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_RELAY=181,       /* Set a relay to a condition. |Relay number| Setting (1=on, 0=off, others possible depending on system hardware)| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_REPEAT_RELAY=182,    /* Cycle a relay on and off for a desired number of cyles with a desired period. |Relay number| Cycle count| Cycle time (seconds, decimal)| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_SET_SERVO=183,       /* Set a servo to a desired PWM value. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_DO_REPEAT_SERVO=184,    /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |Servo number| PWM (microseconds, 1000 to 2000 typical)| Cycle count| Cycle time (seconds)| Empty| Empty| Empty|  */
	MAV_CMD_DO_CONTROL_VIDEO=200,   /* Control onboard camera system. |Camera ID (-1 for all)| Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| Transmission mode: 0: video stream, >0: single images every n seconds (decimal)| Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| Empty| Empty| Empty|  */
	MAV_CMD_DO_LAST=240,            /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |Empty| Empty| Empty| Empty| Empty| Empty| Empty|  */
	MAV_CMD_PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. |Gyro calibration: 0: no, 1: yes| Magnetometer calibration: 0: no, 1: yes| Ground pressure: 0: no, 1: yes| Radio calibration: 0: no, 1: yes| Empty| Empty| Empty|  */
	MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow| X axis offset (or generic dimension 1), in the sensor's raw units| Y axis offset (or generic dimension 2), in the sensor's raw units| Z axis offset (or generic dimension 3), in the sensor's raw units| Generic dimension 4, in the sensor's raw units| Generic dimension 5, in the sensor's raw units| Generic dimension 6, in the sensor's raw units|  */
	MAV_CMD_PREFLIGHT_STORAGE=245,  /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM| Reserved| Reserved| Empty| Empty| Empty|  */
	MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |0: Do nothing for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot.| 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer.| Reserved| Reserved| Empty| Empty| Empty|  */
	MAV_CMD_OVERRIDE_GOTO=252,      /* Hold / continue the current action |MAV_GOTO_DO_HOLD: hold MAV_GOTO_DO_CONTINUE: continue with next item in mission plan| MAV_GOTO_HOLD_AT_CURRENT_POSITION: Hold at current position MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position| MAV_FRAME coordinate frame of hold point| Desired yaw angle in degrees| Latitude / X position| Longitude / Y position| Altitude / Z position|  */
	MAV_CMD_MISSION_START=300,      /* start running a mission |first_item: the first mission item to run| last_item:  the last mission item to run (after this item is run, the mission ends)|  */
	MAV_CMD_COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |1 to arm, 0 to disarm|  */
	MAV_CMD_ENUM_END=401            /*  */
};

/*----------------------------------- Types ----------------------------------*/

typedef enum {                  // Origin: mavlink\mavlink_types.h
   MAVLINK_TYPE_CHAR     = 0,
   MAVLINK_TYPE_UINT8_T  = 1,
   MAVLINK_TYPE_INT8_T   = 2,
   MAVLINK_TYPE_UINT16_T = 3,
   MAVLINK_TYPE_INT16_T  = 4,
   MAVLINK_TYPE_UINT32_T = 5,
   MAVLINK_TYPE_INT32_T  = 6,
   MAVLINK_TYPE_UINT64_T = 7,
   MAVLINK_TYPE_INT64_T  = 8,
   MAVLINK_TYPE_FLOAT    = 9,
   MAVLINK_TYPE_DOUBLE   = 10
} mavlink_message_type_t;

typedef enum {                  // Origin: mavlink\mavlink_types.h
    MAVLINK_PARSE_STATE_UNINIT=0,
    MAVLINK_PARSE_STATE_IDLE,
    MAVLINK_PARSE_STATE_GOT_STX,
    MAVLINK_PARSE_STATE_GOT_SEQ,
    MAVLINK_PARSE_STATE_GOT_LENGTH,
    MAVLINK_PARSE_STATE_GOT_SYSID,
    MAVLINK_PARSE_STATE_GOT_COMPID,
    MAVLINK_PARSE_STATE_GOT_MSGID,
    MAVLINK_PARSE_STATE_GOT_PAYLOAD,
    MAVLINK_PARSE_STATE_GOT_CRC1
} mavlink_parse_state_t;        ///< The state machine for the comm parser

/*---------------------------------- Constants -------------------------------*/

VAR_STATIC const uint8_t Mavlink_Crc[] = MAVLINK_MESSAGE_CRCS ;

/// Names of parameters
VAR_STATIC const uint8_t sParameter_Name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH] = {
/*
    "RLL2SRV_P\0",     // Roll Kp
    "RLL2SRV_I\0",     // Roll Ki
//  "RLL2SRV_D\0",     // Roll Kd
    "PTCH2SRV_P\0",    // Pitch Kp
    "PTCH2SRV_I\0",    // Pitch Ki
//  "PTCH2SRV_D\0",    // Pitch Kd
    "ENRGY2THR_P\0",   // Altitude Kp (via throttle)
    "ENRGY2THR_I\0",   // Altitude Ki (via throttle)
//  "ENRGY2THR_D\0",   // Altitude Kd (via throttle)
    "HDNG2RLL_P\0",    // Navigation Kp (via roll)
    "HDNG2RLL_I\0"     // Navigation Ki (via roll)
//  "HDNG2RLL_D\0"     // Navigation Kd (via roll)

    "CTRL_ROL_ANG_P\0", // Roll Kp
    "CTRL_ROL_ANG_I\0", // Roll Ki
//  "CTRL_ROL_ANG_D\0", // Roll Kd
    "CTRL_PCH_ANG_P\0", // Pitch Kp
    "CTRL_PCH_ANG_I\0", // Pitch Ki
//  "CTRL_PCH_ANG_D\0", // Pitch Kd
    "NAV_ALT_POS_P\0",  // Altitude Kp (via throttle)
    "NAV_ALT_POS_I\0",  // Altitude Ki (via throttle)
 //  NAV_ALT_POS_D"\0", // Altitude Kd (via throttle)
    "NAV_ROL_ANG_P\0",  // Navigation Kp (via roll)
    "NAV_ROL_ANG_I\0"   // Navigation Ki (via roll)
//  "NAV_ROL_ANG_D\0"   // Navigation Kd (via roll)
*/
    // Copter GCS standard: GROUP_SUBGROUP_P / _I / _D / _IMAX
    "ROL_ANG_P\0",  // Roll Kp
    "ROL_ANG_I\0",  // Roll Ki
//  "ROL_ANG_D\0",  // Roll Kd
    "PCH_ANG_P\0",  // Pitch Kp
    "PCH_ANG_I\0",  // Pitch Ki
//  "PCH_ANG_D\0",  // Pitch Kd
    "ALT_POS_P\0",  // Altitude Kp (via throttle)
    "ALT_POS_I\0",  // Altitude Ki (via throttle)
//  "ALT_POS_D"\0", // Altitude Kd (via throttle)
    "NAV_ANG_P\0",  // Navigation Kp (via roll)
    "NAV_ANG_I\0",  // Navigation Ki (via roll)
//  "NAV_ANG_D\0",  // Navigation Kd (via roll)
    "NAV_BANK\0"    // Max bank angle during navigation [deg]
};

//VAR_STATIC const uint8_t Autopilot_Type = MAV_AUTOPILOT_GENERIC;  // Autopilot capabilities
//VAR_STATIC const uint8_t System_Type = MAV_TYPE_FIXED_WING;       // Aircraft type
VAR_STATIC const uint8_t System_ID = 1;                             // System ID (original: 20)
VAR_STATIC const enum MAV_COMPONENT Component_ID = 1;               // Component ID (original: MAV_COMP_ID_IMU)

/*---------------------------------- Globals ---------------------------------*/

/*
VAR_GLOBAL xQueueHandle xTelemetry_Queue;
VAR_GLOBAL struct global_struct global_data;
*/

/*----------------------------------- Locals ---------------------------------*/

/*
VAR_STATIC uint8_t System_Mode = MAV_MODE_PREFLIGHT;  // Booting up
VAR_STATIC uint8_t System_State = MAV_STATE_STANDBY;  // System ready for flight
VAR_STATIC uint16_t mode = MAV_MODE_UNINIT;           // Defined in mavlink_types.h, which is included by mavlink.h
VAR_STATIC uint32_t custom_mode;
VAR_STATIC mavlink_system_t mavlink_system;
VAR_STATIC mavlink_param_set_t set;
VAR_STATIC mavlink_status_t status;
VAR_STATIC mavlink_message_t msg;
*/

VAR_STATIC uint8_t current_tx_seq = 0;
//VAR_STATIC uint16_t packet_drops = 0;
//VAR_STATIC uint8_t packet_rx_drop_count;
VAR_STATIC uint16_t m_parameter_i = ONBOARD_PARAM_COUNT;
VAR_STATIC uint8_t msgid;
VAR_STATIC uint16_t Crc;
VAR_STATIC STRUCT_WPT wpt;

VAR_STATIC uint8_t Rx_Msg[PAYLOAD_LEN];                 // buffer for incoming messages
VAR_STATIC uint8_t Tx_Msg[PACKET_LEN];                  // buffer for outgoing messages

VAR_STATIC uint8_t ucStream_Tick[MAV_DATA_STREAM_ENUM_END] = { // tick counters for data streams
    MAX_STREAM_RATE,    /*  0: all data streams */
    MAX_STREAM_RATE,    /*  1: IMU_RAW, GPS_RAW, GPS_STATUS */
    MAX_STREAM_RATE,    /*  2: GPS_STATUS, CONTROL_STATUS, AUX_STATUS */
    MAX_STREAM_RATE,    /*  3: RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW */
    MAX_STREAM_RATE,    /*  4: ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT */
    MAX_STREAM_RATE,    /*  5: - */
    MAX_STREAM_RATE,    /*  6: LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT */
    MAX_STREAM_RATE,    /*  7: - */
    MAX_STREAM_RATE,    /*  8: - */
    MAX_STREAM_RATE,    /*  9: - */
    MAX_STREAM_RATE,    /* 10: Extra 1, autopilot dependent */
    MAX_STREAM_RATE,    /* 11: Extra 2, autopilot dependent */
    MAX_STREAM_RATE     /* 12: Extra 3, autopilot dependent */
};

VAR_STATIC uint8_t ucStream_Rate[MAV_DATA_STREAM_ENUM_END] = { // frequency of data streams
    0,  /*  0: all data streams */
    0,  /*  1: IMU_RAW, GPS_RAW, GPS_STATUS */
    0,  /*  2: GPS_STATUS, CONTROL_STATUS, AUX_STATUS */
    0,  /*  3: RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW */
    0,  /*  4: ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT */
    0,  /*  5: - */
    1,  /*  6: LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT */
    0,  /*  7: - */
    0,  /*  8: - */
    0,  /*  9: - */
    5,  /* 10: Extra 1, autopilot dependent */
    2,  /* 11: Extra 2, autopilot dependent */
    0   /* 12: Extra 3, autopilot dependent */
};

VAR_STATIC float fParam_Value[ONBOARD_PARAM_COUNT] = {
    ROLL_KP,    //!< default roll kp
    ROLL_KI,    //!< default roll ki
    PITCH_KP,   //!< default pitch kp
    PITCH_KI,   //!< default pitch ki
    0.0f,       //!< dummy placeholder
    0.0f,       //!< dummy placeholder
    NAV_KP,     //!< default direction kp
    NAV_KI,     //!< default direction ki
    NAV_BANK    //!< default max bank angle
};              //!< gains for PID loops

/*--------------------------------- Prototypes -------------------------------*/

static __inline void Checksum_Init( void );
static __inline void Checksum_Accumulate( uint8_t data );
static void Mavlink_Send( uint8_t crc_extra );
void Mavlink_Heartbeat( void );
void Mavlink_Hud( void );
void Mavlink_Attitude( void );
void Mavlink_Gps_Raw( void );
void Mavlink_Position( void );
void Mavlink_Param_Send( uint16_t param_index, uint16_t param_count );
void Mavlink_Param_Set( void );
void Mavlink_HIL_State( void );
static bool Mavlink_Parse( void );
bool Mavlink_Stream_Trigger( uint8_t stream );

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
    uint8_t length = Tx_Msg[1];

    Checksum_Init();
    Tx_Msg[0] = MAVLINK_STX;
    Tx_Msg[2] = current_tx_seq++;          // One sequence number per component
    Tx_Msg[3] = System_ID;
    Tx_Msg[4] = (uint8_t)Component_ID;

    for (j = 1; j < length + 6; j++) {
        Checksum_Accumulate(Tx_Msg[j]);
    }
    Checksum_Accumulate(crc_extra);

    Tx_Msg[j++] = (uint8_t)(Crc & 0xFF);
    Tx_Msg[j] = (uint8_t)(Crc >> 8);
    for (j = 0; j < length + 8; j++) {
        USART1_Putch(Tx_Msg[j]);
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
/// Name = MAVLINK_MSG_ID_HEARTBEAT, ID = 0, Length = 9
///
/// Field         Offset  Type    Meaning
/// ----------------------------------------------------------------------------
/// OSD mode         0   uint32_t Bitfield for use for autopilot-specific flags.
/// Heartbeat type   4   uint8_t  Type of the MAV (quadrotor, heli, see MAV_TYPE ENUM)
/// Base mode        6   uint8_t  System mode bitfield, see MAV_MODE_FLAGS ENUM
/// autopilot        ?   uint8_t  Autopilot type / class. see MAV_AUTOPILOT ENUM
/// system_status    ?   uint8_t  System status flag, see MAV_STATE ENUM
/// mavlink_version  ?   uint8_t_mavlink_version
///                               MAVLink version, read only,
///                               gets added by protocol because of magic data type
///
//----------------------------------------------------------------------------
void Mavlink_Heartbeat( void ) {
	uint16_t j;

    for (j = 0; j < PACKET_LEN; j++) {
        Tx_Msg[j] = 0;
    }

    Tx_Msg[1] = 9;                              // Payload length
	Tx_Msg[5] = MAVLINK_MSG_ID_HEARTBEAT;       // Heartbeat message ID
	Tx_Msg[10] = (uint8_t)MAV_TYPE_FIXED_WING;  // Type of the MAV, defined in MAV_TYPE ENUM
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

    Tx_Msg[1] = 20;                                          // Payload length
    Tx_Msg[5] = MAVLINK_MSG_ID_VFR_HUD;                      // VFR HUD message ID
    *((float *)(&Tx_Msg[6])) = 0.0f;                         // Airspeed
    *((float *)(&Tx_Msg[10])) = (float)Gps_Speed_Kt();       // GPS speed
    *((float *)(&Tx_Msg[14])) = Nav_Altitude();              // Altitude
    *((float *)(&Tx_Msg[18])) = 0.0f;                        // Climb rate
    *((uint16_t *)(&Tx_Msg[22])) = Gps_Heading_Deg();        // Heading
    *((uint16_t *)(&Tx_Msg[24])) = (uint16_t)Nav_Throttle(); // Throttle

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

    Tx_Msg[1] = 28;                                    // Payload length
    Tx_Msg[5] = MAVLINK_MSG_ID_ATTITUDE;               // Attitude message ID
    *((uint32_t *)(&Tx_Msg[6])) = 0;                   // time from boot [ms]
    *((float *)(&Tx_Msg[10])) = Attitude_Roll_Rad();   // roll
    *((float *)(&Tx_Msg[14])) = Attitude_Pitch_Rad();  // pitch
    *((float *)(&Tx_Msg[18])) = Attitude_Yaw_Rad();    // yaw
    *((float *)(&Tx_Msg[22])) = 0.0f;                  // roll rate
    *((float *)(&Tx_Msg[26])) = 0.0f;                  // pitch rate
    *((float *)(&Tx_Msg[30])) = 0.0f;                  // yaw rate

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_ATTITUDE]);
}

//----------------------------------------------------------------------------
//
/// \brief   Send GPS raw data
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
void Mavlink_Gps_Raw( void ) {

    Tx_Msg[1] = 30;                                            // Payload length
    Tx_Msg[5] = MAVLINK_MSG_ID_GPS_RAW_INT;                    // GPS message ID
    *((uint64_t *)(&Tx_Msg[6])) = 0;                           // time from boot [us]
    *((int32_t *)(&Tx_Msg[14])) = Gps_Latitude();              // latitude
    *((int32_t *)(&Tx_Msg[18])) = Gps_Longitude();             // longitude
    *((int32_t *)(&Tx_Msg[22])) = (int32_t)Nav_Altitude();     // altitude
    *((uint16_t *)(&Tx_Msg[26])) = 65535;                      // eph
    *((uint16_t *)(&Tx_Msg[28])) = 65535;                      // epv
    *((uint16_t *)(&Tx_Msg[30])) = Gps_Speed_Kt();             // velocity
    *((uint16_t *)(&Tx_Msg[32])) = Gps_Heading_Deg();          // course over ground
    Tx_Msg[34] = Gps_Fix();                                    // fix type
    Tx_Msg[35] = 255;                                          // satellites

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_GPS_RAW_INT]);
}

//----------------------------------------------------------------------------
//
/// \brief   Send position
/// \param   -
/// \returns -
/// \remarks
/// Name = MAVLINK_MSG_ID_GLOBAL_POSITION_INT, ID = 33, Length = 28
///
/// Field             Offset Type       Meaning
/// ------------------------------------------
/// time_usec            0   uint32_t   time since boot [ms]
/// lat                  8   int32_t    latitude
/// lon                 12   int32_t    longitude
/// alt                 16   int32_t    altitude
/// relative_alt        20   int32_t    altitude above ground
/// vx                  22   uint16_t   ground x speed (latitude)
/// vy                  24   uint16_t   ground y speed (longitude)
/// vz                  26   uint16_t   ground z speed (altitude)
/// hdg                 28   uint16_t   compass heading
///
//----------------------------------------------------------------------------
void Mavlink_Position( void ) {

    Tx_Msg[1] = 28;                                            // payload length
    Tx_Msg[5] = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;            // global position message ID
    *((uint32_t *)(&Tx_Msg[6])) = 0;                           // time from boot [ms]
    *(( int32_t *)(&Tx_Msg[10])) = Gps_Latitude();             // latitude
    *(( int32_t *)(&Tx_Msg[14])) = Gps_Longitude();            // longitude
    *(( int32_t *)(&Tx_Msg[18])) = (int32_t)Nav_Altitude();    // altitude
    *(( int32_t *)(&Tx_Msg[22])) = (int32_t)Nav_Altitude();    // altitude above ground
    *((uint16_t *)(&Tx_Msg[26])) = 0;                          // ground x speed
    *((uint16_t *)(&Tx_Msg[28])) = 0;                          // ground y speed
    *((uint16_t *)(&Tx_Msg[30])) = 0;                          // ground z speed
    *((uint16_t *)(&Tx_Msg[32])) = Gps_Heading_Deg();          // compass heading

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_GLOBAL_POSITION_INT]);
}

//----------------------------------------------------------------------------
//
/// \brief   Send parameter value
/// \param   param_index = index of parameter to be sent
/// \returns -
/// \remarks
/// Name = MAVLINK_MSG_ID_PARAM_VALUE, ID = 22, Length = 25
///
/// Field       Offset Type      Meaning
/// -------------------------------------
/// param_value   0    float     Onboard parameter value
/// param_count   4    uint16_t  Total number of onboard parameters
/// param_index   6    uint16_t  Index of current onboard parameter
/// param_id      8    array     Onboard parameter name, null terminated ...
/// param_type    24   uint8_t   Onboard parameter type: see MAVLINK_TYPE enum
///
//----------------------------------------------------------------------------
void Mavlink_Param_Send( uint16_t param_index, uint16_t param_count ) {

    uint8_t j;

    Tx_Msg[1] = 25;                                        // Payload length
    Tx_Msg[5] = MAVLINK_MSG_ID_PARAM_VALUE;                // Parameter value message ID
    *((float *)(&Tx_Msg[6])) = fParam_Value[param_index];  // Parameter value
    *((uint16_t *)(&Tx_Msg[10])) = param_count;            // Total number of parameters
    *((uint16_t *)(&Tx_Msg[12])) = param_index;            // Parameter index
    for (j = 0; j < 16; j++) {
        Tx_Msg[j + 14] = sParameter_Name[param_index][j];  // Parameter name
    }
    Tx_Msg[30] = (uint8_t)MAVLINK_TYPE_FLOAT;              // Parameter type

    Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_PARAM_VALUE]);
}

//----------------------------------------------------------------------------
//
/// \brief   Set parameter
/// \param   -
/// \returns TRUE if parameter has been set, FALSE otherwise
/// \remarks
/// Name = MAVLINK_MSG_ID_PARAM_SET, ID = 23, Length = 23
///
/// Field         Offset Type     Meaning
/// -------------------------------------
/// param_value      0   float    Onboard parameter value
/// target_system    4   uint8_t  System ID
/// target_component 5   uint8_t  Component ID
/// param_id, 16     6   array    Onboard parameter name,
///                               null terminated if length < 16 chars
///                               without null termination if length = 16 chars
/// param_type      22   uint8_t  Onboard parameter type: see MAVLINK_TYPE enum
///
//----------------------------------------------------------------------------
void Mavlink_Param_Set( void ) {

    float f_value;
    bool match;
	uint8_t i, j;

    f_value = *(float *)(&Rx_Msg[0]);
    if ((Rx_Msg[0] == System_ID) &&                // message is for this system
        (Rx_Msg[1] == Component_ID)) {             // message is for this component
        for (i = 0; i < ONBOARD_PARAM_COUNT; i++) {
            match = TRUE;
            for (j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++) {
                if (Rx_Msg[j + 6] != sParameter_Name[i][j]) {   // compare name
                    match = FALSE;
                }
                if (sParameter_Name[i][j] == '\0') {            // null termination is reached
                    j = ONBOARD_PARAM_NAME_LENGTH;              // force termination of for loop
                }
            }
            if (match) {                                            // name matched and
                if ((fParam_Value[i] != f_value) &&                 // there is a difference and
                     !isnan(f_value) &&                             // new value is a number and
                     !isinf(f_value) &&                             // is NOT infinity and
                     (Rx_Msg[22] == (uint8_t)MAVLINK_TYPE_FLOAT)) { // is a float
                    fParam_Value[i] = f_value;                      // write changes
                    Mavlink_Param_Send(i, 1);                       // emit changes
                }
            }
        }
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Get HIL status
/// \param   -
/// \returns -
/// \remarks
/// Name = MAVLINK_MSG_ID_HIL_STATE
///
/// Field       Offset Type     Meaning
/// -------------------------------------
/// time_usec      0   uint64_t
/// roll           8   float
/// pitch         12   float
/// yaw           16   float
/// rollspeed     20   float
/// pitchspeed    24   float
/// yawspeed      28   float
/// lat           32   int32_t
/// lon           36   int32_t
/// alt           40   int32_t
/// vx            44   int16_t
/// vy            46   int16_t
/// vz            48   int16_t
/// xacc          50   int16_t
/// yacc          52   int16_t
/// zacc          54   int16_t
///
//----------------------------------------------------------------------------
void Mavlink_HIL_State( void ) {
/*
    rollspeed  = *((float *)(&Tx_Msg[26]));    //
    pitchspeed = *((float *)(&Tx_Msg[30]));    //
    yawspeed   = *((float *)(&Tx_Msg[34]));    //
    lat = *((int32_t *)(&Tx_Msg[38]));         //
    lon = *((int32_t *)(&Tx_Msg[42]));         //
    alt = *((int32_t *)(&Tx_Msg[46]));         //
    xacc = *((int16_t *)(&Tx_Msg[56]));        //
    yacc = *((int16_t *)(&Tx_Msg[58]));        //
    zacc = *((int16_t *)(&Tx_Msg[60]));        //
*/
}

//----------------------------------------------------------------------------
//
/// \brief   Decode "request data stream" message
/// \param   -
/// \returns -
/// \remarks
/// Name = REQUEST_DATA_STREAM, ID = 66, Length = 6
///
/// Field         Offset Type   Meaning
/// ----------------------------------------------------------------------
/// req_message_rate 0 uint16_t interval between two messages of this type
/// target_system    2 uint8_t  target requested to send the stream
/// target_component 3 uint8_t  component requested to send the stream
/// req_stream_id    4 uint8_t  ID of the requested data stream
/// start_stop       5 uint8_t  1 to start sending, 0 to stop sending
///
//----------------------------------------------------------------------------
 void Mavlink_Data_Stream( void ) {

    uint8_t freq;

    if (Rx_Msg[5] == 0) {           // stop
        freq = 0;                   // force frequency = 0
    } else if (Rx_Msg[5] == 1) {    // start
        freq = Rx_Msg[0];
    } else {                        // wrong start / stop field
        return;
    }

    switch(Rx_Msg[4]) {             // stream ID

        case MAV_DATA_STREAM_ALL:
            ucStream_Rate[MAV_DATA_STREAM_RAW_SENSORS    ] = freq;
            ucStream_Rate[MAV_DATA_STREAM_EXTENDED_STATUS] = freq;
            ucStream_Rate[MAV_DATA_STREAM_RC_CHANNELS    ] = freq;
            ucStream_Rate[MAV_DATA_STREAM_RAW_CONTROLLER ] = freq;
            ucStream_Rate[MAV_DATA_STREAM_POSITION       ] = freq;
            ucStream_Rate[MAV_DATA_STREAM_EXTRA1         ] = freq;
            ucStream_Rate[MAV_DATA_STREAM_EXTRA2         ] = freq;
            ucStream_Rate[MAV_DATA_STREAM_EXTRA3         ] = freq;
            break;

        case MAV_DATA_STREAM_RAW_SENSORS:
            ucStream_Rate[MAV_DATA_STREAM_RAW_SENSORS] = freq;
            break;

        case MAV_DATA_STREAM_EXTENDED_STATUS:
            ucStream_Rate[MAV_DATA_STREAM_EXTENDED_STATUS] = freq;
            break;

        case MAV_DATA_STREAM_RC_CHANNELS:
            ucStream_Rate[MAV_DATA_STREAM_RC_CHANNELS] = freq;
            break;

        case MAV_DATA_STREAM_RAW_CONTROLLER:
            ucStream_Rate[MAV_DATA_STREAM_RAW_CONTROLLER] = freq;
            break;

        case MAV_DATA_STREAM_POSITION:
            ucStream_Rate[MAV_DATA_STREAM_POSITION] = freq;
            break;

        case MAV_DATA_STREAM_EXTRA1:
            ucStream_Rate[MAV_DATA_STREAM_EXTRA1] = freq;
            break;

        case MAV_DATA_STREAM_EXTRA2:
            ucStream_Rate[MAV_DATA_STREAM_EXTRA2] = freq;
            break;

        case MAV_DATA_STREAM_EXTRA3:
            ucStream_Rate[MAV_DATA_STREAM_EXTRA3] = freq;
            break;

        default:
            break;
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Send total number of waypoints
/// \param   -
/// \returns -
/// \remarks
/// Name = MAVLINK_MSG_ID_MISSION_COUNT, ID = 44, Length = 4
///
/// Field         Offset Type   Meaning
/// ----------------------------------------------------------------------
/// count            0 uint16_t number of waypoints
/// target_system    2 uint8_t  target requested to send the stream
/// target_component 3 uint8_t  component requested to send the stream
///
//----------------------------------------------------------------------------
void Mavlink_Mission_Count( void ) {

    if ((Rx_Msg[0] == System_ID) &&                // message is for this system
        (Rx_Msg[1] == Component_ID)) {             // message is for this component
        Tx_Msg[1] = 4;                                  // payload length
        Tx_Msg[5] = MAVLINK_MSG_ID_MISSION_COUNT;       // mission count message ID
        *((uint16_t *)(&Tx_Msg[6])) = Nav_Wpt_Number(); // number of waypoints

        Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_MISSION_COUNT]);
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Send waypoint data
/// \param   wpt_index = index of waypoint to be sent
/// \param   wpt_count = total number of waypoint
/// \returns -
/// \remarks
/// Name = MAVLINK_MSG_ID_MISSION_ITEM, ID = 39, Length = 37
///
/// Field       Offset Type     Meaning
/// --------------------------------------------------------------------
/// param1            0 float    acceptance radius
/// param2            4 float    permanence time
/// param3            8 float    orbit direction
/// param4           12 float    yaw orientation
/// x                16 float    x position or latitude
/// y                20 float    y position or longitude
/// z                24 float    z position or altitude
/// seq              28 uint16_t sequence
/// command          30 uint16_t scheduled action
/// target_system    32 uint8_t  target requested to send the stream
/// target_component 33 uint8_t  component requested to send the stream
/// frame            34 uint8_t  coordinate system
/// current          35 uint8_t  false:0, true:1
/// auto_continue    36 uint8_t  autocontinue to next wp
///
//----------------------------------------------------------------------------
void Mavlink_Mission_Item( void ) {

    uint16_t index;

    if ((Rx_Msg[2] == System_ID) &&                // message is for this system
        (Rx_Msg[3] == Component_ID)) {             // message is for this component

        index = *((uint16_t *)(&Rx_Msg[0]));                    // get waypoint index
        Nav_Wpt_Get(index, &wpt);                               // get waypoint data
        Tx_Msg[1] = 37;                                         // payload length
        Tx_Msg[5] = MAVLINK_MSG_ID_MISSION_ITEM;                // message ID
        *((float    *)(&Tx_Msg[ 6])) = 100.0f;                  // radius
        *((float    *)(&Tx_Msg[10])) =   0.0f;                  // time
        *((float    *)(&Tx_Msg[14])) =   0.0f;                  // orbit
        *((float    *)(&Tx_Msg[18])) =   0.0f;                  // yaw
        *((float    *)(&Tx_Msg[22])) = wpt.Lat;                 // latitude
        *((float    *)(&Tx_Msg[26])) = wpt.Lon;                 // longitude
        *((float    *)(&Tx_Msg[30])) = wpt.Alt;                 // altitude
        *((uint16_t *)(&Tx_Msg[34])) = index;                   // sequence
        *((uint16_t *)(&Tx_Msg[36])) = (uint8_t)MAV_CMD_NAV_WAYPOINT; // command
                          Tx_Msg[38] = 1;                       // target sys
                          Tx_Msg[39] = 1;                       // target comp
                          Tx_Msg[40] = (uint8_t)MAV_FRAME_GLOBAL; // frame
                          Tx_Msg[41] = 0;                       // current
                          Tx_Msg[42] = 1;                       // auto continue

        Mavlink_Send(Mavlink_Crc[MAVLINK_MSG_ID_MISSION_ITEM]);
    }
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
//    static uint8_t magic;
//    static uint8_t sysid;
//    static uint8_t compid;
    static mavlink_parse_state_t parse_state = MAVLINK_PARSE_STATE_UNINIT;
    static uint8_t len;
//    static uint8_t buffer_overrun = 0;
//    static uint8_t parse_error = 0;
    static uint8_t packet_idx;
    static uint8_t seq;
    static uint8_t current_rx_seq;
//    static uint8_t packet_rx_success_count = 0;

    bool msg_received = FALSE;

    while (USART1_Getch (&c)) {               // received another character
	switch (parse_state) {
        case MAVLINK_PARSE_STATE_UNINIT:
        case MAVLINK_PARSE_STATE_IDLE:
            if (c == MAVLINK_STX) {
                parse_state = MAVLINK_PARSE_STATE_GOT_STX;
                len = 0;
//                magic = c;
                Checksum_Init();
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_STX:
            if (msg_received || (c > PAYLOAD_LEN)) {
//                buffer_overrun++;
//                parse_error++;
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
//            sysid = c;
            Checksum_Accumulate(c);
            parse_state = MAVLINK_PARSE_STATE_GOT_SYSID;
            break;

        case MAVLINK_PARSE_STATE_GOT_SYSID:
//            compid = c;
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
            Rx_Msg[packet_idx++] = c;
            Checksum_Accumulate(c);
            if (packet_idx == len) {
                parse_state = MAVLINK_PARSE_STATE_GOT_PAYLOAD;
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_PAYLOAD:
            Checksum_Accumulate(Mavlink_Crc[msgid]);
            if (c != (Crc & 0xFF)) { // Check first checksum byte
//                parse_error++;
                msg_received = FALSE;
                parse_state = MAVLINK_PARSE_STATE_IDLE;
                if (c == MAVLINK_STX) {
                    parse_state = MAVLINK_PARSE_STATE_GOT_STX;
                    len = 0;
                    Checksum_Init();
                }
            } else {
                parse_state = MAVLINK_PARSE_STATE_GOT_CRC1;
                Rx_Msg[packet_idx] = c;
            }
            break;

        case MAVLINK_PARSE_STATE_GOT_CRC1:
            if (c != (Crc >> 8)) {	// Check second checksum byte
//                parse_error++;
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
                Rx_Msg[packet_idx + 1] = c;
    //            memcpy(r_message, rxmsg, sizeof(mavlink_message_t));
            }
            break;
        }

        // If a message has been sucessfully decoded, check index
        if (msg_received) {
            current_rx_seq = seq;
            // Initial condition: If no packet has been received so far, drop count is undefined
//            if (packet_rx_success_count == 0) packet_rx_drop_count = 0;
            // Count this packet as received
//            packet_rx_success_count++;
        }

        current_rx_seq = current_rx_seq + 1;
//        packet_rx_drop_count = parse_error;
//        parse_error = 0;
	}
    return (msg_received);
}

//----------------------------------------------------------------------------
//
/// \brief   Receive mavlink packets
/// \param   -
/// \returns -
/// \remarks Handles packet value by calling the appropriate functions.
///          See Ardupilot function handleMessage at following link:
/// https://github.com/diydrones/ardupilot/blob/master/ArduPlane/GCS_Mavlink.pde
///
//----------------------------------------------------------------------------
void Mavlink_Receive(void)
{
    if (Mavlink_Parse()) {                          // Received a correct packet
        switch (msgid) {                            // Handle message
            case MAVLINK_MSG_ID_HEARTBEAT:          // E.g. read GCS heartbeat and go into comm lost mode if timer times out
                break;
            case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
                Mavlink_Data_Stream();
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:	    //
                break;
            case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //
                m_parameter_i = 0;
                break;
            case MAVLINK_MSG_ID_PARAM_SET:          //
                Mavlink_Param_Set();
                break;
            case MAVLINK_MSG_ID_PARAM_VALUE:        //
                break;
            case MAVLINK_MSG_ID_HIL_STATE:
                Mavlink_HIL_State();
                break;
            case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
                Mavlink_Mission_Count();
                break;
            case MAVLINK_MSG_ID_MISSION_REQUEST:
                Mavlink_Mission_Item();
                break;
            default:				                // Do nothing
                break;
        }
    }		                                        //
//    packet_drops += packet_rx_drop_count;           // Update global packet drops counter
}

//----------------------------------------------------------------------------
//
/// \brief   Send low-priority messages at a maximum rate of xx Hertz
/// \param   -
/// \returns -
/// \remarks This function sends messages at a lower rate to not exceed the
///          wireless bandwidth. It sends one message each time it is called
///          until the buffer is empty.
///          Call this function with xx Hertz to increase/decrease bandwidth.
///          See Ardupilot function queued_param_send at link:
/// https://github.com/diydrones/ardupilot/blob/master/ArduPlane/GCS_Mavlink.pde
///
//----------------------------------------------------------------------------
void Mavlink_Queued_Send(uint8_t cycles)
{
    if (m_parameter_i < ONBOARD_PARAM_COUNT) {      // must send parameters
        if ((cycles % 10) == 0) {                   // @ 5 Hz
            Mavlink_Param_Send(m_parameter_i++, ONBOARD_PARAM_COUNT);  // send parameters
        }
    } else if ((cycles % 200) == 0) {               // @ 0.25 Hz
    } else if ((cycles % 50) == 0) {                // @ 1 Hz
        Mavlink_Heartbeat();                        // send heartbeat
    } else if ((cycles % 5) == 0) {                 // @ 10 Hz
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Send data stream
/// \param   -
/// \returns -
/// \remarks call function @ 50 Hz
///
//----------------------------------------------------------------------------
void Mavlink_Stream_Send(void)
{
/*
    if (_queued_parameter != NULL) {
        if (streamRateParams.get() <= 0) {
            streamRateParams.set(50);
        }
        if (stream_trigger(STREAM_PARAMS)) {
            send_message(MSG_NEXT_PARAM);
        }
    }
*/

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_RAW_SENSORS)) {
        // MSG_RAW_IMU1
        // MSG_RAW_IMU2
        // MSG_RAW_IMU3
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_EXTENDED_STATUS)) {
        // MSG_EXTENDED_STATUS1
        // MSG_EXTENDED_STATUS2
        // MSG_CURRENT_WAYPOINT
        // MSG_GPS_RAW TODO - remove this message after location message is working
        // MSG_NAV_CONTROLLER_OUTPUT
        // MSG_FENCE_STATUS
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_RC_CHANNELS)) {
        // MSG_RADIO_OUT
        // MSG_RADIO_IN
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_RAW_CONTROLLER)) {
        // MSG_SERVO_OUT
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_POSITION)) {
        Mavlink_Position();
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_EXTRA1)) {
        Mavlink_Attitude();
        // MSG_SIMSTATE
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_EXTRA2)) {
        Mavlink_Hud();
    }

    if (Mavlink_Stream_Trigger((uint8_t)MAV_DATA_STREAM_EXTRA3)) {
        // MSG_AHRS
        // MSG_HWSTATUS
        // MSG_WIND
    }
}

//----------------------------------------------------------------------------
//
/// \brief   Updates stream tick and tells if it must be transmitted
/// \param   stream = ID of strem to be checked
/// \returns TRUE if stream must be transmitted
/// \remarks Working principle:
///     - if there's a "queued parameter" or we're receiving waypoints,
///       transmission frequency is divided by 4
///     - queued parameters are those requested with PARAM_REQUEST_LIST
///     - slowdown factor is always < 100 and depends on tx buffer space:
///         buffer space < 20  slowdown +3
///         buffer space < 50  slowdown +1
///         buffer space > 95  slowdown -2
///         buffer space > 90  slowdown -1
///
//----------------------------------------------------------------------------
bool Mavlink_Stream_Trigger(uint8_t stream)
{
    uint8_t rate = ucStream_Rate[stream];

    // send at a much lower rate while handling waypoints and parameter sends
/*
    if (waypoint_receiving || _queued_parameter != NULL ) {
        rate >>= 2;
    }
*/
    if (rate == 0) {
        return FALSE;
    }

    if (ucStream_Tick[stream] == 0) {   // we're triggering now,
        if (rate > MAX_STREAM_RATE) {   // setup the next trigger point
            rate = MAX_STREAM_RATE;
        }
        ucStream_Tick[stream] = (MAX_STREAM_RATE / rate) /* + stream_slowdown*/;
        return TRUE;
    }

    ucStream_Tick[stream]--;            // count down at 50Hz
    return FALSE;
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
{
   (void) piSensors;
}

//----------------------------------------------------------------------------
//
/// \brief
/// \param   -
/// \returns -
/// \remarks
///
//----------------------------------------------------------------------------
float Telemetry_Get_Gain(telEnum_Gain gain)
{
    float f_result;

    switch (gain) {
        case TEL_ROLL_KP :
        case TEL_ROLL_KI :
        case TEL_PITCH_KP :
        case TEL_PITCH_KI :
        case TEL_ALT_KP :
        case TEL_ALT_KI :
        case TEL_NAV_KP :
        case TEL_NAV_KI :
            f_result = fParam_Value[gain];
            break;

        case TEL_NAV_BANK :
            f_result = ToRad(fParam_Value[gain]);
            break;

        case TEL_GAIN_NUMBER :
        default :
            f_result = 0.0f;
            break;
    }
    return f_result;
}

//----------------------------------------------------------------------------
//
/// \brief
/// \param   -
/// \returns -
/// \remarks
///
//----------------------------------------------------------------------------
float Telemetry_Get_Speed(void)
{
    return 0.0f;    //fTrueAirSpeed;
}

//----------------------------------------------------------------------------
//
/// \brief
/// \param   -
/// \returns -
/// \remarks
///
//----------------------------------------------------------------------------
float Telemetry_Get_Altitude(void)
{
    return 0.0f;    //fAltitude;
}


