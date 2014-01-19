/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief Configuration definitions and control gains
 *
 * @file
 *
 * Change:
 *
 *============================================================================*/

#define PI        3.141592f             /* PI */

#define DEGTORAD(x) (((x) * PI) / 180.0f)  /* degree to radian conversion */
#define RADTODEG(x) (((x) * 180.0f) / PI)  /* radian to degree conversion */

/* Frequency of control loops */
#define SAMPLES_PER_SECOND  50

/* DCM matrix updating interval */
#define DELTA_T         (1.0f / SAMPLES_PER_SECOND)

/*
 * Accelerometer sensitivity (source: ADXL345 datasheet)
 * Equivalent to 1 g in the raw data from accelerometer
 *  Full scale | Sensitivity [LSB/g]
 * ------------+----------------------
 *     +/-2 g  |       256
 *     +/-4 g  |       128
 *     +/-8 g  |        64
 *    +/-16 g  |        32
 */
#define GRAVITY         64              /* full scale = 8g */

/*
 * Accelerometer scale factor (source: ADXL345 datasheet)
 *  Full scale | Factor [g/digit] | Factor [mss/digit]
 * ------------+------------------+------------------------
 *     +/-2 g  |      0.0039      |      0.03822
 *     +/-4 g  |      0.0078      |      0.7644
 *     +/-8 g  |      0.0156      |      0.15288
 *    +/-16 g  |      0.0312      |      0.30576
*/
#define ACCEL_GAIN      0.15288f         /* full scale = 8g	 */

/*
 * Gyroscope scale factor (source: L3G4200 datasheet)
 *  Full scale | Factor [dps/digit] | Factor [rps/digit]
 * ------------+--------------------+----------------------
 *  +/-250 dps |       0.00875      |      0.000152716
 *  +/-500 dps |       0.01750      |      0.000366519
 * +/-2000 dps |       0.07000      |      0.001221730
 */
#define GYRO_GAIN       0.001221730f    /* full scale = 2000 dps */

/* Navigation PID initial gains */
#define NAV_KP          5.0f            /* Navigation P gain */
#define NAV_KI          0.05f           /* Navigation I gain */
#define NAV_KD          0.0f            /* Navigation D gain */
#define NAV_BANK        20.0f           /* Navigation bank angle max [deg] */

/* Speed PID initial gains */
#define SPEED_KP        0.99f           /* Speed P gain */
#define SPEED_KI        0.1f            /* Speed I gain */
#define SPEED_KD        0.0f            /* Speed D gain */

/* Altitude PID initial gains */
#define ALT_KP          0.99f           /* Altitude P gain */
#define ALT_KI          0.1f            /* Altitude I gain */
#define ALT_KD          0.0f            /* Altitude D gain */

/* Attitude roll PID initial gains */
#define ROLL_KP         0.99f           /* Roll P gain */
#define ROLL_KI         0.1f            /* Roll I gain */
#define ROLL_KD         0.0f            /* Roll D gain */

/* Attitude pitch PID initial gains */
#define PITCH_KP        0.99f           /* Pitch P gain */
#define PITCH_KI        0.1f            /* Pitch P gain */
#define PITCH_KD        0.0f            /* Pitch D gain */

/*
 * Angle that the nose of the plane will pitch downward during
 * a return to launch, used to increase speed (and wind penetration).
 * Set it to zero to disable this feature.
 */
#define RTL_PITCH_DOWN  0.0f            /* Return to launch pitch down [deg] */

/* comment out this line if you are not going to use altitude hold */
#define ALTITUDEHOLD                    /* altitude hold option */

/*
 * The range of altitude within which to linearly vary the throttle
 * and pitch to maintain altitude. Bigger values makes altitude hold
 * smoother, and is suggested for very fast planes.
 */
#define HEIGHT_MARGIN   20.0f           /* altitude hold margin */

/*
 * Use ALT_HOLD_THROTTLE_MAX when below HEIGHT_MARGIN of the target height.
 * Interpolate between ALT_HOLD_THROTTLE_MAX and ALT_HOLD_THROTTLE_MIN
 * when within HEIGHT_MARGIN of the target height.
 * Use ALT_HOLD_THROTTLE_MIN when above HEIGHT_MARGIN of the target height.
 * Throttle values are from -1.0 to +1.0
 */
#define ALT_HOLD_THROTTLE_MIN   0.0f        /* min throttle for altitude hold */
#define ALT_HOLD_THROTTLE_MAX   0.8f        /* max throttle for altitude hold */

#define HEIGHTMAX               1500.0f     /* maximum target height [m] */
#define HEIGHT_MARGIN           20.0f       /* height margin [m] */

#define MINIMUMTHROTTLE         -0.3f       /* minimum throttle */

#define PITCHATMINTHROTTLE      DEGTORAD(0.1f) /* Pitch angle at minimum throttle [rad] */
#define PITCHATMAXTHROTTLE      DEGTORAD(8.0f) /* Pitch angle at maximum throttle [rad] */
#define PITCHATZEROTHROTTLE     DEGTORAD(-6.0f)/* Pitch angle while gliding [rad] */

/* Simulator option definitions */
#define SIM_NONE    0                   /* No simulator */
#define XPLANE      1                   /* Simulator X-Plane */
#define FLIGHTGEAR  2                   /* Simulator Flightgear */
#define SIMULATOR   SIM_NONE            /* Current simulator option */

/* Sensor type definitions for multiwii protocol */
#define ACC         1                   /* Accelerometer available */
#define MAG         0                   /* Magnetometer not available */
#define GYRO        1                   /* Gyrometer available */
#define BARO        1                   /* Barometer available */
#define GPS_SERIAL  1                   /* GPS serial available */
#define GPS_BAUD    57600               /* GPS Baud rate */
#define GPS         1                   /* GPS available */
#define SONAR       0                   /* Sonar not available */

/* Multiwii waypoint support */
#define USE_MWI_WP	1

/* Telemetry type definition */
#define TELEMETRY_MAVLINK

/* Type of aircraft */
/* define GIMBAL */
/* define BI */
/* define TRI */
/* define QUADP */
/* define QUADX */
/* define Y4 */
/* define Y6 */
/* define HEX6 */
/* define HEX6X */
/* define HEX6H */  /* New Model */
/* define OCTOX8 */
/* define OCTOFLATP */
/* define OCTOFLATX */
/* define FLYING_WING */
/* define VTAIL4 */
#define AIRPLANE
/* define SINGLECOPTER */
/* define DUALCOPTER */
/* define HELI_120_CCPM */
/* define HELI_90_DEG */

/* Aircraft model */
#define EASYSTAR            1   /* Easystar, 3 axis, aileron direction control */
#define TYCHO               2   /* Tycho, 2 axis, rudder only direction control */
#define LEUKO               3   /* Leuko, flying wing, rudder and aileron mixed */
#define EPPFPV              4   /* Hobby King EPP FPV, 3 axis, aileron direction control */
#define MODEL          EPPFPV   /* current model */

/* RC channel definitions */
#if (MODEL == TYCHO)
#define AILERON_CHANNEL     0   /* Aileron control channel */
#define ELEVATOR_CHANNEL    1   /* Elevator control channel */
#define THROTTLE_CHANNEL    2   /* Throttle control channel */
#define RUDDER_CHANNEL      3   /* Rudder control channel */
#define KP_CHANNEL          4   /* PID Kp channel */
#define KI_CHANNEL          5   /* PID Ki channel */
#define RC_CHANNELS         7   /* Number of channels, modify according to RC */
#elif (MODEL == LEUKO)
#define ELEVATOR_CHANNEL    0   /* Elevator control channel */
#define DELTA1_CHANNEL      0   /* Delta control channel */
#define DELTA2_CHANNEL      1   /* Delta control channel */
#define THROTTLE_CHANNEL    2   /* Throttle control channel */
#define RUDDER_CHANNEL      3   /* Rudder control channel */
#define MODE_CHANNEL        4   /* Mode selection channel */
#define KP_CHANNEL          5   /* PID Kp channel */
#define KI_CHANNEL          6   /* PID Ki channel */
#define RC_CHANNELS         7   /* Number of channels, modify according to RC */
#elif (MODEL == EASYSTAR)
#define THROTTLE_CHANNEL    0   /* Throttle control channel */
#define ELEVATOR_CHANNEL    2   /* Elevator control channel */
#define RUDDER_CHANNEL      3   /* Rudder control channel */
#define MODE_CHANNEL        4   /* Mode selection channel */
#define AILERON_CHANNEL     1   /* Aileron channel */
#define KP_CHANNEL          6   /* PID Kp channel */
#define KI_CHANNEL          8   /* PID Ki channel */
#define RC_CHANNELS         7   /* Number of channels, modify according to RC */
#elif (MODEL == EPPFPV)
#define AILERON_CHANNEL     0   /* Aileron channel */
#define ELEVATOR_CHANNEL    1   /* Elevator control channel */
#define THROTTLE_CHANNEL    2   /* Throttle control channel */
#define RUDDER_CHANNEL      3   /* Rudder control channel */
#define MODE_CHANNEL        4   /* Mode selection channel */
#define KP_CHANNEL          5   /* PID Kp channel */
#define KI_CHANNEL          6   /* PID Ki channel */
#define RC_CHANNELS         7   /* Number of channels, modify according to RC */
#else
#error Aircraft model undefined !
#endif

