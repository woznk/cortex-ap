//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief Configuration definitions and control gains
///
/// \file
///
// CHANGES modified altitude hold parameters, 
//         rad <-> deg conversion macros moved here from DCM.c
//
//============================================================================*/

#define PI        3.141592f

#define ToRad(x) (((x) * PI) / 180.0f)
#define ToDeg(x) (((x) * 180.0f) / PI)

/* Frequency of attitude control loop */
#ifdef _WINDOWS
#  define SAMPLES_PER_SECOND  10
#else
#  define SAMPLES_PER_SECOND  40
#endif

/* DCM matrix updating interval */
#define DELTA_T         (1.0f / SAMPLES_PER_SECOND)

/* Equivalent to 1 g in the raw data from accelerometer */
//#define GRAVITY         256           // full scale = 2g
#define GRAVITY         64              // full scale = 8g

/* Accelerometer conversion factor to [m/s/s] */
//#define ACCEL_GAIN      0.03828125f   // full scale = 2g
#define ACCEL_GAIN      0.153125f       // full scale = 8g

/* Gyroscope conversion factor to [rad/s] */
//#define GYRO_GAIN       0.0001527163f // full scale = 250 dps
#define GYRO_GAIN       0.0012217f      // full scale = 2000 dps

/* Navigation PID initial gains */
#define NAV_KP          5.0f            //! Navigation P gain
#define NAV_KI          0.05f           //! Navigation I gain
#define NAV_KD          0.0f            //! Navigation D gain

/* Speed PID initial gains */
#define SPEED_KP        1.0f            //! Speed P gain
#define SPEED_KI        0.1f            //! Speed I gain
#define SPEED_KD        0.0f            //! Speed D gain

/* Altitude PID initial gains */
#define ALT_KP          1.0f            //! Altitude P gain
#define ALT_KI          0.1f            //! Altitude I gain
#define ALT_KD          0.0f            //! Altitude D gain

/* Attitude roll PID initial gains */
#define ROLL_KP         1.0f            //! Roll P gain
#define ROLL_KI         0.1f            //! Roll I gain
#define ROLL_KD         0.0f            //! Roll D gain

/* Attitude pitch PID initial gains */
#define PITCH_KP        1.0f            //! Pitch P gain
#define PITCH_KI        0.1f            //! Pitch P gain
#define PITCH_KD        0.0f            //! Pitch D gain

/* Angle that the nose of the plane will pitch downward during
   a return to launch, used to increase speed (and wind penetration).
   Set it to zero to disable this feature. */
#define RTL_PITCH_DOWN  0.0f            //! Return to launch pitch down [deg]

/* comment out this line if you are not going to use altitude hold */
#define ALTITUDEHOLD

/* The range of altitude within which to linearly vary the throttle
   and pitch to maintain altitude. Bigger values makes altitude hold
   smoother, and is suggested for very fast planes. */
#define HEIGHT_MARGIN   20.0f

/* Use ALT_HOLD_THROTTLE_MAX when below HEIGHT_MARGIN of the target height.
   Interpolate between ALT_HOLD_THROTTLE_MAX and ALT_HOLD_THROTTLE_MIN
   when within HEIGHT_MARGIN of the target height.
   Use ALT_HOLD_THROTTLE_MIN when above HEIGHT_MARGIN of the target height.
   Throttle values are from -1.0 to +1.0.*/
#define ALT_HOLD_THROTTLE_MIN   -0.2f
#define ALT_HOLD_THROTTLE_MAX   0.4f

#define HEIGHTMAX           1500.0f     //! maximum target height [m]

#define HEIGHT_MARGIN       20.0f       //! height margin [m]

#define MINIMUMTHROTTLE     -0.3f       //! minimum throttle

#define PITCHATMINTHROTTLE  ToRad(0.1f) //! Pitch angle at minimum throttle [rad]

#define PITCHATMAXTHROTTLE  ToRad(8.0f) //! Pitch angle at maximum throttle [rad]

#define PITCHATZEROTHROTTLE ToRad(-6.0f)//! Pitch angle while gliding [rad]

#define SIM_NONE    0                   // No simulator
#define XPLANE      1                   // Simulator X-Plane
#define FLIGHTGEAR  2                   // Simulator Flightgear

#define SIMULATOR   XPLANE
