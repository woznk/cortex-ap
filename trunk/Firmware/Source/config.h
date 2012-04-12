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
// CHANGES updated default values of PID gains
//
//============================================================================*/

#define PI          3.141592f

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

/* Limits servo throw by controlling pulse width saturation.
   Set it to 1.0 if you want full servo throw, otherwise set it
   to the portion that you want */
#define SERVOSAT        1.0f            //! Limits servo throw

/* comment out this line if you are not going to use altitude hold,
   to avoid spurious interrupts from the unused PWM channel */
#define ALTITUDEHOLD

#define HEIGHTMAX       300.0f          //! maximum target height [m]

#define MINIMUMTHROTTLE 0.35f           //! minimum throttle

#define PITCHATMINTHROTTLE  0.0f        //! Pitch angle at minimum throttle [deg]

#define PITCHATMAXTHROTTLE 15.0f        //! Pitch angle at maximum throttle [deg]

#define PITCHATZEROTHROTTLE 0.0f        //! Pitch angle while gliding [deg]

#define SIM_NONE    0                   // No simulator
#define XPLANE      1                   // Simulator X-Plane
#define FLIGHTGEAR  2                   // Simulator Flightgear

#define SIMULATOR   XPLANE
