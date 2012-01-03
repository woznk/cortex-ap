//============================================================================+
//
// $RCSfile: config.h,v $ (HEADER FILE)
// $Revision: 1.3 $
// $Date: 2011/02/02 18:25:20 $
// $Author: Lorenz $
//
/// \brief Configuration definitions and control gains
///
/// \file
///
// CHANGES Added gains for different full scale values
//
//============================================================================*/

#define PI          3.141592f

//! Frequenza di campionamento dell'ADC
#ifdef _WINDOWS
#  define SAMPLES_PER_SECOND  10
#else
#  define SAMPLES_PER_SECOND  10
#endif

//! Intervallo di ricalcolo della matrice DCM
#define DELTA_T         (1.0f / SAMPLES_PER_SECOND)

//! Valore iniziale fattore di conversione da ADC a [m/s/s]
//#define ACCEL_GAIN      0.03828125f   // full scale = 2g
#define ACCEL_GAIN      0.153125f     // full scale = 8g

//! Valore iniziale fattore di conversione da ADC a [rad/s]
//#define GYRO_GAIN       0.0001527163f // full scale = 250 dps 
#define GYRO_GAIN       0.0012217f    // full scale = 2000 dps

//! P feedback gain for steering, around 0.1 (Matrixpilot YAWKP, 0.0625)
#define DIR_KP          0.03f

//! I feedback gain for steering (Matrixpilot -)
#define DIR_KI          0.04f

//! D feedback gain for steering (Matrixpilot -)
#define DIR_KD          0.25f

//! Equivalent to 1 g in the raw ADC data from accelerometer
//#define GRAVITY         256    // full scale = 2g
#define GRAVITY         64     // full scale = 8g

//! P feedback gain for roll leveling, around 0.25 (Matrixpilot ROLLKP, 0.25)
#define ROLL_KP         0.1f

//! Rate feedback gain for roll damping, around 0.125 (Matrixpilot ROLLKD, 0.12 * SCALEGYRO)
#define ROLL_KD         0.1f

//! P feedback gain for pitch leveling, around 0.125 (Matrixpilot PITCHGAIN, -)
#define PITCH_KP        0.5f

//! D feedback gain for pitch damping, around 0.0625 (Matrixpilot PITCHGAIN, -)
#define PITCH_KD        0.03f

//! Pitch boost (optional, I do not use it myself), around 0.25 (Matrixpilot PITCHBOOST, -)
#define PITCH_BOOST     0.0f

//! Return to launch pitch down in degrees
#define RTL_PITCH_DOWN  0.0f
/// Angle that the nose of the plane will pitch downward during a return to launch,
/// used to increase speed (and wind penetration). Set it to zero to disable this feature.

//! Limits servo throw by controlling pulse width saturation.
#define SERVOSAT        1.0f
/// Set it to 1.0 if you want full servo throw, otherwise set it to the portion that you want

#define ALTITUDEHOLD
/// comment out this line if you are not going to use altitude hold,
/// to avoid spurious interrupts from the unused PWM channel

//! maximum target height in meters
#define HEIGHTMAX       100.0f

//! minimum throttle
#define MINIMUMTHROTTLE 0.35f

//! Target pitch angle in degrees at minimum throttle
#define PITCHATMINTHROTTLE  0.0f

//! Target pitch angle in degrees at maximum throttle
#define PITCHATMAXTHROTTLE 15.0f

//! Target pitch angle in degrees while gliding
#define PITCHATZEROTHROTTLE 0.0f

#define SIM_NONE    0   // No simulator
#define XPLANE      1   // Simulator X-Plane
#define FLIGHTGEAR  2   // Simulator Flightgear

#define SIMULATOR   SIM_NONE
