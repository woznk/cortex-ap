//=============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
///
/// \brief Direction Cosine Matrix calculations
///
/// \file
/// Meaning of DCM_Matrix rows / columns:
/// \code
///         col    0              1               2
///     row
///                |              |               |
///      0    -----|--------------|---------------|-----> earth X axis (west)
///                |              |               |
///      1    -----|--------------|---------------|-----> earth Y axis (north)
///                |              |               |
///      2    -----|--------------|---------------|-----> earth Z axis (down)
///                |              |               |
///                v              v               v
///           plane X axis   plane Y axis    plane Z axis
///
/// \endcode
/// The first row of the matrix represent the projection of the earth X axis
/// on the X, Y, and Z axes of the aircraft.
///
/// The second row of the matrix represent the projection of the earth Y axis
/// on the axes of the aircraft.
///
/// The third row of the matrix represent the projection of the earth Z axis
/// on the axes of the aircraft.
///
/// Meaning of DCM_Matrix elements:
///
/// \code
///          col    0                1                 2
///      row
///
///       0   cos(Xp ^ Xe)    cos(Yp ^ Xe)     cos(Zp ^ Xe)
///
///       1   cos(Xp ^ Ye)    cos(Yp ^ Ye)     cos(Zp ^ Ye)
///
///       2   cos(Xp ^ Ze)    cos(Yp ^ Ze)     cos(Zp ^ Ze)
///
/// \endcode
///
/// where cos(Xp ^ Xe) is the cosine of the angle between plane X axis and
/// earth X axis, cos(Yp ^ Xe) is the cosine of the angle between plane Y
/// axis and earth X axis, and so on.
///
/// Following cosines are mostly relevant:
///
/// DCM[2][0] = cosine of the angle between aircraft X axis and earth Z axis.
/// It is equal to zero when the aircraft X axis is level with earth XY plane
/// i.e. the aircraft is longitudinally levelled, pitch angle = 0.
///
/// DCM[2][1] = cosine of the angle between aircraft Y axis and earth Z axis.
/// It is equal to zero when the aircraft Y axis is level with earth XY plane
/// i.e. the aircraft is laterally levelled, roll angle = 0.
///
/// DCM[1][1] = cosine of the angle between aircraft Y axis and earth Y axis.
/// It is equal to one when aircraft Y axis is aligned with earth Y axis.
///
/// DCM is the identity matrix when the aircraft is sitting level on the ground
/// facing north.
///
/// Meaning of Gyro_Vector elements:
///
/// \code
///
///     element          meaning     UDB variable
///
///     Gyro_Vector[0]   roll rate   omegagyro[1]
///     Gyro_Vector[1]   pitch rate  omegagyro[0]
///     Gyro_Vector[2]   yaw rate    omegagyro[2]
///
/// \endcode
///
// Change: added option for multiwii telemetry or ardupilot-like telemetry
//
//=============================================================================+

#include "stm32f10x.h"

#include "math.h"
#include "vmath.h"
/* uncomment telemetry type that applies */
#include "telemetry.h"
//#include "multiwii.h"
#include "config.h"
#include "nav.h"
#include "DCM.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

/// Initial P gain for roll/pitch compensation
#define PITCHROLL_KP    0.03f       // Typical values 0.1f, 0.015f, 0.01f, 0.0013f

/// Initial I gain for roll/pitch compensation
#define PITCHROLL_KI    0.000005f   // Typical values 0.000005f, 0.000002f

/// Initial P gain for yaw compensation
#define YAW_KP          0.5f        // Typical values 0.5f, 0.27f

/// Initial P gain for yaw compensation
#define YAW_KI          0.0005f     // Typical values 0.0005f

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/// Direction Cosine Matrix
VAR_GLOBAL float DCM_Matrix[3][3] =
{
    { 1.0f, 0.0f, 0.0f },
    { 0.0f, 1.0f, 0.0f },
    { 0.0f, 0.0f, 1.0f }
};

/// Raw gyroscope data
VAR_GLOBAL float Gyro_Vector[3] =
{ 0.0f, 0.0f, 0.0f };

/// g-corrected gyroscope data
VAR_GLOBAL float Omega_Vector[3] =
{ 0.0f, 0.0f, 0.0f };

/// Conversion gain from ADC to angular speed in deg/s
VAR_GLOBAL float Gyro_Gain = GYRO_GAIN;

/// Conversion gain from ADC to acceleration in m/s/s
VAR_GLOBAL float Accel_Gain = ACCEL_GAIN;

/// Proportional gain roll/pitch compensation
VAR_GLOBAL float PitchRoll_Kp = PITCHROLL_KP;

/// Integral gain roll/pitch compensation
VAR_GLOBAL float PitchRoll_Ki = PITCHROLL_KI;

/// Proportional gain yaw compensation
VAR_GLOBAL float Yaw_Kp = YAW_KP;

/// Integral gain yaw compensation
VAR_GLOBAL float Yaw_Ki = YAW_KI;

/// Velocity 3D
VAR_GLOBAL float fGround_Speed = 0.0f;

/*----------------------------------- Locals ---------------------------------*/

/// Gyros here
float Update_Matrix[3][3] = {
    { 1.0f, 0.0f, 0.0f },
    { 0.0f, 1.0f, 0.0f },
    { 0.0f, 0.0f, 1.0f }
};

/// Temporary matrix
float Temporary_Matrix[3][3] = {
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f }
};

/// Acceleration vector
float Accel_Vector[3] =
    { 0.0f, 0.0f, 0.0f };

/// Temporary for intermediate calculation
float Omega[3] =
    { 0.0f, 0.0f, 0.0f };

/// Omega proportional correction
float Omega_P[3] =
    { 0.0f, 0.0f, 0.0f };

/// Omega integral correction
float Omega_I[3] =
    { 0.0f, 0.0f, 0.0f };

/// roll/pitch error vector
float errorRollPitch[3] =
    { 0.0f, 0.0f, 0.0f };

/// yaw error vector
float errorYaw[3] =
    { 0.0f, 0.0f, 0.0f };

/// course error in deg
float errorCourse = 180.0f;

/// Course overground X axis
float COGX = 1.0f;

/// Course overground Y axis
float COGY = 0.0f;

/*--------------------------------- Prototypes -------------------------------*/

///----------------------------------------------------------------------------
///
/// Normalize DCM matrix
/// \return      -
/// \remarks
///
///----------------------------------------------------------------------------
void
Normalize(void)
{
    float error = 0.0f;
    float renorm = 0.0f;
    float temporary[3][3] = {
       { 0.0f, 0.0f, 0.0f },
       { 0.0f, 0.0f, 0.0f },
       { 0.0f, 0.0f, 0.0f }
    };

    //
    //                    error                         error
    // Xorthogonal = X - ------- Y , Yorthogonal = Y - ------- X        Eq. 19
    //                      2                             2
    //
    error = -VectorDotProduct(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * 0.5f;
    VectorScale(&temporary[0][0], &DCM_Matrix[1][0], error);
    VectorScale(&temporary[1][0], &DCM_Matrix[0][0], error);
    VectorAdd(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);
    VectorAdd(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);

    //
    // Zorthogonal = X orthogonal /\ Yorthogonal                        Eq. 20
    //
    VectorCrossProduct(&temporary[2][0], &temporary[0][0], &temporary[1][0]);

    //
    //               1
    // Xnormalized = - (3 - Xorthogonal . Xorthogonal) Xorthogonal
    //               2
    //
    //               1
    // Yormalized =  - (3 - Yorthogonal . Yorthogonal) Yorthogonal      Eq. 21
    //               2
    //
    //               1
    // Znormalized = - (3 - Zorthogonal . Zorthogonal) Zorthogonal
    //               2
    //
    renorm = 0.5f * (3.0f - VectorDotProduct(&temporary[0][0], &temporary[0][0]));
    VectorScale(&DCM_Matrix[0][0], &temporary[0][0], renorm);
    renorm = 0.5f * (3.0f - VectorDotProduct(&temporary[1][0], &temporary[1][0]));
    VectorScale(&DCM_Matrix[1][0], &temporary[1][0], renorm);
    renorm = 0.5f * (3.0f - VectorDotProduct(&temporary[2][0], &temporary[2][0]));
    VectorScale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

///----------------------------------------------------------------------------
///
/// Adjust acceleration
/// \return  -
/// \remarks Computes reference acceleration as
///                                                                 \code
/// g         = Accelerometer + A                 (Eq. 26)
///  reference                   centrifugal                        \endcode
///
/// where
///                                                                 \code
/// A           = omega     /\ V                  (Eq. 25)
///  centrifugal       gyro                                         \endcode
///
/// and the velocity vector has only the x component
///                                                                 \code
///     | velocity |
///     |          |
/// V = |    0     |
///     |          |
///     |    0     |                                                \endcode
///
/// The acceleration is then scaled from ADC values to g.
///
///----------------------------------------------------------------------------
void
AccelAdjust(void)
{
#if (SIMULATOR == SIM_NONE)
    fGround_Speed = ((float)Gps_Speed());
    fGround_Speed = (fGround_Speed * 1852.0f) / 36000.0f; // convert [kt] to [m/s]
#else
    fGround_Speed = Telemetry_Get_Speed();
#endif
    Accel_Vector[1] += ((fGround_Speed * Omega[2] * 9.81f) / GRAVITY);
    Accel_Vector[2] -= ((fGround_Speed * Omega[1] * 9.81f) / GRAVITY);
}


///----------------------------------------------------------------------------
///
/// Compensate for roll / pitch / yaw drift
/// \return  -
/// \remarks La correzione di rollio e beccheggio e' data da: \code
///                        | Rzx |
///                        |     |
/// RollPitch correction = | Rzy | /\ g
///                        |     |     reference
///                        | Rzz |
/// \endcode
///
/// La correzione di imbardata e' calcolata prima nel sistema di riferimento di
/// terra: \code
/// Yaw correction (ground) = xb  /\ COG
///                             p
/// \endcode dove \code
///
/// xb
///   p
///
/// \endcode e' la proiezione dell'asse X dell'aereo sul piano X Y del sistema
/// di riferimento di terra, e \code COG \endcode (Course Over Ground) e' la
/// proiezione della prua dell'aereo sul piano X Y del sistema di riferimento
/// di terra. La correzione di imbardata viene trasformata nel sistema di
/// riferimento dell'aereo:
/// \code
///                                                       | Rzx |
///                                                       |     |
/// Yaw correction (aircraft) = Yaw correction (ground) . | Rzy |
///                                                       |     |
///                                                       | Rzz |
/// \endcode
///
///----------------------------------------------------------------------------
void
CompensateDrift( void )
{
    static float Scaled_Omega_P[3];
    static float Scaled_Omega_I[3];
    float fCourse_Over_Ground;

    // RollPitch correction
    VectorCrossProduct(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[2][0]);

    VectorScale(&Omega_P[0], &errorRollPitch[0], PitchRoll_Kp);
    VectorScale(&Scaled_Omega_I[0], &errorRollPitch[0], PitchRoll_Ki);
    VectorAdd(Omega_I, Omega_I, Scaled_Omega_I);

    //
    // Course over ground
    //
    fCourse_Over_Ground = (float)Gps_Heading();
    COGX = cosf(ToRad(fCourse_Over_Ground));
    COGY = sinf(ToRad(fCourse_Over_Ground));

    //
    // Yaw correction (ground)
    //
    errorCourse=(DCM_Matrix[0][0] * COGY) - (DCM_Matrix[1][0] * COGX);

    //
    // Yaw correction (aircraft)
    //
    VectorScale(errorYaw, &DCM_Matrix[2][0], errorCourse);

    //
    // YAW proportional gain.
    //
    VectorScale(&Scaled_Omega_P[0], &errorYaw[0], Yaw_Kp);

    //
    // Adding proportional.
    //
    VectorAdd(Omega_P, Omega_P, Scaled_Omega_P);

    //
    // YAW integral gain.
    //
    VectorScale(&Scaled_Omega_I[0], &errorYaw[0], Yaw_Ki);

    //
    // Adding integral to the Omega_I
    //
    VectorAdd(Omega_I, Omega_I, Scaled_Omega_I);
}


///----------------------------------------------------------------------------
///
/// Update DCM matrix
/// \return      -
/// \remarks
///
///----------------------------------------------------------------------------
void
MatrixUpdate(int16_t *sensor)
{
    //
    // Indexes for multiplication
    //
    int x, y;

    //
    // Accelerometer signals
    //
    Accel_Vector[0] = Accel_Gain * (*sensor++);   // accel x
    Accel_Vector[1] = Accel_Gain * (*sensor++);   // accel y
    Accel_Vector[2] = Accel_Gain * (*sensor++);   // accel z
    //
    // Gyro signals
    //
    Gyro_Vector[0] = Gyro_Gain * (*sensor++);     // omega x
    Gyro_Vector[1] = Gyro_Gain * (*sensor++);     // omega y
    Gyro_Vector[2] = Gyro_Gain * (*sensor);       // omega z

    //
    // adding integral
    //
    VectorAdd(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);

    //
    // adding proportional
    //
    VectorAdd(&Omega_Vector[0], &Omega[0], &Omega_P[0]);

    //
    // adjust centrifugal acceleration.
    //
    AccelAdjust();

    Update_Matrix[0][0] = 0.0f;
    Update_Matrix[0][1] = -DELTA_T * Omega_Vector[2];   // -z
    Update_Matrix[0][2] =  DELTA_T * Omega_Vector[1];   //  y
    Update_Matrix[1][0] =  DELTA_T * Omega_Vector[2];   //  z
    Update_Matrix[1][1] = 0.0f;
    Update_Matrix[1][2] = -DELTA_T * Omega_Vector[0];   // -x
    Update_Matrix[2][0] = -DELTA_T * Omega_Vector[1];   // -y
    Update_Matrix[2][1] =  DELTA_T * Omega_Vector[0];   //  x
    Update_Matrix[2][2] = 0.0f;

    //
    //  Update DCM matrix
    //
    MatrixMultiply(DCM_Matrix, Update_Matrix, Temporary_Matrix);
    for ( x = 0; x < 3; x++ ) {
        for ( y = 0; y < 3; y++ ) {
            DCM_Matrix[x][y] += Temporary_Matrix[x][y];
        }
    }
}
