/**============================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief Direction Cosine Matrix calculations
 *
 * @file
 * Meaning of f_dcm_matrix rows / columns:
 * \code
 *         col    0              1               2
 *     row
 *                |              |               |
 *      0    -----|--------------|---------------|-----> earth X axis (west)
 *                |              |               |
 *      1    -----|--------------|---------------|-----> earth Y axis (north)
 *                |              |               |
 *      2    -----|--------------|---------------|-----> earth Z axis (down)
 *                |              |               |
 *                v              v               v
 *           plane X axis   plane Y axis    plane Z axis
 *
 * \endcode
 * The first row of the matrix represent the projection of the earth X axis
 * on the X, Y, and Z axes of the aircraft.
 *
 * The second row of the matrix represent the projection of the earth Y axis
 * on the axes of the aircraft.
 *
 * The third row of the matrix represent the projection of the earth Z axis
 * on the axes of the aircraft.
 *
 * Meaning of f_dcm_matrix elements:
 *
 * \code
 *          col    0                1                 2
 *      row
 *
 *       0   cos(Xp ^ Xe)    cos(Yp ^ Xe)     cos(Zp ^ Xe)
 *
 *       1   cos(Xp ^ Ye)    cos(Yp ^ Ye)     cos(Zp ^ Ye)
 *
 *       2   cos(Xp ^ Ze)    cos(Yp ^ Ze)     cos(Zp ^ Ze)
 *
 * \endcode
 *
 * where cos(Xp ^ Xe) is the cosine of the angle between plane X axis and
 * earth X axis, cos(Yp ^ Xe) is the cosine of the angle between plane Y
 * axis and earth X axis, and so on.
 *
 * Following cosines are mostly relevant:
 *
 * DCM[2][0] = cosine of the angle between aircraft X axis and earth Z axis.
 * It is equal to zero when the aircraft X axis is level with earth XY plane
 * i.e. the aircraft is longitudinally levelled, pitch angle = 0.
 *
 * DCM[2][1] = cosine of the angle between aircraft Y axis and earth Z axis.
 * It is equal to zero when the aircraft Y axis is level with earth XY plane
 * i.e. the aircraft is laterally levelled, roll angle = 0.
 *
 * DCM[1][1] = cosine of the angle between aircraft Y axis and earth Y axis.
 * It is equal to one when aircraft Y axis is aligned with earth Y axis.
 *
 * DCM is the identity matrix when the aircraft is sitting level on the ground
 * facing north.
 *
 * Meaning of f_gyro_vector elements:
 *
 * \code
 *
 *     element          meaning     UDB variable
 *
 *     f_gyro_vector[0]   roll rate   omegagyro[1]
 *     f_gyro_vector[1]   pitch rate  omegagyro[0]
 *     f_gyro_vector[2]   yaw rate    omegagyro[2]
 *
 * \endcode
 *
 * Change
 *
 *=============================================================================+*/

#include "ch.h"

#include "config.h"
#include "math.h"
#include "vmath.h"
#include "gps.h"
#include "DCM.h"

/*--------------------------------- Definitions ------------------------------*/

/* Initial P gain for roll/pitch compensation */
/* Typical values 0.1f, 0.015f, 0.01f, 0.0013f */
#define PITCHROLL_KP    0.1f

/* Initial I gain for roll/pitch compensation*/
/* Typical values 0.000005f, 0.000002f */
#define PITCHROLL_KI    0.000005f

/* Initial P gain for yaw compensation */
/* Typical values 0.5f, 0.27f */
#define YAW_KP          0.5f

/* Initial P gain for yaw compensation */
/* Typical values 0.0005f */
#define YAW_KI          0.0005f

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/* Direction Cosine Matrix*/
static float f_dcm_matrix[3][3] =
{
    { 1.0f, 0.0f, 0.0f },
    { 0.0f, 1.0f, 0.0f },
    { 0.0f, 0.0f, 1.0f }
};

/* Raw gyroscope data*/
static float f_gyro_vector[3] =
{ 0.0f, 0.0f, 0.0f };

/* g-corrected gyroscope data*/
static float f_omega_vector[3] =
{ 0.0f, 0.0f, 0.0f };

/* Conversion gain from ADC to angular speed in deg/s*/
static float f_gyro_gain = GYRO_GAIN;

/* Conversion gain from ADC to acceleration in m/s/s*/
static float f_accel_gain = ACCEL_GAIN;

/* Proportional gain roll/pitch compensation*/
static float f_pitchroll_Kp = PITCHROLL_KP;

/* Integral gain roll/pitch compensation*/
static float f_pitchroll_Ki = PITCHROLL_KI;

/* Proportional gain yaw compensation*/
static float f_yaw_Kp = YAW_KP;

/* Integral gain yaw compensation*/
static float f_yaw_Ki = YAW_KI;

/* Velocity 3D*/
static float f_ground_speed = 0.0f;

/*----------------------------------- Locals ---------------------------------*/

/* Gyros here*/
static float f_upd_matrix[3][3] = {
    { 1.0f, 0.0f, 0.0f },
    { 0.0f, 1.0f, 0.0f },
    { 0.0f, 0.0f, 1.0f }
};

/* Temporary matrix*/
static float f_temp_matrix[3][3] = {
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f },
    { 0.0f, 0.0f, 0.0f }
};

/* Acceleration vector*/
static float f_accel_vector[3] =
    { 0.0f, 0.0f, 0.0f };

/* Temporary for intermediate calculation*/
static float f_omega[3] =
    { 0.0f, 0.0f, 0.0f };

/* f_omega proportional correction*/
static float f_omega_p[3] =
    { 0.0f, 0.0f, 0.0f };

/* f_omega integral correction*/
static float f_omega_i[3] =
    { 0.0f, 0.0f, 0.0f };

/* roll/pitch error vector*/
static float f_error_rollpitch[3] =
    { 0.0f, 0.0f, 0.0f };

/* yaw error vector*/
static float f_error_yaw[3] =
    { 0.0f, 0.0f, 0.0f };

/* course error in deg*/
static float f_error_course = 180.0f;

/* Course overground X axis*/
static float f_cog_x = 1.0f;

/* Course overground Y axis*/
static float f_cog_y = 0.0f;

static float f_scaled_omega_p[3];

static float f_scaled_omega_i[3];

/* aircraft pitch */
static float f_pitch = 0.0f;

/* aircraft roll */
static float f_roll = 0.0f;

/* aircraft yaw */
static float f_yaw = 0.0f;

/* semaphore */
static Semaphore sem_dcm;

/*--------------------------------- Prototypes -------------------------------*/

__inline static void accel_adjust(void);

/*---------------------------------- Functions -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   Initialize DCM
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Init_DCM ( void ) {

  chSemInit(&sem_dcm, 1);   /* Semaphore initialization */
}

/*----------------------------------------------------------------------------
 *
 * @brief   Normalize DCM matrix
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Normalize(void)
{
    float error = 0.0f;
    float renorm = 0.0f;
    /*
                          error                         error
       Xorthogonal = X - ------- Y , Yorthogonal = Y - ------- X        Eq. 19
                            2                             2
    */
    error = -VectorDotProduct(&f_dcm_matrix[0][0], &f_dcm_matrix[1][0]) * 0.5f;
    VectorScale(&f_temp_matrix[0][0], &f_dcm_matrix[1][0], error);
    VectorScale(&f_temp_matrix[1][0], &f_dcm_matrix[0][0], error);
    VectorAdd(&f_temp_matrix[0][0], &f_temp_matrix[0][0], &f_dcm_matrix[0][0]);
    VectorAdd(&f_temp_matrix[1][0], &f_temp_matrix[1][0], &f_dcm_matrix[1][0]);

    /*
       Zorthogonal = X orthogonal /\ Yorthogonal                        Eq. 20
    */
    VectorCrossProduct(&f_temp_matrix[2][0], &f_temp_matrix[0][0], &f_temp_matrix[1][0]);

    /*
                    1
      Xnormalized = - (3 - Xorthogonal . Xorthogonal) Xorthogonal
                    2

                    1
      Yormalized =  - (3 - Yorthogonal . Yorthogonal) Yorthogonal      Eq. 21
                    2

                   1
      Znormalized = - (3 - Zorthogonal . Zorthogonal) Zorthogonal
                    2
    */
    renorm = 0.5f * (3.0f - VectorDotProduct(&f_temp_matrix[0][0], &f_temp_matrix[0][0]));
    VectorScale(&f_dcm_matrix[0][0], &f_temp_matrix[0][0], renorm);
    renorm = 0.5f * (3.0f - VectorDotProduct(&f_temp_matrix[1][0], &f_temp_matrix[1][0]));
    VectorScale(&f_dcm_matrix[1][0], &f_temp_matrix[1][0], renorm);
    renorm = 0.5f * (3.0f - VectorDotProduct(&f_temp_matrix[2][0], &f_temp_matrix[2][0]));
    VectorScale(&f_dcm_matrix[2][0], &f_temp_matrix[2][0], renorm);

    /* update pitch, roll, yaw */
    chSemWait(&sem_dcm);
    f_pitch = -asinf(f_dcm_matrix[2][0]);
    f_roll = asinf(f_dcm_matrix[2][1]);
    f_yaw = atan2f(f_dcm_matrix[1][0], f_dcm_matrix[0][0]);
    chSemSignal(&sem_dcm);
}


/*----------------------------------------------------------------------------
 *
 * @brief   Adjust acceleration
 * @return  -
 * @remarks Computes reference acceleration as
 *                                                                 \code
 * g         = Accelerometer + A                 (Eq. 26)
 *  reference                   centrifugal                        \endcode
 *
 * where
 *                                                                 \code
 * A           = omega     /\ V                  (Eq. 25)
 *  centrifugal       gyro                                         \endcode
 *
 * and the velocity vector has only the x component
 *                                                                 \code
 *     | velocity |
 *     |          |
 * V = |    0     |
 *     |          |
 *     |    0     |                                                \endcode
 *
 * The acceleration is then scaled from ADC values to g.
 *
 *----------------------------------------------------------------------------*/
__inline static void accel_adjust(void)
{
#if (SIMULATOR == SIM_NONE)
    f_ground_speed = (float)Gps_Speed_Kt();
    f_ground_speed = (f_ground_speed * 1852.0f) / 36000.0f; /* convert [kt] to [m/s] */
#else
    f_ground_speed = Simulator_Get_Speed();
#endif
    f_accel_vector[1] += ((f_ground_speed * f_omega[2] * 9.81f) / GRAVITY);
    f_accel_vector[2] -= ((f_ground_speed * f_omega[1] * 9.81f) / GRAVITY);
}


/*----------------------------------------------------------------------------
 *
 * Compensate for roll / pitch / yaw drift
 * @return  -
 * @remarks La correzione di rollio e beccheggio e' data da: \code
 *                        | Rzx |
 *                        |     |
 * RollPitch correction = | Rzy | /\ g
 *                        |     |     reference
 *                        | Rzz |
 * \endcode
 *
 * La correzione di imbardata e' calcolata prima nel sistema di riferimento di
 * terra: \code
 * Yaw correction (ground) = xb  /\ COG
 *                             p
 * \endcode dove \code
 *
 * xb
 *   p
 *
 * \endcode e' la proiezione dell'asse X dell'aereo sul piano X Y del sistema
 * di riferimento di terra, e \code COG \endcode (Course Over Ground) e' la
 * proiezione della prua dell'aereo sul piano X Y del sistema di riferimento
 * di terra. La correzione di imbardata viene trasformata nel sistema di
 * riferimento dell'aereo:
 * \code
 *                                                       | Rzx |
 *                                                       |     |
 * Yaw correction (aircraft) = Yaw correction (ground) . | Rzy |
 *                                                       |     |
 *                                                       | Rzz |
 * \endcode
 *
 *----------------------------------------------------------------------------*/
void CompensateDrift( void )
{
    float fCourse_Over_Ground;

    /* RollPitch correction */
    VectorCrossProduct(&f_error_rollpitch[0], &f_accel_vector[0], &f_dcm_matrix[2][0]);

    VectorScale(&f_omega_p[0], &f_error_rollpitch[0], f_pitchroll_Kp);
    VectorScale(&f_scaled_omega_i[0], &f_error_rollpitch[0], f_pitchroll_Ki);
    VectorAdd(f_omega_i, f_omega_i, f_scaled_omega_i);

    /* Course over ground */
    fCourse_Over_Ground = (float)Gps_COG_Deg();
    f_cog_x = cosf(DEGTORAD(fCourse_Over_Ground));
    f_cog_y = sinf(DEGTORAD(fCourse_Over_Ground));

    /* Yaw correction (ground) */
    f_error_course=(f_dcm_matrix[0][0] * f_cog_y) - (f_dcm_matrix[1][0] * f_cog_x);

    /* Yaw correction (aircraft) */
    VectorScale(f_error_yaw, &f_dcm_matrix[2][0], f_error_course);

    /* YAW proportional gain. */
    VectorScale(&f_scaled_omega_p[0], &f_error_yaw[0], f_yaw_Kp);

    /* Adding proportional. */
    VectorAdd(f_omega_p, f_omega_p, f_scaled_omega_p);

    /* YAW integral gain. */
    VectorScale(&f_scaled_omega_i[0], &f_error_yaw[0], f_yaw_Ki);

    /* Adding integral to the f_omega_i */
    VectorAdd(f_omega_i, f_omega_i, f_scaled_omega_i);
}


/*----------------------------------------------------------------------------
 *
 * @brief   Update DCM matrix
 * @return  -
 * @remarks
 *
 *----------------------------------------------------------------------------*/
void MatrixUpdate(const int16_t *sensor)
{
    /* Indexes for multiplication */
    uint8_t x, y;

    /* Accelerometer signals */
    f_accel_vector[0] = f_accel_gain * (*sensor++);   /* accel x */
    f_accel_vector[1] = f_accel_gain * (*sensor++);   /* accel y */
    f_accel_vector[2] = f_accel_gain * (*sensor++);   /* accel z */

    /* Gyro signals */
    f_gyro_vector[0] = f_gyro_gain * (*sensor++);     /* omega x */
    f_gyro_vector[1] = f_gyro_gain * (*sensor++);     /* omega y */
    f_gyro_vector[2] = f_gyro_gain * (*sensor);       /* omega z */

    /* adding integral */
    VectorAdd(&f_omega[0], &f_gyro_vector[0], &f_omega_i[0]);

    /* adding proportional */
    VectorAdd(&f_omega_vector[0], &f_omega[0], &f_omega_p[0]);

    /* adjust centrifugal acceleration. */
    accel_adjust();

    f_upd_matrix[0][0] = 0.0f;
    f_upd_matrix[0][1] = -DELTA_T * f_omega_vector[2];   /* -z */
    f_upd_matrix[0][2] =  DELTA_T * f_omega_vector[1];   /*  y */
    f_upd_matrix[1][0] =  DELTA_T * f_omega_vector[2];   /*  z */
    f_upd_matrix[1][1] = 0.0f;
    f_upd_matrix[1][2] = -DELTA_T * f_omega_vector[0];   /* -x */
    f_upd_matrix[2][0] = -DELTA_T * f_omega_vector[1];   /* -y */
    f_upd_matrix[2][1] =  DELTA_T * f_omega_vector[0];   /*  x */
    f_upd_matrix[2][2] = 0.0f;

    /* Update DCM matrix */
    MatrixMultiply((const float (*)[3])f_dcm_matrix, (const float (*)[3])f_upd_matrix, f_temp_matrix);
    for ( x = 0; x < 3; x++ ) {
        for ( y = 0; y < 3; y++ ) {
            f_dcm_matrix[x][y] += f_temp_matrix[x][y];
        }
    }
}


/*----------------------------------------------------------------------------
 *
 * @brief   Aircraft pitch.
 * @return  aircraft pitch angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float AHRS_Pitch_Rad(void)
{
  float f_temp;

  chSemWait(&sem_dcm);
  f_temp = f_pitch;
  chSemSignal(&sem_dcm);

  return (f_temp);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Aircraft roll.
 * @return  aircraft roll angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float AHRS_Roll_Rad(void)
{
  float f_temp;

  chSemWait(&sem_dcm);
  f_temp = f_roll;
  chSemSignal(&sem_dcm);

  return (f_temp);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Aircraft yaw.
 * @return  aircraft roll angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float AHRS_Yaw_Rad(void)
{
  float f_temp;

  chSemWait(&sem_dcm);
  f_temp = f_yaw;
  chSemSignal(&sem_dcm);

  return (f_temp);
}

