/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @file
 *
 * @brief aircraft control
 *
 * Reference point for pitch and roll stabilization is computed as
 *
 *     -asinf(DCM[...][...])
 *
 * instead of
 *
 *     acosf(DCM[...][...]) - PI/2
 *
 * This returns an angle value that's consistent with angle convention for
 * roll and pitch angles, without need to either subtract PI/2 from reference
 * point or to add PI/2 to the set point.
 *
 * @todo
 * replace SERVO_NEUTRAL with RC command center position value
 *
 * Change: restored sensor calibration
 *
 *============================================================================*/

#include "ch.h"
#include "hal.h"
#include "math.h"

#include "config.h"
#include "dcm.h"
/* #include "simulator.h" */
#include "mavlink.h"
#include "nav.h"
#include "pid.h"
#include "servo.h"
#include "rc.h"
#include "control.h"

/** @addtogroup cortex_ap
  * @{
  */

/** @addtogroup control
  * @{
  */

/*--------------------------------- Definitions ------------------------------*/

#if (SIMULATOR == SIM_NONE)
  #define Get_Gain Telemetry_Get_Gain
#else
  #define Get_Gain Simulator_Get_Gain
#endif

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

static int16_t i_aileron;                           /* aileron servo position */
static int16_t i_elevator;                          /* elevator servo position */
static int16_t i_throttle;                          /* throttle servo position */
static xPID Roll_Pid;                               /* roll PID */
static xPID Pitch_Pid;                              /* pitch PID */
static xPID Nav_Pid;                                /* navigation PID */
static float f_temp;                                /* */
static float f_output;                              /* PID output */
static float f_pitch = 0.0f;                        /* pitch */
static float f_roll = 0.0f;                         /* roll */
static float f_heading = 0.0f;                      /* heading */
static float f_height_margin = HEIGHT_MARGIN;        /* altitude hold margin */
static float f_throttle = MINIMUMTHROTTLE;           /* commanded throttle */
static float f_throttle_min = ALT_HOLD_THROTTLE_MIN; /* altitude hold min throttle */
static float f_throttle_max = ALT_HOLD_THROTTLE_MAX; /* altitude hold max throttle */

/*--------------------------------- Prototypes -------------------------------*/

__inline static void stabilize( void );
__inline static void direction_control( void );
__inline static void altitude_control( void );
__inline static void camera_control( void );

/*--------------------------------- Functions --------------------------------*/


/*----------------------------------------------------------------------------
 *
 * @brief   Initialize control loops.
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Init_Control(void )
{
    Roll_Pid.fGain = 500.0f;            /* limit servo throw */
    Roll_Pid.fMin = -1.0f;              /* */
    Roll_Pid.fMax = 1.0f;               /* */
    Roll_Pid.fKp = ROLL_KP;             /* init gains with default values */
    Roll_Pid.fKi = ROLL_KI;             /* */
    Roll_Pid.fKd = ROLL_KD;             /* */

    Pitch_Pid.fGain = 500.0f;           /* limit servo throw */
    Pitch_Pid.fMin = -1.0f;             /* */
    Pitch_Pid.fMax = 1.0f;              /* */
    Pitch_Pid.fKp = PITCH_KP;           /* init gains with default values */
    Pitch_Pid.fKi = PITCH_KI;           /* */
    Pitch_Pid.fKd = PITCH_KD;           /* */

    Nav_Pid.fGain = DEGTORAD(NAV_BANK);    /* limit bank angle during navigation */
    Nav_Pid.fMin = -1.0f;               /* */
    Nav_Pid.fMax = 1.0f;                /* */
    Nav_Pid.fKp = NAV_KP;               /* init gains with default values */
    Nav_Pid.fKi = NAV_KI;               /* */
    Nav_Pid.fKd = NAV_KD;               /* */

    PID_Init(&Roll_Pid);                /* initialize PIDs */
    PID_Init(&Pitch_Pid);
    PID_Init(&Nav_Pid);
}


/*----------------------------------------------------------------------------
 *
 * @brief   Control
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Control ( void ) {

    static uint8_t uc_old_mode = MODE_MANUAL;
    uint8_t uc_current_mode;
    float f_max_roll;

    /* update PID gains */
#if (0)
    Pitch_Pid.fKp = Get_Gain(TEL_PITCH_KP);
    Pitch_Pid.fKi = Get_Gain(TEL_PITCH_KI);
    Roll_Pid.fKp = Get_Gain(TEL_ROLL_KP);
    Roll_Pid.fKi = Get_Gain(TEL_ROLL_KI);
    Nav_Pid.fKp = Get_Gain(TEL_NAV_KP);
    Nav_Pid.fKi = Get_Gain(TEL_NAV_KI);
    Nav_Pid.fGain = Get_Gain(TEL_NAV_BANK);
#endif
    f_max_roll = Get_Gain(TEL_NAV_BANK);

    Roll_Pid.fKp = (float)(Get_RC_Channel(KP_CHANNEL) - 1000) / 500.0f;
    Roll_Pid.fKi = (float)(Get_RC_Channel(KI_CHANNEL) - 1000) / 1000.0f;
	if (Roll_Pid.fKp < 0.0f ) { Roll_Pid.fKp = 0.0f; }
	if (Roll_Pid.fKi < 0.0f ) { Roll_Pid.fKi = 0.0f; }

    /* read RC commands */
    i_aileron = Get_RC_Channel(AILERON_CHANNEL);
    i_elevator = Get_RC_Channel(ELEVATOR_CHANNEL);
    i_throttle = Get_RC_Channel(THROTTLE_CHANNEL);

    /* read aircraft attitude */
    f_heading = AHRS_Yaw_Rad( );
    f_pitch = AHRS_Pitch_Rad( );
    f_roll = AHRS_Roll_Rad( );

    /* limit roll */
    if (f_roll < -f_max_roll) {
        f_roll = -f_max_roll;
    } else if (f_roll > f_max_roll) {
        f_roll = f_max_roll;
    }

    uc_current_mode = Get_RC_Mode();

    switch (uc_current_mode) {

        case MODE_STAB:             /* STABILIZED MODE */
            stabilize();
            break;

        case MODE_NAV:              /* NAVIGATION MODE */
            direction_control();
            altitude_control();
            break;

        case MODE_FPV:              /* FPV MODE */
            camera_control();
            break;

        case MODE_MANUAL:           /* MANUAL MODE */
            /* state entry transition */
            if (uc_old_mode != MODE_MANUAL) {
                /* reset PID controllers */
                PID_Init(&Roll_Pid);
                PID_Init(&Pitch_Pid);
                PID_Init(&Nav_Pid);
            }
            break;

        case MODE_RTL:
        default:
            break;
    }

    uc_old_mode = uc_current_mode;

    /* update controls */
    Set_Servo(SERVO_AILERON, i_aileron);
    Set_Servo(SERVO_ELEVATOR, i_elevator);
    Set_Servo(SERVO_THROTTLE, i_throttle);
}


/*----------------------------------------------------------------------------
 *
 * @brief   stabilize aircraft
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
__inline static void stabilize( void ) {

  i_aileron -= SERVO_NEUTRAL;
  i_elevator -= SERVO_NEUTRAL;

  /* pitch control */
  Pitch_Pid.fSetpoint = ((float)i_elevator / 500.0f);    /* setpoint for pitch */
  Pitch_Pid.fInput = f_pitch;
  f_output = PID_Compute(&Pitch_Pid);                    /* pitch PID */
  i_elevator = SERVO_NEUTRAL + (int16_t)f_output;

  /* roll control */
  Roll_Pid.fSetpoint = ((float)i_aileron / 500.0f);      /* setpoint for bank */
  Roll_Pid.fInput = f_roll;
  f_output = PID_Compute(&Roll_Pid);                     /* roll PID */
  i_aileron = SERVO_NEUTRAL + (int16_t)f_output;
}

/*----------------------------------------------------------------------------
 *
 * @brief   control aircraft direction
 * @return  -
 * @remarks implements two nested PI(D) loops: outermost loop compensates the
 *          error between aircraft heading and bearing to next waypoint,
 *          innermost loop controls aircraft's roll.
 *
 *----------------------------------------------------------------------------*/
__inline static void direction_control( void ) {

  /* compute direction error */
  f_temp = f_heading - Nav_Bearing_Rad();

  /* limit direction error between [-PI, PI] */
  if (f_temp < -PI) {
     f_temp = f_temp + (2.0f * PI);
  } else if (f_temp > PI) {
     f_temp = f_temp - (2.0f * PI);
  }

  /* normalize direction error */
  f_temp = f_temp / PI;

  /* direction control */
  Nav_Pid.fSetpoint = f_temp;
  Nav_Pid.fInput = 0.0f;
  f_temp = PID_Compute(&Nav_Pid);                   /* direction PID is .. */

  /* roll control */
  Roll_Pid.fSetpoint = f_temp;                      /* .. roll setpoint */
  Roll_Pid.fInput = f_roll;
  f_output = PID_Compute(&Roll_Pid);                /* roll PID */
  i_aileron = SERVO_NEUTRAL + (int16_t)f_output;
}

/*----------------------------------------------------------------------------
 *
 * @brief   altitude control
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
__inline static void altitude_control( void ) {

    /* get altitude error */
    f_temp = Nav_Alt_Error();

    /* interpolate throttle and pitch */
    if (f_temp > f_height_margin) {             /* we're too high */
        f_throttle = f_throttle_min;            /* minimum throttle */
        f_temp = PITCHATMINTHROTTLE;
    } else if (f_temp < -f_height_margin) {     /* we're too low */
        f_throttle = f_throttle_max;            /* max throttle */
        f_temp = PITCHATMAXTHROTTLE;
    } else {                                    /* interpolate */
        f_temp = (f_temp - f_height_margin) / (2.0f * f_height_margin);
        f_throttle = f_temp * (f_throttle_min - f_throttle_max) + f_throttle_min;
        f_temp = f_temp * (PITCHATMINTHROTTLE - PITCHATMAXTHROTTLE) + PITCHATMINTHROTTLE;
    }

    /* set throttle */
    i_throttle = SERVO_NEUTRAL + (int16_t)(500.0f * f_throttle);

    /* pitch control */
    Pitch_Pid.fInput = f_pitch;
    Pitch_Pid.fSetpoint = f_temp;
    f_output = PID_Compute(&Pitch_Pid);             /* pitch PID */
    i_elevator = SERVO_NEUTRAL + (int16_t)f_output;
}

/*----------------------------------------------------------------------------
 *
 * @brief   camera control
 * @return  -
 * @remarks roll and pitch angles are simply output to aileron servo and
 *          elevator servo respectively.
 *
 *----------------------------------------------------------------------------*/
__inline static void camera_control( void ) {

  f_temp = -(f_pitch * 1800.0f) / PI;            /* current pitch (reversed) */
  i_elevator = SERVO_NEUTRAL + (int16_t)f_temp;  /* show on elevator servo */
  f_temp = -(f_roll * 1800.0f) / PI;             /* current bank (reversed) */
  i_aileron = SERVO_NEUTRAL + (int16_t)f_temp;   /* show on aileron servo */
}

/**
  * @}
  */

/**
  * @}
  */

/*****END OF FILE****/

