/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief Servo driver
 *
 * @file
 *
 * Change:
 *
 *============================================================================*/

#include "ch.h"
#include "hal.h"
#include "servo.h"

/*--------------------------------- Definitions ------------------------------*/

/*----------------------------------- Macros ---------------------------------*/

/* saturate servo position */
#define SATURATE(p)    if (p < SERVO_MIN) { p = SERVO_MIN; } \
                       if (p > SERVO_MAX) { p = SERVO_MAX; }

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/* position of servos */
static int16_t i_servo_position[SERVO_NUMBER] = {
    SERVO_NEUTRAL,  /* aileron */
    SERVO_NEUTRAL,  /* rudder */
    SERVO_NEUTRAL,  /* elevator */
    SERVO_NEUTRAL   /* throttle */
};

/* sign of servo values */
static int16_t i_servo_sign[SERVO_NUMBER] = {
     1,             /* aileron */
     1,             /* rudder */
    -1,             /* elevator */
     1              /* throttle */
};

/*
 * PWM configuration structure.
 * Cyclic callback enabled, channels 3 and 4 enabled without callbacks,
 * the active state is a logic one.
 */
static PWMConfig pwmcfg = {
  1000000,                          /* 1 MHz PWM clock frequency.  */
  20000,                            /* PWM period 20 mS (in ticks).  */
  NULL,                             /* callback function */
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  /* HW dependent part.*/
  0,
  0,
#if STM32_PWM_USE_ADVANCED
  0
#endif
};

/*--------------------------------- Prototypes -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   Timer 3 initialization for servo output
 * @return  -
 * @remarks Initializes the PWM driver 3, re-routes the TIM3 outputs,
 *          programs the pins as alternate functions.
 *
 *---------------------------------------------------------------------------*/
void Init_Servo( void ) {

  pwmStart(&PWMD3, &pwmcfg);
  palSetGroupMode(GPIOA, PAL_PORT_BIT(GPIOA_SERVO1) | PAL_PORT_BIT(GPIOA_SERVO2),
                  0,
                  PAL_MODE_STM32_ALTERNATE_PUSHPULL);
  palSetGroupMode(GPIOB, PAL_PORT_BIT(GPIOB_SERVO3) | PAL_PORT_BIT(GPIOB_SERVO4),
                  0,
                  PAL_MODE_STM32_ALTERNATE_PUSHPULL);

  pwmEnableChannelI(&PWMD3, 0, SERVO_NEUTRAL);
  pwmEnableChannelI(&PWMD3, 1, SERVO_NEUTRAL);
  pwmEnableChannelI(&PWMD3, 2, SERVO_NEUTRAL);
  pwmEnableChannelI(&PWMD3, 3, SERVO_NEUTRAL);
}

/*----------------------------------------------------------------------------
 *
 * @brief     Set servo position.
 * @param[in] servo: servo identifier
 * @param[in] position: desired position
 * @return    -
 * @remarks   -
 *
 *---------------------------------------------------------------------------*/
void Set_Servo(SERVO_TYPE servo, int16_t position) {

   SATURATE(position);
   position -= SERVO_NEUTRAL;
   position *= i_servo_sign[servo];
   position += SERVO_NEUTRAL;
   i_servo_position[servo] = position;

   pwmEnableChannelI(&PWMD3, servo, position);
}

/*----------------------------------------------------------------------------
 *
 * @brief     Get servo position.
 * @param[in] servo: servo identifier
 * @return    position of servo
 * @remarks   -
 *
 *---------------------------------------------------------------------------*/
int16_t Get_Servo(SERVO_TYPE servo) {
   return i_servo_position[servo];
}


