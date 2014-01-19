/*============================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief  Servo driver
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

}

/*----------------------------------------------------------------------------
 *
 * @brief   Set servo position.
 * @param   servo: servo identifier
 * @param   position: desired position
 * @return  -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
void Set_Servo(SERVO_TYPE servo, int16_t position) {

   SATURATE(position);
   position -= SERVO_NEUTRAL;
   position *= i_servo_sign[servo];
   position += SERVO_NEUTRAL;
   i_servo_position[servo] = position;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get servo position.
 * @param   servo: servo identifier
 * @return  position of servo
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
int16_t Get_Servo(SERVO_TYPE servo) {
   return i_servo_position[servo];
}


