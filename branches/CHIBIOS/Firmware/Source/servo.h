/**===========================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief Servo driver header file
 *
 * @file
 *
 * Change
 *
 *
 *============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#define SERVO_MIN       900  /* Absolute minimum pulse length (0.9 ms) */
#define SERVO_MAX      2100  /* Absolute maximum pulse length (2.1 ms) */
#define SERVO_NEUTRAL  1500  /* Pulse length of servo neutral position (1.5 ms) */

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

typedef enum {
    SERVO_AILERON,
    SERVO_RUDDER,
    SERVO_ELEVATOR,
    SERVO_THROTTLE,
    SERVO_NUMBER
} SERVO_TYPE;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Interface --------------------------------*/

void Init_Servo( void );
void Set_Servo(SERVO_TYPE servo, int16_t position);
int16_t Get_Servo(SERVO_TYPE servo);
