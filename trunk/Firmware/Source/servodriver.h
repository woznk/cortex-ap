//============================================================================+
//
// $RCSfile: $
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief  Servo driver header file
//  CHANGES changed values for SERVO_MIN and SERVO_MAX
//
//============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

#define SERVO_MIN      1000  ///< Absolute minimum pulse length (0.9 ms).
#define SERVO_MAX      2000  ///< Absolute maximum pulse length (2.1 ms).
#define SERVO_NEUTRAL  1500  ///< Pulse length of servo neutral position (1.5 ms).

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

typedef enum {
    SERVO_AILERON,
    SERVO_RUDDER,
    SERVO_ELEVATOR,
    SERVO_THROTTLE
} SERVO_TYPE;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Interface --------------------------------*/

void Servo_Init(void);
void Servo_Set(SERVO_TYPE servo, int16_t position);
