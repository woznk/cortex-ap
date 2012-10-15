//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief PPM input driver header file
///
/// \file
///
//  Change  result of merge of NAV branch:
//          modes MODE_ROLL_TUNE and MODE_PITCH_TUNE renamed MODE_STAB and
//          MODE_NAV respectively
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

#define PPM_SIGNAL_OK       6   ///< radio signal is OK
#define PPM_NO_SIGNAL       0   ///< radio signal is bad

/* Aircraft model type definitions */
#define EASYSTAR            1   ///< Easystar, 3 axis, aileron direction control
#define TYCHO               2   ///< Tycho, 2 axis, rudder only direction control
#define LEUKO               3   ///< Leuko, flying wing, rudder and aileron mixed
#define MODEL               EASYSTAR    ///< current model

/* RC channel definitions */
#if (MODEL == TYCHO)
#define AILERON_CHANNEL     0   ///< Aileron control channel
#define ELEVATOR_CHANNEL    1   ///< Elevator control channel
#define THROTTLE_CHANNEL    2   ///< Throttle control channel
#define RUDDER_CHANNEL      3   ///< Rudder control channel
#define KP_CHANNEL          4   ///< PID Kp channel
#define KI_CHANNEL          5   ///< PID Ki channel
#define RC_CHANNELS         7   ///< Number of channels, modify according to RC
#elif (MODEL == LEUKO)
#define ELEVATOR_CHANNEL    0   ///< Elevator control channel
#define DELTA1_CHANNEL      0   ///< Delta control channel
#define DELTA2_CHANNEL      1   ///< Delta control channel
#define THROTTLE_CHANNEL    2   ///< Throttle control channel
#define RUDDER_CHANNEL      3   ///< Rudder control channel
#define MODE_CHANNEL        4   ///< Mode selection channel
#define KP_CHANNEL          5   ///< PID Kp channel
#define KI_CHANNEL          6   ///< PID Ki channel
#define RC_CHANNELS         7   ///< Number of channels, modify according to RC
#elif (MODEL == EASYSTAR)
#define THROTTLE_CHANNEL    0   ///< Throttle control channel
#define ELEVATOR_CHANNEL    2   ///< Elevator control channel
#define RUDDER_CHANNEL      3   ///< Rudder control channel
#define MODE_CHANNEL        4   ///< Mode selection channel
#define AILERON_CHANNEL     5   ///< Aileron channel
#define KP_CHANNEL          6   ///< PID Kp channel
#define KI_CHANNEL          8   ///< PID Ki channel
#define RC_CHANNELS         7   ///< Number of channels, modify according to RC
#else
#error Aircraft model undefined !
#endif

/* Control mode definitions */
#define MODE_UNDEFINED      0   ///< mode is undefined
#define MODE_STAB           1   ///< stabilize roll and pitch
#define MODE_NAV            2   ///< navigate
#define MODE_MANUAL         3   ///< manual control
#define MODE_RTL            4   ///< return to launch

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

void PPM_Init(void);
uint8_t PPMSignalStatus(void);
uint8_t PPMGetMode(void);
uint16_t PPMGetChannel(uint8_t ucChannel);

