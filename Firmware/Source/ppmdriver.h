//============================================================================+
//
// $RCSfile: $ (SOURCE FILE)
// $Revision: $
// $Date: $
// $Author: $
//
/// \brief
/// PPM input driver header file
///
//  CHANGES added channel definitions for different aircraft models
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

#define PPM_SIGNAL_OK       6
#define PPM_NO_SIGNAL       0

#define EASYSTAR            1
#define TYCHO               2
#define LEUKO               3
#define MODEL               LEUKO

#if (MODEL == TYCHO)
#define AILERON_CHANNEL     0   // Aileron control channel
#define ELEVATOR_CHANNEL    1   // Elevator control channel
#define THROTTLE_CHANNEL    2   // Throttle control channel
#define RUDDER_CHANNEL      3   // Rudder control channel
#define KP_CHANNEL          4   // PID Kp channel
#define KI_CHANNEL          5   // PID Ki channel
#define RC_CHANNELS         7   // Number of channels, modify according to RC
#elif (MODEL == LEUKO)
#define ELEVATOR_CHANNEL    0   // Elevator control channel
#define DELTA1_CHANNEL      0   // Delta control channel
#define DELTA2_CHANNEL      1   // Delta control channel
#define THROTTLE_CHANNEL    2   // Throttle control channel
#define RUDDER_CHANNEL      3   // Rudder control channel
#define MODE_CHANNEL        4   // Mode selection channel
#define KP_CHANNEL          5   // PID Kp channel
#define KI_CHANNEL          6   // PID Ki channel
#define RC_CHANNELS         7   // Number of channels, modify according to RC
#elif (MODEL == EASYSTAR)
#define RUDDER_CHANNEL      1   // Rudder control channel
#define ELEVATOR_CHANNEL    2   // Elevator control channel
#define MODE_CHANNEL        4   // Mode selection channel
#define RC_CHANNELS         6   // Number of channels, modify according to RC
#else
#error Aircraft model undefined !
#endif

#define MODE_UNDEFINED      0
#define MODE_ROLL_TUNE      1
#define MODE_PITCH_TUNE     2
#define MODE_MANUAL         3
#define MODE_RTL            4

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

