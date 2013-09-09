//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief
/// Tentative Mavlink protocol implementation
///
///
/// Changes: added TEL_NAV_BANK parameter, maximum bank angle during navigation
///
//============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/// PID gains
typedef enum {
    TEL_ROLL_KP = 0,    ///<
    TEL_ROLL_KI,        ///<
    TEL_PITCH_KP,       ///<
    TEL_PITCH_KI,       ///<
    TEL_ALT_KP,         ///<
    TEL_ALT_KI,         ///<
    TEL_NAV_KP,         ///<
    TEL_NAV_KI,         ///<
    TEL_NAV_BANK,       ///<
    TEL_GAIN_NUMBER
} telEnum_Gain;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*---------------------------------- Interface -------------------------------*/

void Mavlink_Receive(void);
void Mavlink_Stream_Send(void);
void Mavlink_Queued_Send(uint8_t cycles);
float Telemetry_Get_Gain(telEnum_Gain gain);
void Telemetry_Get_Sensors(int16_t * piSensors);
float Telemetry_Get_Speed(void);
float Telemetry_Get_Altitude(void);

