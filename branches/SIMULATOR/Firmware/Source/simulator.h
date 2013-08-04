//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief simulator interface header file
///
/// \file
///
///
//  Change
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/// Simulator message structure
typedef struct {
    uint8_t ucLength;   //!< length of message
    uint16_t *pcData;   //!< pointer to message content
} simStruct_Message;

/// PID gains
typedef enum {
    SIM_PITCH_KP = 0,   ///<
    SIM_PITCH_KI,       ///<
    SIM_ROLL_KP,        ///<
    SIM_ROLL_KI,        ///<
    SIM_NAV_KP,         ///<
    SIM_NAV_KI,         ///<
    SIM_ALT_KP,         ///<
    SIM_ALT_KI,         ///<
    SIM_GAIN_NUMBER
} simEnum_Gain;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*---------------------------------- Interface -------------------------------*/

void Simulator_Parse( void );
void Simulator_Send_Message(uint16_t *data, uint8_t num);
void Simulator_Send_DCM( void );
void Simulator_Send_Controls( void );
void Simulator_Send_Waypoint( void );
void Simulator_Send_Position( void );
void Simulator_Get_Raw_IMU(int16_t * piSensors);
float Simulator_Get_Gain(simEnum_Gain gain);
float Simulator_Get_Speed(void);
float Simulator_Get_Altitude(void);

