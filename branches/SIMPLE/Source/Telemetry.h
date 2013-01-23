//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief telemetry interface header file
///
/// \file
///
///
//  Change function Telemetry_Get_Sensors() renamed Telemetry_Get_Raw_IMU()
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

/// telemetry message structure
typedef struct {
    uint8_t ucLength;   //!< length of message
    uint16_t *pcData;   //!< pointer to message content
} telStruct_Message;

/// PID gains
typedef enum {
    TEL_PITCH_KP = 0,   ///<
    TEL_PITCH_KI,       ///<
    TEL_ROLL_KP,        ///<
    TEL_ROLL_KI,        ///<
    TEL_NAV_KP,         ///<
    TEL_NAV_KI,         ///<
    TEL_ALT_KP,         ///<
    TEL_ALT_KI,         ///<
    TEL_GAIN_NUMBER
} telEnum_Gain;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

//VAR_GLOBAL xQueueHandle xTelemetry_Queue;

/*---------------------------------- Interface -------------------------------*/

void Telemetry_Parse( void );
void Telemetry_Send_Message(uint16_t *data, uint8_t num);
void Telemetry_Send_DCM( void );
void Telemetry_Send_Controls( void );
void Telemetry_Send_Waypoint( void );
void Telemetry_Send_Position( void );
void Telemetry_Get_Raw_IMU(int16_t * piSensors);
float Telemetry_Get_Gain(telEnum_Gain gain);
float Telemetry_Get_Speed(void);
float Telemetry_Get_Altitude(void);


