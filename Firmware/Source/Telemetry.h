//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief Simulation interface
/// \file
///
//  CHANGES result of merge of NAV branch:
//          corrected enum of PID gains,
//          removed and renamed interface functions 
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

typedef struct {
    uint8_t ucLength;
    uint16_t *pcData;
} telStruct_Message;

typedef enum {
    TEL_PITCH_KP = 0,
    TEL_PITCH_KI,
    TEL_ROLL_KP,
    TEL_ROLL_KI,
    TEL_NAV_KP,
    TEL_NAV_KI,
    TEL_GAIN_NUMBER
} telEnum_Gain;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

//VAR_GLOBAL xQueueHandle xTelemetry_Queue;

/*---------------------------------- Interface -------------------------------*/

void Telemetry_Task( void *pvParameters );
void Telemetry_Get_Sensors(int16_t * piSensors);
float Telemetry_Get_Gain(telEnum_Gain gain);
void Telemetry_Send_Controls(void);
float Telemetry_Get_Speed(void);


