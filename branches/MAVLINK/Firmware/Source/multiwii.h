//============================================================================
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief multiwii telemetry header file
///
/// \file
///
///
//  Change function Telemetry_Get_Sensors() renamed Telemetry_Get_Raw_IMU()
//         function MSP_Recaive() renamed MWI_Receive()
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
    TEL_GAIN_NUMBER
} telEnum_Gain;

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*---------------------------------- Interface -------------------------------*/

void MWI_Receive(void);
void Telemetry_Get_Raw_IMU(int16_t * piSensors);
float Telemetry_Get_Gain(telEnum_Gain gain);
void Telemetry_Send_Controls(void);
float Telemetry_Get_Speed(void);
float Telemetry_Get_Altitude(void);


