/*============================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief telemetry stub
 *
 * @file
 *
 * Change:
 *
 *
 *============================================================================*/

#include "ch.h"
#include "hal.h"

#include "config.h"
#include "mavlink.h"

/*--------------------------------- Definitions ------------------------------*/

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

static float f_param_value[TEL_GAIN_NUMBER] = {
    0.0,       /* ROLL_KP  */
    0.01,      /* ROLL_KI  */
    PITCH_KP,  /* PITCH_KP */
    PITCH_KI,  /* PITCH_KI */
    1.0f,      /* 1.0f     */
    0.1f,      /* 0.1f     */
    NAV_KP,    /* NAV_KP   */
    NAV_KI,    /* NAV_KI   */
    20.0f      /* 20.0f    */
};

/*---------------------------------- Functions -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   Initialize telemetry
 * @return  -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
void Init_Telemetry( void ) {

}

/*----------------------------------------------------------------------------
 *
 * @brief   get telemetry sensors
 * @param   -
 * @returns pointer to raw sensor values
 * @remarks
 *
 *---------------------------------------------------------------------------*/
void Telemetry_Get_Sensors(int16_t * piSensors)
{
   (void) piSensors;
}

/*----------------------------------------------------------------------------
 *
 * @brief   get telemetry gains
 * @param[in] gain identifier of gain
 * @returns gain value
 * @remarks
 *
 *---------------------------------------------------------------------------*/
float Telemetry_Get_Gain(ENUM_GAIN gain)
{
    float f_result;

    switch (gain) {
        case TEL_ROLL_KP :
        case TEL_ROLL_KI :
        case TEL_PITCH_KP :
        case TEL_PITCH_KI :
        case TEL_ALT_KP :
        case TEL_ALT_KI :
        case TEL_NAV_KP :
        case TEL_NAV_KI :
            f_result = f_param_value[gain];
            break;

        case TEL_NAV_BANK :
            f_result = ToRad(f_param_value[gain]);
            break;

        case TEL_GAIN_NUMBER :
        default :
            f_result = 0.0f;
            break;
    }
    return f_result;
}

/*----------------------------------------------------------------------------
 *
 * @brief   get telemetry speed
 * @param   -
 * @returns telemetry speed [?]
 * @remarks
 *
 *---------------------------------------------------------------------------*/
float Telemetry_Get_Speed(void)
{
    return 0.0f;    /* fTrueAirSpeed; */
}

/*----------------------------------------------------------------------------
 *
 * @brief   get telemetry altitude
 * @param   -
 * @returns telemetry altitude [m]
 * @remarks
 *
 *---------------------------------------------------------------------------*/
float Telemetry_Get_Altitude(void)
{
    return 0.0f;    /* fAltitude; */
}

