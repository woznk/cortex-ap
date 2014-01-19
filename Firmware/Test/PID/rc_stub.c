/*============================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief Remote control PPM input
 *
 * @file
 *
 *
 * Change:
 *
 *
 *============================================================================*/

#include "ch.h"
#include "hal.h"

#include "config.h"
#include "rc.h"

/*--------------------------------- Definitions ------------------------------*/

#define PPM_PULSE_MIN       900     /* length of channel pulse for full scale low */
#define PPM_PULSE_MAX       2100    /* length of channel pulse for full scale high */
#define PPM_PULSE_NEUTRAL   1500    /* length of channel pulse for neutral position */

#define MANUAL_THRESHOLD_L  1100    /* lower threshold for MANUAL mode */
#define STAB_THRESHOLD_L    1400    /* lower threshold for STABILIZED mode */
#define STAB_THRESHOLD_U    1600    /* upper threshold for STABILIZED mode */
#define NAV_THRESHOLD_U     1900    /* upper threshold for NAVIGATION mode */

#define REST_THRESHOLD_L    1450    /* lower threshold for rest position */
#define REST_THRESHOLD_U    1550    /* upper threshold for rest position */

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

static int8_t c_signal_level = 10;               /* signal level estimation */
static uint16_t ui_pulse_buffer[RC_CHANNELS] = { /* rc channel pulses */
    PPM_PULSE_NEUTRAL,
    PPM_PULSE_NEUTRAL,
    PPM_PULSE_NEUTRAL,
    PPM_PULSE_NEUTRAL,
    MANUAL_THRESHOLD_L - 10,
    PPM_PULSE_NEUTRAL,
    PPM_PULSE_NEUTRAL
};

static int16_t i_reverse[RC_CHANNELS] = {        /* channel reverse */
        /*  # | TYCHO    | LEUKO    | EASYSTAR | EPPFPV    */
        /* ---+----------+----------+----------+---------- */
    -1, /*  0 | aileron  | delta 1  | throttle | aileron   */
    -1, /*  1 | elevator | delta 2  | aileron  | elevator  */
     1, /*  2 | throttle | throttle | elevator | throttle  */
     1, /*  3 | rudder   | rudder   | rudder   | rudder    */
    -1, /*  4 | kp       | mode     | mode     | mode      */
     1, /*  5 | ki       | kp       | -        | -         */
     1  /*  6 | -        | ki       | kp       | kp        */
};

/*---------------------------------- Functions -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   Initialize RC decoding
 * @return  -
 * @remarks
 *
 *---------------------------------------------------------------------------*/
void Init_RC ( void ) {
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get position of n-th radio channel
 * @return  Pulse length of n-th radio channel in microsecond
 * @remarks
 *
 *---------------------------------------------------------------------------*/
int16_t Get_RC_Channel(uint8_t uc_channel) {

  int16_t position;

  if ( uc_channel < RC_CHANNELS ) {
    position = (int16_t)ui_pulse_buffer[uc_channel];
    position -= PPM_PULSE_NEUTRAL;
    position *= i_reverse[uc_channel];
    position += PPM_PULSE_NEUTRAL;
  } else {
    position = PPM_PULSE_NEUTRAL;
  }
	return position;
}

/*----------------------------------------------------------------------------
 *
 * @brief   Set position of n-th radio channel
 * @return
 * @remarks
 *
 *---------------------------------------------------------------------------*/
void Set_RC_Channel(uint8_t uc_channel, int16_t position) {

  if ( uc_channel < RC_CHANNELS ) {
    ui_pulse_buffer[uc_channel] = position;
  } else {
  }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get mode from MODE_CHANNEL
 * @return  -
 * @remarks
 *
 *---------------------------------------------------------------------------*/
uint8_t Get_RC_Mode(void) {

    uint16_t ui_mode;
    ui_mode = ui_pulse_buffer[MODE_CHANNEL];

    if (c_signal_level == 0) {
        return MODE_RTL;
    } else if (ui_mode < MANUAL_THRESHOLD_L) {      /* mode switch in manual */
        return MODE_MANUAL;
    } else if ((ui_mode > STAB_THRESHOLD_L) &&      /* mode switch in stabilized */
               (ui_mode < STAB_THRESHOLD_U)) {
        return MODE_STAB;
    } else if (ui_mode > NAV_THRESHOLD_U) {         /* mode switch in navigation */
        return MODE_NAV;
    } else {
        return MODE_UNDEFINED;
    }
}


/*----------------------------------------------------------------------------
 *
 * @brief     Returns status of radio signal.
 *
 * @remarks   -
 *
 *---------------------------------------------------------------------------*/
uint8_t Get_RC_Signal(void)
{
  return (uint8_t)c_signal_level;
}

