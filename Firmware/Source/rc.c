/**===========================================================================+
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
 *  uc_pulse_index     aliases          content of ui_pulse_buffer[]
 *
 *        0          see rc.h           channel 0 pulse
 *        1          see rc.h           channel 1 pulse
 *        2          see rc.h           channel 2 pulse
 *        3          see rc.h           channel 3 pulse
 *        4          see rc.h           channel 4 pulse
 *        5          see rc.h           channel 5 pulse
 *        6          see rc.h           channel 6 pulse
 *        7          RC_CHANNELS        none
 *
 *  Before computing time difference, overflow number is checked.
 *  If 2 or more overflows occurred, an invalid pulse length is forced.
 *  Overflow counter is always cleared when a pulse is detected.
 *  Pulse length is saved in a temporary buffer.
 *  Temporary buffer is copied into final buffer only if pulse length
 *  of all channels are good and synchro pulse is good to.
 *  Added counter of channel pulses with correct pulse length.
 *  Counter is copied into a module variable for signal strength indication.
 *
 *  Change:
 *
 *
 *============================================================================*/

#include "ch.h"
#include "hal.h"

#include "config.h"
#include "rc.h"

/*--------------------------------- Definitions ------------------------------*/

#define PPM_SYNC_MIN        4999    /* min length of sync pulse, modify according to RC type */
#define PPM_SYNC_MAX        20001   /* max length of sync pulse, modify according to RC type */
#define PPM_PULSE_MIN       900     /* pulse length for full scale low */
#define PPM_PULSE_MAX       2100    /* pulse length for full scale high */
#define PPM_PULSE_NEUTRAL   1500    /* pulse length for neutral position */

#define PPM_PERIOD          65535   /* capture timer period */
#define PRESCALER           23      /* capture timer prescaler */

#define MANUAL_THRESHOLD_L  1100    /* lower threshold for MANUAL mode */
#define STAB_THRESHOLD_L    1400    /* lower threshold for STABILIZED mode */
#define STAB_THRESHOLD_U    1600    /* upper threshold for STABILIZED mode */
#define NAV_THRESHOLD_U     1900    /* upper threshold for NAVIGATION mode */
#define NEUTRAL_THRESHOLD_L 1450    /* lower threshold for neutral position */
#define NEUTRAL_THRESHOLD_U 1550    /* upper threshold for neutral position */

/*----------------------------------- Macros ---------------------------------*/

#define MISSING_PULSES (c_overflow_count >= 3)    /* missing pulses condition */

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

static void icuovfcb(ICUDriver *icup);      /* overflow callback */
static void icuperiodcb(ICUDriver *icup);   /* period callback */

/*----------------------------------- Locals ---------------------------------*/

static ICUConfig icucfg = {
  ICU_INPUT_ACTIVE_HIGH,
  1000000,                                   /* 1 MHz ICU clock frequency.   */
  NULL,
  icuperiodcb,
  icuovfcb,
  ICU_CHANNEL_2,
  0
};

static int8_t c_pulse_count;                    /* counter of sync pulses */
static int8_t c_signal_level;                   /* signal level estimation */
static int8_t c_overflow_count;                 /* counter of timer overflows */
static uint8_t uc_pulse_index;                  /* channel (pulse) index */
static uint16_t ui_pulse_length;                /* length of current pulse */
static uint16_t ui_temp[RC_CHANNELS];           /* temporary for channel pulses */
static uint16_t ui_pulse_buffer[RC_CHANNELS];   /* rc channel pulses */
static int16_t i_reverse[RC_CHANNELS] = {       /* channel reverse */
        /*  # | TYCHO    | LEUKO    | EASYSTAR | EPP_FPV */
        /* ---+----------+----------+----------+---------- */
    -1, /*  0 | aileron  | delta 1  | throttle | aileron */
    -1, /*  1 | elevator | delta 2  | aileron  | elevator */
     1, /*  2 | throttle | throttle | elevator | throttle */
     1, /*  3 | rudder   | rudder   | rudder   | rudder */
     1, /*  4 | kp       | mode     | mode     | mode */
     1, /*  5 | ki       | kp       | -        | kp */
     1  /*  6 | -        | ki       | kp       | ki */
};

/*---------------------------------- Functions -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   RC input capture overflow call back.
 * @return  -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
static void icuovfcb ( ICUDriver *icup ) {

    if (!MISSING_PULSES) {
        c_overflow_count++;
    }
}

/*----------------------------------------------------------------------------
 *
 * @brief   RC input capture width call back.
 * @return  -
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
static void icuperiodcb ( ICUDriver *icup ) {

  uint8_t j;

  if (c_overflow_count < 2) {                       /* at most one overflow */
     ui_pulse_length = icuGetPeriod(icup);          /* get pulse length */
  } else {                                          /* more than one overflow */
     ui_pulse_length = 0;                           /* force invalid pulse length */
  }
  c_overflow_count = 0;                             /* clear overflow condition */

  switch (uc_pulse_index) {                         /* check wich pulse */
    case RC_CHANNELS:                               /* waiting sync pulse */
      if ((ui_pulse_length > PPM_SYNC_MIN) &&       /* sync pulse detected */
          (ui_pulse_length < PPM_SYNC_MAX)) {       /* */
        uc_pulse_index = 0;                         /* reset index */
        if (c_pulse_count == RC_CHANNELS) {         /* all channels were good */
          for (j = 0; j < RC_CHANNELS; j++) {       /* copy pulse lengths */
            ui_pulse_buffer[j] = ui_temp[j];        /* */
          }
        }
      }
      c_signal_level += (16 * c_pulse_count);       /* update signal level */
      c_signal_level /= 2;
      c_pulse_count = 0;                            /* clear pulse counter */
    break;

    default:                                        /* waiting channel pulse */
      ui_temp[uc_pulse_index++] = ui_pulse_length;  /* save pulse length */
      if ((ui_pulse_length > PPM_PULSE_MIN) &&      /* good pulse length */
          (ui_pulse_length < PPM_PULSE_MAX)) {
        c_pulse_count++;                            /* increase counter */
      }
    break;
  }
}


/*----------------------------------------------------------------------------
 *
 * @brief   Timer 2 initialization for RC input capture
 * @return  -
 * @remarks TIM2CLK = 24 MHz, Prescaler = 23, TIM2 counter clock = 1 MHz
 *
 *---------------------------------------------------------------------------*/
void Init_RC(void) {

  c_pulse_count = 0;
  c_overflow_count = -1;
  for (uc_pulse_index = 0; uc_pulse_index < RC_CHANNELS; uc_pulse_index++) {
    ui_pulse_buffer[uc_pulse_index] = PPM_PULSE_NEUTRAL;
  }
  uc_pulse_index = RC_CHANNELS;

  /*
   * Initializes the ICU driver 2.
   */
  icuStart(&ICUD2, &icucfg);
  icuEnable(&ICUD2);
  chThdSleepMilliseconds(100);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get position of n-th radio channel
 * @return  Pulse length of n-th radio channel in microsecond
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
int16_t Get_RC_Channel(uint8_t uc_channel)
{
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
 * @brief   Get mode from MODE_CHANNEL
 * @return  mode (manual, stabilize, navigation, return to launch)
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
uint8_t Get_RC_Mode(void)
{
    uint16_t ui_mode;
    ui_mode = ui_pulse_buffer[MODE_CHANNEL];

    if (c_signal_level == 0) {
        return MODE_RTL;
    } else if (ui_mode < MANUAL_THRESHOLD_L) {  /* mode switch in manual */
        return MODE_MANUAL;
    } else if ((ui_mode > STAB_THRESHOLD_L) &&  /* mode switch in stabilized */
               (ui_mode < STAB_THRESHOLD_U)) {
        return MODE_STAB;
    } else if (ui_mode > NAV_THRESHOLD_U) {     /* mode switch in navigation */
        return MODE_NAV;
    } else {
        return MODE_UNDEFINED;
    }
}

/*----------------------------------------------------------------------------
 *
 * @brief   Get status of radio signal.
 * @return  level of radio signal (0 = no signal, 6 = signal OK)
 * @remarks -
 *
 *---------------------------------------------------------------------------*/
uint8_t Get_RC_Signal(void)
{
    return (uint8_t)c_signal_level;
}

