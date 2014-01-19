/*
*/

#include <stdlib.h>

#include "ch.h"
#include "hal.h"

#include "ff.h"
#include "config.h"
#include "dcm.h"
#include "imu.h"
#include "gps.h"
#include "rc.h"
#include "baro.h"
#include "servo.h"
#include "control.h"
#include "nav.h"
#include "mavlink.h"
#include "log.h"

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Prototypes -------------------------------*/

/*---------------------------------- Functions -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   Test thread
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
static WORKING_AREA(wa_Test_Thread, 128);
static msg_t Test_Thread(void *arg) {
  chRegSetThreadName("Test_Thread");
  (void)arg;
  while (TRUE) {
    chThdSleepMilliseconds(20);
    Control();                        /* control loop */
  }
  return 0;
}


/*----------------------------------------------------------------------------
 *
 * @brief   Entry point.
 * @remarks The main() function is already a thread in the system on entry.
 *
 *----------------------------------------------------------------------------*/
int main(void) {

  halInit();
  chSysInit();

  chThdSleepMilliseconds(200);
  Init_DCM();
  Init_Servo();
  Init_RC();
  Init_Control();

  /*
   * Create test thread.
   * Duration 2.34 ms
   * Period 20 ms
   */
  chThdCreateStatic(wa_Test_Thread, sizeof(wa_Test_Thread), HIGHPRIO, Test_Thread, NULL);

  /* wait system settling */
  chThdSleepSeconds(2);

  /* set roll position */
  Set_Roll_Rad(0.0872664);

  /* change mode to STAB */
  Set_RC_Channel(MODE_CHANNEL, 1500);

  /* wait system reaction */
  chThdSleepSeconds(10);

  /* change mode to MAN */
  Set_RC_Channel(MODE_CHANNEL, 1000);

  /* wait system reaction */
  chThdSleepSeconds(10);

  /* change mode to STAB */
  Set_RC_Channel(MODE_CHANNEL, 1500);

  /* wait system reaction */
  chThdSleepSeconds(10);

  /* main loop that do nothing */
  while (TRUE) {
    chThdSleepMilliseconds(500);
  }

  return 0;
}
