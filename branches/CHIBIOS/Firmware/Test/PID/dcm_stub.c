/*=============================================================================+
 *
 * $HeadURL: $
 * $Revision: $
 * $Date:  $
 * $Author: $
 *
 * @brief Direction Cosine Matrix stub
 *
 * @file
 *
 * Change
 *
 *=============================================================================+*/

#include "ch.h"

#include "config.h"
#include "dcm.h"

/*--------------------------------- Definitions ------------------------------*/

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/* aircraft pitch */
static float f_pitch = 0.0f;

/* aircraft roll */
static float f_roll = 0.0f;

/* aircraft yaw */
static float f_yaw = 0.0f;

/* semaphore */
static Semaphore sem_dcm;

/*--------------------------------- Prototypes -------------------------------*/

/*---------------------------------- Functions -------------------------------*/

/*----------------------------------------------------------------------------
 *
 * @brief   Initialize DCM
 * @return  -
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Init_DCM ( void ) {
  chSemInit(&sem_dcm, 1);   /* Semaphore initialization */
}

/*----------------------------------------------------------------------------
 *
 * @brief   Aircraft pitch.
 * @return  aircraft pitch angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float AHRS_Pitch_Rad(void)
{
  float f_temp;

  chSemWait(&sem_dcm);
  f_temp = f_pitch;
  chSemSignal(&sem_dcm);

  return (f_temp);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Aircraft roll.
 * @return  aircraft roll angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float AHRS_Roll_Rad(void)
{
  float f_temp;

  chSemWait(&sem_dcm);
  f_temp = f_roll;
  chSemSignal(&sem_dcm);

  return (f_temp);
}

/*----------------------------------------------------------------------------
 *
 * @brief   Aircraft yaw.
 * @return  aircraft roll angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
float AHRS_Yaw_Rad(void)
{
  float f_temp;

  chSemWait(&sem_dcm);
  f_temp = f_yaw;
  chSemSignal(&sem_dcm);

  return (f_temp);
}

/*----------------------------------------------------------------------------
 *
 * @brief   set aircraft pitch.
 * @return  aircraft pitch angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Set_Pitch_Rad ( float val ) {

  chSemWait(&sem_dcm);
  f_pitch = val;
  chSemSignal(&sem_dcm);
}

/*----------------------------------------------------------------------------
 *
 * @brief   set aircraft roll.
 * @return  aircraft roll angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Set_Roll_Rad ( float val ) {

  chSemWait(&sem_dcm);
  f_roll = val;
  chSemSignal(&sem_dcm);
}

/*----------------------------------------------------------------------------
 *
 * @brief   set aircraft yaw.
 * @return  aircraft roll angle [rad]
 * @remarks -
 *
 *----------------------------------------------------------------------------*/
void Set_Yaw_Rad ( float val ) {

  chSemWait(&sem_dcm);
  f_yaw = val;
  chSemSignal(&sem_dcm);
}

