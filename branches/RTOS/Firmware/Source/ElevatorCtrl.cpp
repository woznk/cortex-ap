//============================================================================+
//
// $RCSfile: ElevatorCtrl.cpp,v $ (SOURCE FILE)
// $Revision: 1.5 $
// $Date: 2010/12/30 09:48:04 $
// $Author: Lorenz $
///
/// \brief
/// Elevator control
///
/// \file
/// If the state machine selects pitch feedback, compute it from the pitch gyro
/// and accelerometer.
///
//  CHANGES gains.h sostituito da config.h
//
//============================================================================*/

#include "stdafx.h"

#include "inc/hw_types.h"
#include "math.h"
#include "adcdriver.h"
#include "ppmdriver.h"
#include "DCM.h"
#include "gps.h"
#include "config.h"
#include "elevatorctrl.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#   undef VAR_STATIC
#endif
#define   VAR_STATIC static
#ifdef    VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

#define I_LIMIT_MIN (-0.1f)
#define I_LIMIT_MAX  (0.1f)

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL float Pitch_Kp = PITCH_KP ;
VAR_GLOBAL float Pitch_Kd = PITCH_KD ;
VAR_GLOBAL float Ail_Elv_Mix_Gain = 0.5f ;
VAR_GLOBAL float Pitch_Altitude_Adjust = 0.0f;

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC float elev_accum ;

//----------------------------------------------------------------------------
//
/// \brief   Control of aircraft elevator.
///
/// \remarks Computes elevator deflection to keep aircraft level.
///
//----------------------------------------------------------------------------
void
Elevator_Control(void) {

    float ail_elv_mix;
    float pitch_rate;

    ail_elv_mix = 0;

    // ORIGINALE : navElevMix = rmat[6] * rmat[6] * rollElevMixGain ;
    ail_elv_mix = DCM_Matrix[2][1] * DCM_Matrix[2][1] * Ail_Elv_Mix_Gain ;

    // ORIGINALE : ((rmat[8] * omegagyro[0]) - (rmat[6] * omegagyro[2])) << 1 ;
    pitch_rate = (DCM_Matrix[2][2] * Gyro_Vector[1]) - (DCM_Matrix[2][1] * Gyro_Vector[2]);

    // ORIGINALE : ((rmat[7] - rtlkick + pitchAltitudeAdjust) * Pitch_Kp ) + (Pitch_Kd * Pitch_Rate) ;
    elev_accum = ((DCM_Matrix[2][0] + Pitch_Altitude_Adjust) * Pitch_Kp) + (Pitch_Kd * pitch_rate) ;

    elev_accum += ail_elv_mix;
}

//----------------------------------------------------------------------------
//
/// \brief   Returns elevator position.
///
/// \remarks
///
///
//----------------------------------------------------------------------------
float
Elevator ( void ) {
  if (PPMGetChannel(4) < 1200) {
    return ((float)PPMGetChannel(2) - 1500.0f) / 500.0f;
  } else {
    return elev_accum;
  }
}
