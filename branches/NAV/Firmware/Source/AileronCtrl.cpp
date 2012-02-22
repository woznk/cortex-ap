//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief   Aileron control
///
/// \file
/// Il controllo degli alettoni ha lo scopo primario di stabilizzare il rollio
/// dell'aereo. Puo' essere utilizzato anche per la navigazione, in alternativa
/// al timone.
///
/// 1. Stabilizzazione rollio, MatrixPilot:
/// - il controllo e' abilitato se ROLL_STABILIZATION e pitch_feedback sono TRUE
/// - la parte proporzionale e' (rmat[6] * rollkp), dove rmat[6] e' l'angolo di
///   beccheggio
/// - la parte derivativa e' (rollkd * omegaAccum[1]), dove omegaAccum[1] e'
///   la velocita' di rollio corretta con il vettore gravita'
///
/// 2. Navigazione, MatrixPilot:
/// - la navigazione e' abilitata se AILERON_NAVIGATION e GPS_steering sono TRUE
/// - calcola seno e coseno dell'angolo tra la direzione voluta e quella reale
/// - dal segno del coseno capisce se l'angolo e' compreso tra -90 e 90 deg
/// - se l'angolo e' tra -90 e 90 deg la correzione e' yawkpail * seno angolo
/// - se l'angolo non e' tra -90 e 90 deg la correzione e' + / - (yawkpail / 4)
///
/// 3. Stabilizzazione rollio, IMUCortex:
///
/// Come Matrixpilot ???
///
/// 4. Navigazione, IMUCortex:
/// - la navigazione e' implementata con un controllo PID
/// - la parte proporzionale e' (Dir_Kp * cross_prod) dove cross_prod e' l'errore
///   di direzione, cioe' il seno dell'angolo tra direzione voluta e reale.
/// - la parte integrativa e' (Dir_Ki * (cross_prod * DELTA_T)) dove DELTA_T e'
///   l'intervallo (fisso) di calcolo del PID.
/// - la parte derivativa e'
///   (Dir_Kd * ((cross_prod - previous_error) * SAMPLES_PER_SECOND))
///   dove previous_error e' l'errore di direzione al passo precedente e la
///   moltiplicazione per SAMPLES_PER_SECOND sostituisce la divisione per
///   DELTA_T.
///
//  CHANGES gains.h sostituito da config.h
//
//============================================================================*/

#include "stdafx.h"

#include "inc/hw_types.h"
#include "math.h"
#include "adcdriver.h"
#include "DCM.h"
#include "ppmdriver.h"
#include "nav.h"
#include "config.h"
#include "aileronctrl.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#  undef VAR_STATIC
#endif
#define   VAR_STATIC static
#ifdef    VAR_GLOBAL
#  undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

//
// PID saturation limits
//
#define I_LIMIT_MIN (-0.05f)
#define I_LIMIT_MAX  (0.05f)

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumeration ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

VAR_GLOBAL float Dir_Kp     = DIR_KP;      // Direction error proportional gain
VAR_GLOBAL float Dir_Ki     = DIR_KI;      // Direction error integral gain
VAR_GLOBAL float Dir_Kd     = DIR_KD;      // Direction error derivative gain
VAR_GLOBAL float Roll_Kp    = ROLL_KP;     // Roll error proportional gain
VAR_GLOBAL float Roll_Kd    = ROLL_KD;     // Roll error derivative gain

/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC float roll_feedback ;
VAR_STATIC float previous_error = 0.0f ;
VAR_STATIC float aileron = 0.0f ;
VAR_STATIC float aileron_accum ;

//----------------------------------------------------------------------------
//
/// \brief   Control of aircraft aileron.
///
/// \remarks Computes aileron deflection to steer aircraft toward desired
///          direction.
///
//----------------------------------------------------------------------------
void
Aileron_Control(void) {

   float temp ;
//   float dot_prod ;
   float cross_prod ;
   float desired_dir ;
   float desired_x ;
   float desired_y ;
   float actual_x ;
   float actual_y ;
   float P = 0.0f, I = 0.0f, D = 0.0f;

/*
    if (PPMSignalStatus() == PPM_SIGNAL_OK) {
        aileron = pwc7 + waggle ;
    } else {
        aileron = ailerontrim + waggle ;
    }
*/

    desired_dir = (float)Nav_Bearing();

    if ( 1 /* AILERON_NAVIGATION && flags._.GPS_steering */ ) {

        // Compute X and Y components of desired direction
        desired_x = cosf( ( desired_dir * PI ) / 180.0f ) ;
        desired_y = sinf( ( desired_dir * PI ) / 180.0f ) ;

        // Get X and Y components of actual direction
        actual_x = DCM_Matrix[0][0] ;
        actual_y = DCM_Matrix[1][0] ;

        // Compute cosine of angle between desired and actual direction
/*        dot_prod = (actual_x * desired_x) + (actual_y * desired_y) ;*/

        // Compute sine of angle between desired and actual direction
        cross_prod = (actual_x * desired_y) - (actual_y * desired_x) ;

        // Proportional part.
        // PID error is sine of angle between directions.
        P = cross_prod ;

        // Integral part.
        // Avoid windup of integral part.
        if ((I > I_LIMIT_MIN) && (I < I_LIMIT_MAX)) {
            I += cross_prod * DELTA_T;
        }

        // Derivative part.
        // Multiply by SAMPLES_PER_SECOND instead of dividing by DELTA_T.
        D = (cross_prod - previous_error) * SAMPLES_PER_SECOND ;

        // Add P + I + D terms
        temp =  Dir_Kp * P;
        temp += Dir_Ki * I;
        temp += Dir_Kd * D;

        // Saturate result
        if (temp < I_LIMIT_MIN) {
          aileron_accum = I_LIMIT_MIN;
        } else if (temp > I_LIMIT_MAX) {
          aileron_accum = I_LIMIT_MAX;
        } else {
          aileron_accum = temp;
        }

        // Save current error
        previous_error = cross_prod;
    }

    if ( 1 /* ROLL_STABILIZATION && flags._.pitch_feedback */ ) {
        // Compute feedback = Kd * roll rate.
        // Omega_Vector[0] is g-corrected roll rate.
        roll_feedback = Roll_Kd * Omega_Vector[0];

        // Subtract Kp * roll angle.
        // DCM_Matrix[2][1] is roll angle.
        aileron_accum -= (Roll_Kp * DCM_Matrix[2][1]);
    } else {
        roll_feedback = 0.0f ;
    }

    aileron_accum = aileron - aileron_accum + roll_feedback;
//    PDC1 = pulsesat( aileron_accum ) ;
}

//----------------------------------------------------------------------------
//
/// \brief   aileron control interface.
/// \return  aileron position.
/// \remarks
///
//----------------------------------------------------------------------------
float
Ailerons ( void ) {
  if (PPMGetChannel(4) < 1200) {
    return ((float)PPMGetChannel(1) - 1500.0f) / 500.0f;
  } else {
    return aileron_accum;
  }
}
