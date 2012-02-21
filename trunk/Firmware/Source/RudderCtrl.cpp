//============================================================================+
//
// $HeadURL: $
// $Revision: $
// $Date:  $
// $Author: $
//
/// \brief Rudder control
///
/// \file
///
//  CHANGES eliminate variabili non usate
//
//============================================================================*/

#include "stdafx.h"

#include "inc/hw_types.h"
#include "math.h"
#include "adcdriver.h"
#include "DCM.h"
#include "ppmdriver.h"
#include "nav.h"
#include "gps.h"
#include "config.h"
#include "rudderctrl.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#   undef VAR_STATIC
#endif
#define   VAR_STATIC static
#ifdef    VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

#define I_LIMIT_MIN (-2.0f)
#define I_LIMIT_MAX  (2.0f)

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/
#if 0
VAR_GLOBAL float Dir_Kp = DIR_KP ;
VAR_GLOBAL float Dir_Ki = DIR_KI ;
VAR_GLOBAL float Dir_Kd = DIR_KD ;
#endif
/*----------------------------------- Locals ---------------------------------*/

VAR_STATIC float rudder_accum = 0.0f;

//----------------------------------------------------------------------------
//
/// \brief   Control of aircraft rudder.
///
/// \remarks Computes rudder deflection to steer aircraft toward desired
///          direction.
///
//----------------------------------------------------------------------------
void
Rudder_Control(void) {
#if 0
    float temp ;
    float cross_prod ;
    float desired_dir ;
    float desired_x ;
    float desired_y ;
    float actual_x ;
    float actual_y ;
    float P = 0.0f, I = 0.0f, D = 0.0f;
/*
    if ( flags._.radio_on )    {
        rudder = pwc7 + waggle ;
    } else {
        rudder = ruddtrim + waggle ;
    }
*/

    desired_dir = 270.0f;//(float)GPSDirection();

    if ( 1 /* flags._.GPS_steering */) {

        desired_x = -sinf( ( desired_dir * PI ) / 180.0f ) ;
        desired_y = cosf( ( desired_dir * PI ) / 180.0f ) ;

        actual_x = DCM_Matrix[0][1] ; // rmat[1]
        actual_y = DCM_Matrix[1][1] ; // rmat[4]

//        dot_prod = (actual_x * desired_x) + (actual_y * desired_y) ;
        cross_prod = (actual_x * desired_y) - (actual_y * desired_x) ;

        // Proportional part (cross product is PID error)
        P = cross_prod ;

        // Integral part (avoid windup of integral part)
        if ((I > I_LIMIT_MIN) && (I < I_LIMIT_MAX)) {
            I += cross_prod * DELTA_T;
        }

        // Derivative part (multiply by SAMPLES_PER_SECOND instead of dividing by DELTA_T)
        D = (cross_prod - previous_error) * SAMPLES_PER_SECOND ;

        // Add terms
        temp =  Dir_Kp * P;
        temp += Dir_Ki * I;
        temp += Dir_Kd * D;

        rudder_accum = max(I_LIMIT_MIN, min(I_LIMIT_MAX, temp));

        // Save the actual error
        previous_error = cross_prod;
   }

    if ( 1 /* flags._.GPS_steering || flags._.pitch_feedback */ ) {
        gyro_feedback = Dir_Kd * Omega_Vector[2] ;
        //Dir_Boost = (( Dir_Boost_Gain * ( rudder - ruddtrim ) ))>>3 ;
    } else {
        gyro_feedback = 0.0f ;
        //Dir_Boost = 0.0f ;
    }

    if ( 0 /* PORTDbits.RD3 */) {
        rudder_accum = rudder_accum - gyro_feedback ; // + rudder + Dir_Boost ;
        //PDC1 = pulsesat( rudder_accum ) ;
        //rudderDeflection = ruddtrim - PDC1 ;
    } else {
        rudder_accum = rudder_accum + gyro_feedback ; // + rudder + Dir_Boost ;
        //PDC1 = pulsesat( rudder_accum ) ;
        //rudderDeflection = PDC1 - ruddtrim ;
    }
#endif
}

//----------------------------------------------------------------------------
//
/// \brief   Returns rudder position.
///
/// \remarks
///
///
//----------------------------------------------------------------------------
float
Rudder ( void ) {
  if (PPMGetChannel(4) < 1200) {
    return ((float)PPMGetChannel(3) - 1500.0f) / 500.0f;
  } else {
    return rudder_accum;
  }
}
