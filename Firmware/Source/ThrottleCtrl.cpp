//============================================================================+
//
// $RCSfile: ThrottleCtrl.cpp,v $ (SOURCE FILE)
// $Revision: 1.3 $
// $Date: 2010/12/30 10:00:51 $
// $Author: Lorenz $
//
/// \brief
///
/// \file
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
#include "throttlectrl.h"

/*--------------------------------- Definitions ------------------------------*/

#ifdef    VAR_STATIC
#   undef VAR_STATIC
#endif
#define   VAR_STATIC static
#ifdef    VAR_GLOBAL
#   undef VAR_GLOBAL
#endif
#define   VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

//----------------------------------------------------------------------------
//
/// \brief Control of aircraft throttle.
///
/// \remarks
///
//----------------------------------------------------------------------------
void
Throttle_Control(void) {

}

//----------------------------------------------------------------------------
//
/// \brief   Returns throttle position.
///
/// \remarks
///
///
//----------------------------------------------------------------------------
float
Throttle ( void ) {
  if (PPMGetChannel(4) < 1200) {
    return ((float)PPMGetChannel(0) - 1500.0f) / 500.0f;
  } else {
    return 0.2f;
  }
}
