//============================================================================+
//
// $RCSfile: servodriver.h,v $ (SOURCE FILE)
// $Revision: 1.4 $
// $Date: 2010/04/02 16:29:48 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             Servo driver header file
//
//  CHANGES     aggiunto underscore ai nomi delle funzioni
//
//============================================================================*/

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_STATIC
#undef VAR_STATIC
#endif
#define VAR_STATIC static
#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*----------------------------------- Types ----------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*---------------------------------- Globals ---------------------------------*/

/*----------------------------------- Locals ---------------------------------*/

/*--------------------------------- Interface --------------------------------*/

void ServoInit(void);
void ServoUpdate(void);
