//============================================================================
//
// $RCSfile: nav.h,v $ (HEADER FILE)
// $Revision: 1.2 $
// $Date: 2010/01/25 21:42:02 $
// $Author: Lorenz $
//
//  LANGUAGE    C
//  DESCRIPTION
/// \file
///             Navigation manager header file
//  CHANGES     
//
//============================================================================

/*--------------------------------- Definitions ------------------------------*/

#ifdef VAR_GLOBAL
#undef VAR_GLOBAL
#endif
#define VAR_GLOBAL extern

/*----------------------------------- Macros ---------------------------------*/

/*-------------------------------- Enumerations ------------------------------*/

/*------------------------------------ Types ---------------------------------*/

/*---------------------------------- Constants -------------------------------*/

/*----------------------------------- Globals --------------------------------*/

/*---------------------------------- Interface -------------------------------*/

tBoolean Nav_Init ( void );
void Navigate ( void );
int Nav_Bearing ( void );
unsigned int Nav_Distance ( void );
unsigned int Nav_WaypointIndex ( void );

